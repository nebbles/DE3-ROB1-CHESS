#!/usr/bin/env python

from __future__ import print_function
import rosbag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

import os
import numpy as np
import cv2
import struct
import csv
import socket


class RGBDExporter:
    def __init__(self, node_name):
        try:
            rospy.init_node(node_name, anonymous=True)
        except socket.error as e:
            raise UserWarning("roscore not running")

        # read parameters
        try:
            # input/output paths
            bag_file_path = rospy.get_param('~bag_file')
            self.export_path = rospy.get_param('~export_dir')
            # image topics with CompressedImage
            self.topic_rgb = rospy.get_param('~topic_rgb', default="/camera/rgb/image_rect_color/compressed")
            self.topic_depth = rospy.get_param('~topic_depth', default="/camera/depth/image_rect_raw/compressedDepth")
            # topic with JointState
            self.topic_joints = rospy.get_param('~topic_joints', default="/joint_states")
        except KeyError as e:
            raise UserWarning(e.message+" is undefined")

        self.topics = [self.topic_rgb, self.topic_depth, self.topic_joints]

        bag_file_path = os.path.expanduser(bag_file_path)
        print("reading:",bag_file_path)
        self.bag = rosbag.Bag(bag_file_path, mode='r')
        print("duration:", self.bag.get_end_time()-self.bag.get_start_time(),"s")

        self.export_path = os.path.expanduser(self.export_path)
        self.path_colour = os.path.join(self.export_path, "colour")
        self.path_depth = os.path.join(self.export_path, "depth")
        self.path_joint_values = os.path.join(self.export_path, "joints.csv")
        self.path_ref_time = os.path.join(self.export_path, "time.csv")

        self.cvbridge = CvBridge()

    def export(self):
        print("exporting to: "+self.export_path)
        if os.path.exists(self.export_path):
            raise UserWarning("path "+self.export_path+" already exists!")

        # create export directories
        for dir_path in [self.path_colour, self.path_depth]:
            if not os.path.exists(dir_path):
                os.makedirs(dir_path)

        time_csv = csv.writer(open(self.path_ref_time, 'w'), delimiter=' ')

        end_time = None
        # for exporting subpart of log, select end time in seconds, e.g. 10 seconds from start of log
        # end_time = rospy.Time.from_sec(self.bag.get_start_time() + 10)

        # get timestamps for all messages to select reference topic with smallest amount of messages
        # get available joint names and their oldest value, e.g. we are looking into the future and assume
        # that the first seen joint value reflects the state of the joint before this time
        times = dict()
        for t in self.topics:
            times[t] = []
        full_jnt_values = dict() # store first (oldest) joint values of complete set
        for topic, msg, t in self.bag.read_messages(topics=self.topics, end_time=end_time):
            # get set of joints
            if topic == self.topic_joints:
                for ijoint in range(len(msg.name)):
                    if msg.name[ijoint] not in full_jnt_values:
                        full_jnt_values[msg.name[ijoint]] = msg.position[ijoint]
            times[topic].append(msg.header.stamp)

        print("joints:", full_jnt_values.keys())

        # remove topics with no messages
        [(times.pop(top, None), self.topics.remove(top)) for top in times.keys() if len(times[top])==0]

        if len(full_jnt_values.keys())>0:
            joint_csv = csv.writer(open(self.path_joint_values, 'w'), delimiter=' ')
            # write joint names
            full_joint_list_sorted = sorted(full_jnt_values.keys())
            joint_csv.writerow(full_joint_list_sorted)

        # get reference time and topic
        ref_topic = ""
        min_len = np.inf
        for topic in times.keys():
            # find topic with smallest amount of messages as reference
            if len(times[topic]) < min_len:
                ref_topic = topic
                min_len = len(times[topic])

        print("reference topic: "+ref_topic+" ("+str(min_len)+" messages)")

        # sample and hold synchronisation
        sync_msg = dict()
        for top in self.topics:
            sync_msg[top] = None
        has_all_msg = False

        for topic, msg, t in self.bag.read_messages(topics=self.topics, end_time=end_time):
            # merge all received joints
            if topic == self.topic_joints:
                for ijoint in range(len(msg.name)):
                    full_jnt_values[msg.name[ijoint]] = msg.position[ijoint]
                    sync_msg[topic] = full_jnt_values
            else:
                sync_msg[topic] = msg

            # export at occurrence of reference message and if all remaining messages have been received
            # e.g. we export the reference message and the newest messages of other topics
            if topic==ref_topic and (has_all_msg or all([v is not None for v in sync_msg.values()])):
                # faster evaluation of previous statement
                has_all_msg = True

                # export
                ref_time = msg.header.stamp
                time_csv.writerow([ref_time])

                for sync_topic in sync_msg.keys():
                    if sync_topic == self.topic_joints:
                        # export full joints, sorted by joint name
                        jvalues = []
                        for jname in full_joint_list_sorted:
                            jvalues.append(sync_msg[sync_topic][jname])
                        joint_csv.writerow(jvalues)

                    elif sync_topic == self.topic_rgb:
                        # export RGB
                        if msg._type == Image._type:
                            colour_img = self.cvbridge.imgmsg_to_cv2(sync_msg[sync_topic])
                        elif msg._type == CompressedImage._type:
                            colour_img = self.cvbridge.compressed_imgmsg_to_cv2(sync_msg[sync_topic])
                        else:
                            print("unsupported:",msg._type)
                        cv2.imwrite(os.path.join(self.path_colour, "colour_" + str(ref_time) + ".png"), colour_img)

                    elif sync_topic == self.topic_depth and msg._type == CompressedImage._type:
                        # export depth
                        depth_fmt, compr_type = sync_msg[sync_topic].format.split(';')
                        # remove white space
                        depth_fmt = depth_fmt.strip()
                        compr_type = compr_type.strip()

                        if compr_type == "compressedDepth":
                            # remove header from raw data
                            # C header definition at:
                            # /opt/ros/indigo/include/compressed_depth_image_transport/compression_common.h
                            # enum compressionFormat {
                            #   UNDEFINED = -1, INV_DEPTH
                            # };
                            # struct ConfigHeader {
                            #   compressionFormat format;
                            #   float depthParam[2];
                            # };
                            # header size = enum (4 byte) + float[2] (2 x 4 byte) = 12 byte
                            # enum size may vary and needs to be adapted if decoding fails
                            depth_header_size = 12
                            raw_data = sync_msg[sync_topic].data[depth_header_size:]

                            depth_img_raw = cv2.imdecode(np.fromstring(raw_data, np.uint8), cv2.CV_LOAD_IMAGE_UNCHANGED)
                            if depth_img_raw is None:
                                # probably wrong header size
                                raise Exception("Could not decode compressed depth image."
                                                "You may need to change 'depth_header_size'!")

                            if depth_fmt == "16UC1":
                                # write raw image data
                                depth_img = depth_img_raw
                            elif depth_fmt == "32FC1":
                                raw_header = sync_msg[sync_topic].data[:depth_header_size]
                                # header: int, float, float
                                [compfmt, depthQuantA, depthQuantB] = struct.unpack('iff', raw_header)
                                depth_img_scaled = depthQuantA / (depth_img_raw.astype(np.float32) - depthQuantB)
                                # filter max values
                                depth_img_scaled[depth_img_raw == 0] = 0

                                # depth_img_scaled provides distance in meters as f32
                                # for storing it as png, we need to convert it to 16UC1 again (depth in mm)
                                depth_img_mm = (depth_img_scaled * 1000).astype(np.uint16)
                                depth_img = depth_img_mm
                            else:
                                raise Exception("Decoding of '"+sync_msg[sync_topic].format+"' is not implemented!")

                        else:
                            if depth_fmt == "16UC1":
                                # assume that all 16bit image representations can be decoded by opencv
                                rawimgdata = sync_msg[sync_topic].data
                                depth_img = cv2.imdecode(np.fromstring(rawimgdata, np.uint8), cv2.CV_LOAD_IMAGE_UNCHANGED)
                            else:
                                raise Exception("Decoding of '" + sync_msg[sync_topic].format + "' is not implemented!")

                        # write image
                        cv2.imwrite(os.path.join(self.path_depth, "depth_" + str(ref_time) + ".png"), depth_img)

                    elif sync_topic == self.topic_depth and msg._type == Image._type:
                        depth_img = self.cvbridge.imgmsg_to_cv2(sync_msg[sync_topic])
                        cv2.imwrite(os.path.join(self.path_depth, "depth_" + str(ref_time) + ".png"), depth_img)

        print("done")

    def __del__(self):
        # delete all private parameters
        try:
            rospy.delete_param(rospy.get_name())
        except KeyError:
            pass
        except socket.error:
            pass

        # close log file
        try:
            self.bag.close()
        except AttributeError:
            pass


if __name__ == '__main__':
    try:
        exporter = RGBDExporter("rgbd_exporter")
        exporter.export()
    except UserWarning as e:
        print(e.message)
        exit()
