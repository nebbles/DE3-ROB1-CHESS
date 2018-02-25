# rosbag exporter for compressed images and joint states

Synchronised export of compressed colour (topic compressed) and depth (topic compressedDepth) images, and joint states.

```
rosrun rgbd_export rgbd_sync_export.py \
  _bag_file:=~/rosbags/experiment1.bag \
  _export_dir:=~/rgb_log_exp/exp1 \
  _topic_rgb:=/camera/rgb/image_rect_color/compressed \
  _topic_depth:=/camera/depth/image_rect_raw/compressedDepth \
  _topic_joints:=/joint_states
```
