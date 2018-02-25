#include <string>
#include <set>

// chilitags
#include <chilitags/chilitags.hpp>

// opencv2
#include <opencv2/core/core.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#ifdef WITH_KNOWLEDGE
#include <liboro/oro.h>
#include <liboro/socket_connector.h>
#endif

#define USE_CHILITAGS_DEFAULT_PARAM -1

class ChilitagsDetector
{
public:
    /**
       Creates an object ready to find the 3D pose of chilitags.

 \param rosNode The node which has created the object.
 \param camaera_frame The name of the camera frame for projection onto.
 \param configFilename The name of the YAML configuration file describing rigid
        clusters of tags. The chilitags library is distributed with a sample
        configuration file documenting the expected format.
 \param omitOtherTags If true, ignore the tags that are not explicitly
        listed in the configuration file. If false (default), the 3D pose of all detected
        tags will be estimated.
 \param defaultTagSize The default size of tags (used to compute their 3D pose) when not
        explicitly specified in the configuration file. To be accurate, the unit
        must match the unit used for the camera calibration (usually, millimetres).

        The default value is 20. A value of -1 will cause the default value from
        the chilitags3d library to be used.

        Note that it assumes all the tags have the same size. If tags have
        different size, you may want to list them in the configuration file.

*/
    ChilitagsDetector(ros::NodeHandle& rosNode,
                      const std::string& configFilename = "",
                      bool omitOtherTags = false,
                      double tagSize = USE_CHILITAGS_DEFAULT_PARAM);

private:

#ifdef WITH_KNOWLEDGE
    oro::SocketConnector connector;
    oro::Ontology* kb;
#endif

    ros::NodeHandle& rosNode;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber sub;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    image_geometry::PinholeCameraModel cameramodel;
    cv::Mat cameraMatrix, distCoeffs;
    bool firstUncalibratedImage;

    cv::Mat inputImage;
    chilitags::Chilitags3D chilitags3d;
    std::set<std::string> objectsSeen;

    void setROSTransform(cv::Matx44d trans, tf::Transform& transform);

    void findMarkers(const sensor_msgs::ImageConstPtr& msg,
                     const sensor_msgs::CameraInfoConstPtr& camerainfo);
};

