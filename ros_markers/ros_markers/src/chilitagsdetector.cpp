#include "chilitagsdetector.hpp"

using namespace std;
using namespace cv;

// how many second in the *future* the markers transformation should be published?
// this allow to compensate for the 'slowness' of tag detection, but introduce
// some lag in TF.
#define TRANSFORM_FUTURE_DATING 0

ChilitagsDetector::ChilitagsDetector(ros::NodeHandle& rosNode,
                                     const string& configFilename,
                                     bool omitOtherTags,
                                     double tagSize) :
            rosNode(rosNode),
            it(rosNode),
            firstUncalibratedImage(true),
#ifdef WITH_KNOWLEDGE
            connector("localhost", "6969"),
#endif
            chilitags3d(cv::Size(0,0)) // will call setDefaultTagSize with default chilitags parameter values

{
#ifdef WITH_KNOWLEDGE
    try {
        kb = oro::Ontology::createWithConnector(connector);
    } catch (oro::OntologyServerException ose) {
        ROS_ERROR_STREAM("Could not connect to the knowledge base: " << ose.what());
        exit(1);
    }
#endif

    sub = it.subscribeCamera("image", 1, &ChilitagsDetector::findMarkers, this);

    if(!configFilename.empty()) {
    	chilitags3d.readTagConfiguration(configFilename, omitOtherTags);
    }

    if(tagSize!=USE_CHILITAGS_DEFAULT_PARAM)
        chilitags3d.setDefaultTagSize(tagSize); // use specified value

}


void ChilitagsDetector::setROSTransform(Matx44d trans, tf::Transform& transform)
{
    transform.setOrigin( tf::Vector3( trans(0,3) / 1000,
                                    trans(1,3) / 1000,
                                    trans(2,3) / 1000) );

    tf::Quaternion qrot;
    tf::Matrix3x3 mrot(
        trans(0,0), trans(0,1), trans(0,2),
        trans(1,0), trans(1,1), trans(1,2),
        trans(2,0), trans(2,1), trans(2,2));
    mrot.getRotation(qrot);
    transform.setRotation(qrot);
}

void ChilitagsDetector::findMarkers(const sensor_msgs::ImageConstPtr& msg, 
                                    const sensor_msgs::CameraInfoConstPtr& camerainfo)
{
    // updating the camera model is cheap if not modified
    cameramodel.fromCameraInfo(camerainfo);
    // publishing uncalibrated images? -> return (according to CameraInfo message documentation,
    // K[0] == 0.0 <=> uncalibrated).
    if(cameramodel.intrinsicMatrix()(0,0) == 0.0) {
        if(firstUncalibratedImage) {
            ROS_ERROR("Camera publishes uncalibrated images. Can not detect markers.");
            ROS_WARN("Detection will start over again when camera info is available.");
        }
        firstUncalibratedImage = false;
        return;
    }
    firstUncalibratedImage = true;
    // TODO: can we avoid to re-set the calibration matrices for every frame? ie,
    // how to know that the camera info has changed?
    chilitags3d.setCalibration(cameramodel.intrinsicMatrix(), 
                                cameramodel.distortionCoeffs());

    // hopefully no copy here:
    //  - assignement operator of cv::Mat does not copy the data
    //  - toCvShare does no copy if the default (source) encoding is used.
    inputImage = cv_bridge::toCvShare(msg)->image; 

        /********************************************************************
        *                      Markers detection                           *
        ********************************************************************/

    auto foundObjects = chilitags3d.estimate(inputImage);
    ROS_INFO_STREAM(foundObjects.size() << " objects found.");

    /****************************************************************
    *                Publish TF transforms                          *
    *****************************************************************/

#ifdef WITH_KNOWLEDGE
    auto previouslySeen(objectsSeen);
#endif
    objectsSeen.clear();

    for (auto& kv : foundObjects) {

        objectsSeen.insert(kv.first);
        setROSTransform(kv.second, 
                        transform);

        br.sendTransform(
                tf::StampedTransform(transform, 
                                        ros::Time::now() + ros::Duration(TRANSFORM_FUTURE_DATING), 
                                        cameramodel.tfFrame(),
                                        kv.first));
    }

#ifdef WITH_KNOWLEDGE
    set<oro::Statement> stmts;
    for(const auto& obj : objectsSeen) {
        if (previouslySeen.find(obj) == previouslySeen.end()) {
            stmts.insert(obj + " isVisible true");
            stmts.insert(obj + " rdf:type FiducialMarker");
        }
    }
    if (!stmts.empty()) kb->add(stmts);

    stmts.clear();
    for (const auto& pobj : previouslySeen) {
        if (objectsSeen.find(pobj) == objectsSeen.end()) {
            stmts.insert(pobj + " isVisible true");
        }
    }
    if (!stmts.empty()) kb->remove(stmts);
#endif


}

