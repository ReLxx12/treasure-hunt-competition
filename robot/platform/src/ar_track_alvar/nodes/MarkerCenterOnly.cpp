/*
 * @Description: output marker center only
 * @version: v1.00
 */
#include <std_msgs/Bool.h>
#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <ar_track_alvar/ParamsConfig.h>

using namespace alvar;
using namespace std;

Camera *cam;
cv_bridge::CvImagePtr cv_ptr_;
image_transport::Subscriber cam_sub_;
ros::Publisher markerCenterPub_;
ar_track_alvar_msgs::AlvarMarkers arPoseMarkers_;
MarkerDetector<MarkerData> marker_detector;

double max_frequency = 8.0;
double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam_image_topic;
std::string cam_info_topic;
int marker_resolution = 5; // default marker resolution
int marker_margin = 2;     // default marker margin

void getCapCallback(const sensor_msgs::ImageConstPtr &image_msg)
{
  // If we've already gotten the cam info, then go ahead
  if (cam->getCamInfo_)
  {
    try
    {
      // Convert the image
      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

      // Get the estimated pose of the main markers by using all the markers in each bundle
      marker_detector.Detect(
          cv_ptr_->image, cam, true, false,
          max_new_marker_error, max_track_error, CVSEQ, true);

      arPoseMarkers_.markers.clear();
      for (size_t i = 0; i < marker_detector.markers->size(); i++)
      {
        // Get the marker id
        int id = (*(marker_detector.markers))[i].GetId();

        // Create the marker messages
        ar_track_alvar_msgs::AlvarMarker ar_pose_marker;
        std::vector<PointDouble> &corners = (*(marker_detector.markers))[i].marker_corners_img;
        ar_pose_marker.center.x = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
        ar_pose_marker.center.y = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
        ar_pose_marker.header.stamp = image_msg->header.stamp;
        ar_pose_marker.id = id;

        // push to arPoseMarkers_
        arPoseMarkers_.markers.push_back(ar_pose_marker);
      }

      // publish to topic
      arPoseMarkers_.header.stamp = image_msg->header.stamp;
      markerCenterPub_.publish(arPoseMarkers_);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Error in ar_track_alvar callback: %s", e.what());
    }
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "marker_center");
  ros::NodeHandle n, pn("~");

  // Get params from ros param server.
  pn.param("marker_size", marker_size, 10.0);
  pn.param("max_new_marker_error", max_new_marker_error, 0.08);
  pn.param("max_track_error", max_track_error, 0.2);
  pn.param("max_frequency", max_frequency, 8.0);
  pn.setParam("max_frequency", max_frequency); // in case it was not set.
  pn.param("marker_resolution", marker_resolution, 5);
  pn.param("marker_margin", marker_margin, 2);

  // Camera input topics. Use remapping to map to your camera topics.
  cam_image_topic = "camera_image";
  cam_info_topic = "camera_info";

  // Set dynamically configurable parameters so they don't get replaced by
  // default values
  pn.setParam("marker_size", marker_size);
  pn.setParam("max_new_marker_error", max_new_marker_error);
  pn.setParam("max_track_error", max_track_error);
  marker_detector.SetMarkerSize(marker_size, marker_resolution, marker_margin);

  // Run at the configured rate, discarding pointcloud msgs if necessary
  ros::Rate rate(max_frequency);

  // Create a new camera object using cam_info_topic
  cam = new Camera(n, cam_info_topic);

  // New topic "marker_center" publisher
  markerCenterPub_ = n.advertise<ar_track_alvar_msgs::AlvarMarkers>("marker_center", 0);

  // Give tf a chance to catch up before the camera callback starts asking for
  // transforms
  // It will also reconfigure parameters for the first time, setting the default
  // values
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  image_transport::ImageTransport it_(n);
  cam_sub_ = it_.subscribe(cam_image_topic, 1, &getCapCallback); // Subscribe the topic

  // loop
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
