#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string.h>

namespace smb_highlevel_controller {

/**
 * Class containing the SMB Highlevel Controller
 */
class SmbHighlevelController {
 public:
  /** Constructor */
  SmbHighlevelController(ros::NodeHandle& nodeHandle);

  /** Destructor */
  virtual ~SmbHighlevelController();

 private:
  bool readParameters();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /*bonus task solution */
  void pclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

  ros::NodeHandle nodeHandle_;
  ros::Subscriber scanSubscriber_;
  std::string scanTopic_;
  int subscriberQueueSize_;

  /*bonus task solution */
  ros::Subscriber pclSubscriber_;

  /*controller */
  float xVel_;
  float kpAng_;

  /*goal pose */  
  ros::Publisher cmdPublisher_;

  void getGoalPose(geometry_msgs::PoseStamped &goalPose, const sensor_msgs::LaserScan::ConstPtr &msg);
  void transformOdom(geometry_msgs::PoseStamped & pose, geometry_msgs::PoseStamped & targetPose);
  void moveToGoal(const geometry_msgs::PoseStamped &goalPose);

  /*TF listener*/
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  /*Viz marker*/
  ros::Publisher markerPublisher_;

  /*publish the visualization marker on the pillar*/
  void visMarkerPublish(const geometry_msgs::PoseStamped &goalPose);

};

}  // namespace smb_highlevel_controller
