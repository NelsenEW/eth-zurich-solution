#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>
#include <sensor_msgs/PointCloud2.h>

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
};

}  // namespace smb_highlevel_controller
