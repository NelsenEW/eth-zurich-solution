#include <cmath>
#include <algorithm>
#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) :
		nodeHandle_(nodeHandle), subscriberQueueSize_(10), scanTopic_("/scan") {
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	scanSubscriber_ = nodeHandle_.subscribe(scanTopic_, subscriberQueueSize_,
			&SmbHighlevelController::scanCallback, this);
	pclSubscriber_ = nodeHandle_.subscribe("/rslidar_points",1, &SmbHighlevelController::pclCallback, this);
}

SmbHighlevelController::~SmbHighlevelController() {
}

bool SmbHighlevelController::readParameters() {
	bool success = true;
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/scan_subscriber_topic_name", scanTopic_);
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/scan_subscriber_queue_size",
			subscriberQueueSize_);
	return success;
}

/* bonus task solution */
void SmbHighlevelController::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& message){
    ROS_INFO_STREAM_THROTTLE(2.0, "Number of points in 3D cloud: " << message->height * message->row_step);
}

void SmbHighlevelController::scanCallback(
		const sensor_msgs::LaserScan::ConstPtr &msg) {
	double min = *std::min_element(msg->ranges.begin(), msg->ranges.end());
	ROS_INFO_STREAM_THROTTLE(2.0,"Minimum range [m]: " << min);
}

}  // namespace smb_highlevel_controller
