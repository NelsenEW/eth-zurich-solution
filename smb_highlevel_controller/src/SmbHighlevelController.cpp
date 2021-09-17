#include <cmath>
#include <algorithm>
#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) :
		nodeHandle_(nodeHandle), subscriberQueueSize_(10), 
		scanTopic_("/scan"), tfListener_(tfBuffer_){
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	scanSubscriber_ = nodeHandle_.subscribe(scanTopic_, subscriberQueueSize_,
			&SmbHighlevelController::scanCallback, this);
	pclSubscriber_ = nodeHandle_.subscribe("/rslidar_points",1, &SmbHighlevelController::pclCallback, this);

	cmdPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 0);
    markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
    startServer_ = nodeHandle_.advertiseService(serviceName_, &SmbHighlevelController::startCallback, this);
}

SmbHighlevelController::~SmbHighlevelController() {
}

bool SmbHighlevelController::readParameters() {
	bool success = true;
    // Access parameter with global name
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/scan_subscriber_topic_name", scanTopic_);
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/scan_subscriber_queue_size",
			subscriberQueueSize_);
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/x_vel", xVel_);
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/p_gain_ang", kpAng_);
    success &= nodeHandle_.getParam(
            "/smb_highlevel_controller/service_name", serviceName_);
    success &= nodeHandle_.getParam(
            "/smb_highlevel_controller/start_robot", isStart_);
	return success;
}

/* bonus task solution */
void SmbHighlevelController::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO_STREAM_THROTTLE(2.0, "Number of points in 3D cloud: " << msg->height * msg->row_step);
}

void SmbHighlevelController::scanCallback(
		const sensor_msgs::LaserScan::ConstPtr &msg){
    geometry_msgs::PoseStamped goalPose, goalPoseOdom;
    getGoalPose(goalPose, msg);
    transformOdom(goalPose, goalPoseOdom);
    moveToGoal(goalPose);
    visMarkerPublish(goalPoseOdom);
}

void SmbHighlevelController::getGoalPose(geometry_msgs::PoseStamped &goalPose, 
		const sensor_msgs::LaserScan::ConstPtr &msg){
    auto minDistanceIterator = std::min_element(msg->ranges.begin(), msg->ranges.end());
    auto minDistanceIndex = std::distance(msg->ranges.begin(), minDistanceIterator);
    const auto distance = *minDistanceIterator;
	const auto angle = msg->angle_min + msg->angle_increment * minDistanceIndex; // in radian

    goalPose.header.frame_id = "rslidar";
    goalPose.header.stamp = ros::Time(0);
    goalPose.pose.position.x = distance * cos(angle);
    goalPose.pose.position.y = distance * sin(angle);
    goalPose.pose.position.z = 0;
    goalPose.pose.orientation.x = 0.0;
    goalPose.pose.orientation.y = 0.0;
    goalPose.pose.orientation.z = 0.0;
    goalPose.pose.orientation.w = 1.0; 

    ROS_INFO_STREAM_THROTTLE(2.0,"Minimum range [m]: " << distance);
    ROS_INFO_STREAM_THROTTLE(2.0,"Angle from the robot [radian]: " <<  angle);
}

void SmbHighlevelController::transformOdom(geometry_msgs::PoseStamped & pose, geometry_msgs::PoseStamped & targetPose){
    try{
        targetPose = tfBuffer_.transform(pose, "odom", ros::Duration(0));
    } catch (tf2::TransformException &exception) {
        ROS_WARN("%s", exception.what());
    }
}
void SmbHighlevelController::moveToGoal(const geometry_msgs::PoseStamped &goalPose){
    geometry_msgs::Twist velMsg;
    if (isStart_){
        velMsg.linear.x = xVel_;
        velMsg.angular.z = kpAng_ * atan2(goalPose.pose.position.y, goalPose.pose.position.x);
    }
    cmdPublisher_.publish(velMsg);
    ROS_DEBUG_STREAM_THROTTLE(2.0, "velMsg.linear.x : " << velMsg.linear.x);
	ROS_DEBUG_STREAM_THROTTLE(2.0, "velMsg.angular.z : " << velMsg.angular.z);
}

void SmbHighlevelController::visMarkerPublish(const geometry_msgs::PoseStamped &goalPose){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "smb_highlevel_controller";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = goalPose.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markerPublisher_.publish( marker );
}

bool SmbHighlevelController::startCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp){
    resp.success = true;
    if (isStart_ != req.data){
        isStart_ = req.data;
        resp.message = std::string("Set SMB to ") + ((req.data) ? "start" : "stop");
        ROS_WARN("SMB %s service called: %s", serviceName_.c_str(),resp.message.c_str()); // Another way to log using C style char
    }
    return resp.success;
}

}  // namespace smb_highlevel_controller
