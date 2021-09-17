#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>

ros::ServiceClient stopClient;

std::string serviceName;
std::string scanTopic;

bool maxDistanceUpdate = false;
bool stopCalled = false;
bool priorCollision = false;

float maxDistanceToPillar;
float collisionThreshold;
float prevImuX;


void requestStop(){
    std_srvs::SetBool startSrv;
    startSrv.request.data = false;
    if (!stopClient.call(startSrv)){
        ROS_ERROR("Failed to request stop");
        ros::requestShutdown();
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr imuData){
    auto imuX = imuData->linear_acceleration.x;
    auto absChangeX = abs(imuX - prevImuX);
    if(absChangeX > collisionThreshold){
        ROS_WARN_ONCE("Change in imu x: %lf", absChangeX);
        requestStop();
    }
    prevImuX = imuX;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr laserMsg){
    auto minDistance = *std::min_element(laserMsg->ranges.begin(), laserMsg->ranges.end());
    // During startup, set the margin to range_min if the margin is less than the range_min
    if (maxDistanceToPillar < laserMsg->range_min && !maxDistanceUpdate){
        maxDistanceToPillar = laserMsg->range_min;
        maxDistanceUpdate = true;
    }

    if (minDistance  < maxDistanceToPillar){
        requestStop();
    }
}


bool getParam(ros::NodeHandle& nh){
    bool success = true;
    // Access parameters with private naming is also allowed!
    success &= nh.getParam("max_distance_to_pillar", maxDistanceToPillar);
    success &= nh.getParam("service_name", serviceName);
    success &= nh.getParam("scan_subscriber_topic_name", scanTopic);
    success &= nh.getParam("collision_threshold", collisionThreshold);
    success &= nh.getParam("prior_collision", priorCollision);
    return success;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "stop_if_near");
    ros::NodeHandle nodeHandle("~");   
    if (!getParam(nodeHandle)){
        ROS_ERROR("Failed to get one of the param");
        ros::requestShutdown();
    }
    ros::Duration(1).sleep(); // filtered the initial ramp in imu data due to spawn.
    stopClient = nodeHandle.serviceClient<std_srvs::SetBool>(serviceName);
    ros::Subscriber stopSub;
    if(priorCollision){
        stopSub = nodeHandle.subscribe("/scan", 10, scanCallback);
    }else{
        stopSub = nodeHandle.subscribe("/imu0", 10, imuCallback);
    }
    ros::spin();
    return 0;
}