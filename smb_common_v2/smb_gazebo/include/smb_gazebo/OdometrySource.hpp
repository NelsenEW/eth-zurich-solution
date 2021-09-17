/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
*/

#ifndef GAZEBO_ROS_P3D_HH
#define GAZEBO_ROS_P3D_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_plugins/PubQueue.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace gazebo
{
class OdometrySource : public ModelPlugin
{
public:
  OdometrySource();

  virtual ~OdometrySource();

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  virtual void UpdateChild();

private:
  physics::WorldPtr world_;
  physics::ModelPtr model_;

  physics::LinkPtr link_;

  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;

  nav_msgs::Odometry pose_msg_;

  std::string link_name_;

  std::string topic_name_;

  std::string odom_frame_;
  std::string child_odom_frame_;

  ignition::math::Pose3d offset_;
  ignition::math::Pose3d odom_pose_;

  // wait sensor to settle before spawning odom frame
  bool settled_ = false;
  double wait_to_settle_sec_ = 5.0;
  double max_linear_speed_before_settle_ = 1e-3;
  double max_angular_speed_before_settle_ = 1e-2;

  boost::mutex lock;
  common::Time last_time_;

  // tf
  bool publish_tf_;
  geometry_msgs::TransformStamped odomTf_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  // rate control
  double update_rate_;

  double gaussian_noise_;

  double GaussianKernel(double mu, double sigma);

  std::string robot_namespace_;

  ros::CallbackQueue p3d_queue_;
  void P3DQueueThread();
  boost::thread callback_queue_thread_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;

  unsigned int seed;

  // ros publish multi queue, prevents publish() blocking
  PubMultiQueue pmq;
};
}
#endif