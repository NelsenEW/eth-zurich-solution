/*
  * Copyright 2013 Open Source Robotics Foundation
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

#include <string>
#include <tf/tf.h>
#include <stdlib.h>

#include "smb_gazebo/OdometrySource.hpp"
#include <ignition/math/Rand.hh>

namespace gazebo
{

// Constructor
OdometrySource::OdometrySource()
{
}

// Destructor
OdometrySource::~OdometrySource()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  this->p3d_queue_.clear();
  this->p3d_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

// Load the controller
void OdometrySource::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
            _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("odom_plugin", "plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("odom_plugin", "plugin error: bodyName: %s does not exist\n",
                    this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("odomFrame"))
  {
    ROS_FATAL_NAMED("odom_plugin", "plugin missing <odomFrame>, cannot proceed");
    return;
  }
  else
    this->odom_frame_ = _sdf->GetElement("odomFrame")->Get<std::string>();

  if (!_sdf->HasElement("childOdomFrame"))
  {
    ROS_FATAL_NAMED("odom_plugin", "plugin missing <childOdomFrame>, cannot proceed");
    return;
  }
  else
    this->child_odom_frame_ = _sdf->GetElement("childOdomFrame")->Get<std::string>();

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("odom_plugin", "plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("publishTf"))
  {
    ROS_FATAL_NAMED("odom_plugin", "plugin missing <publishTf>, default to false");
    this->publish_tf_ = false;
  }
  else
    this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();

  if (!_sdf->HasElement("xyzOffset"))
  {
    ROS_INFO_NAMED("odom_plugin", "plugin missing <xyzOffset>, defaults to 0s");
    this->offset_.Pos() = ignition::math::Vector3d(0, 0, 0);
  }
  else
    this->offset_.Pos() = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();

  if (!_sdf->HasElement("rpyOffset"))
  {
    ROS_INFO_NAMED("odom_plugin", "plugin missing <rpyOffset>, defaults to 0s");
    this->offset_.Rot() = ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 0));
  }
  else
    this->offset_.Rot() = ignition::math::Quaterniond(_sdf->GetElement("rpyOffset")->Get<ignition::math::Vector3d>());

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_INFO_NAMED("odom_plugin", "plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO_NAMED("odom_plugin", "plugin missing <updateRate>, defaults to 0.0"
                           " (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

  if (_sdf->HasElement("maxLinearSpeedBeforeSettle")){
        this->max_linear_speed_before_settle_ = _sdf->GetElement("maxLinearSpeedBeforeSettle")->Get<double>();
  }
  if (_sdf->HasElement("maxAngularSpeedBeforeSettle")){
        this->max_angular_speed_before_settle_ = _sdf->GetElement("maxAngularSpeedBeforeSettle")->Get<double>();
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("odom_plugin", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_STREAM("Odom plugin initialized: " << std::endl
    << "odom frame:       " << this->odom_frame_ << std::endl
    << "child odom frame: " << this->child_odom_frame_ << std::endl
    << "sensor link:      " << this->link_name_ << std::endl
    << "topic name:       " << this->topic_name_ << std::endl
    << "publish tf:       " << this->publish_tf_);

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  if (this->topic_name_ != "")
  {
    this->pub_Queue = this->pmq.addPub<nav_msgs::Odometry>();
    this->pub_ =
            this->rosnode_->advertise<nav_msgs::Odometry>(this->topic_name_, 1);
  }

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif

  // start custom queue for p3d
  this->callback_queue_thread_ = boost::thread(
          boost::bind(&OdometrySource::P3DQueueThread, this));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&OdometrySource::UpdateChild, this));

}

// Update the controller
void OdometrySource::UpdateChild()
{
  if (!this->link_)
    return;

#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = this->world_->SimTime();
#else
  common::Time cur_time = this->world_->GetSimTime();
#endif

  double dt = (cur_time - this->last_time_).Double();
  if(!settled_){
    if (dt > 1.0){
      ignition::math::Vector3d linvel = this->link_->WorldLinearVel();
      ignition::math::Vector3d angvel = this->link_->WorldAngularVel();
      if (linvel.Length() < max_angular_speed_before_settle_ &&
        angvel.Length() < max_angular_speed_before_settle_){

        this->odom_pose_ = this->link_->WorldPose(); // world to odom
        ignition::math::Vector3d rpy = this->odom_pose_.Rot().Euler();
        double yaw = rpy.Z();
        ignition::math::Quaterniond odomRotation(0.0, 0.0, yaw);
        this->odom_pose_.Rot() = odomRotation;
        settled_ = true;
        ROS_INFO_STREAM("Odometry frame initialized at: " << this->odom_pose_);
        return;
      }
      ROS_INFO_STREAM("Wait to settle before init odometry frame. Sleep for 1.0 sec. Speed is "<<linvel.Length()<<" (max is "<<max_linear_speed_before_settle_<<"); "<<angvel.Length()<<" (max is "<<max_angular_speed_before_settle_<<")");
      this->last_time_ = cur_time;
    }
    return;
  }

  if (cur_time < this->last_time_)
  {
    ROS_WARN_NAMED("odom_plugin", "Negative update time difference detected.");
    this->last_time_ = cur_time;
  }

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  // differentiate to get accelerations
  double tmp_dt = cur_time.Double() - this->last_time_.Double();
  if (tmp_dt != 0)
  {
    this->lock.lock();

    if (this->topic_name_ != "")
    {
      // copy data into pose message
      this->pose_msg_.header.frame_id = this->odom_frame_;
      this->pose_msg_.header.stamp.sec = cur_time.sec;
      this->pose_msg_.header.stamp.nsec = cur_time.nsec;

      this->pose_msg_.child_frame_id = this->child_odom_frame_;

      ignition::math::Pose3d pose;

      // get inertial Rates
      // Get Pose/Orientation
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Vector3d vpos = this->link_->WorldLinearVel();
      ignition::math::Vector3d veul = this->link_->WorldAngularVel();
      pose = this->link_->WorldPose();
#else
      ignition::math::Vector3d vpos = this->link_->GetWorldLinearVel().Ign();
      ignition::math::Vector3d veul = this->link_->GetWorldAngularVel().Ign();
      pose = this->link_->GetWorldPose().Ign();
#endif
      pose.Pos() = pose.Pos() - odom_pose_.Pos();
      pose.Pos() = odom_pose_.Rot().RotateVectorReverse(pose.Pos());
      pose.Rot() = odom_pose_.Rot().Inverse() * pose.Rot();    // R_o_p =  R_o_w  *  R_w_p

      vpos = odom_pose_.Rot().RotateVector(vpos);
      veul = odom_pose_.Rot().RotateVector(veul);

      vpos = pose.Rot().RotateVectorReverse(vpos);
      veul = pose.Rot().RotateVectorReverse(veul);

      // Apply Constant Offsets
      // apply xyz offsets and get position and rotation components
      pose.Pos() = pose.Pos() + this->offset_.Pos();
      // apply rpy offsets
      pose.Rot() = this->offset_.Rot()*pose.Rot();
      pose.Rot().Normalize();

      // Fill out messages
      this->pose_msg_.pose.pose.position.x    = pose.Pos().X();
      this->pose_msg_.pose.pose.position.y    = pose.Pos().Y();
      this->pose_msg_.pose.pose.position.z    = pose.Pos().Z();

      this->pose_msg_.pose.pose.orientation.x = pose.Rot().X();
      this->pose_msg_.pose.pose.orientation.y = pose.Rot().Y();
      this->pose_msg_.pose.pose.orientation.z = pose.Rot().Z();
      this->pose_msg_.pose.pose.orientation.w = pose.Rot().W();

      this->pose_msg_.twist.twist.linear.x  = vpos.X() +
                                              this->GaussianKernel(0, this->gaussian_noise_);
      this->pose_msg_.twist.twist.linear.y  = vpos.Y() +
                                              this->GaussianKernel(0, this->gaussian_noise_);
      this->pose_msg_.twist.twist.linear.z  = vpos.Z() +
                                              this->GaussianKernel(0, this->gaussian_noise_);
      // pass euler angular rates
      this->pose_msg_.twist.twist.angular.x = veul.X() +
                                              this->GaussianKernel(0, this->gaussian_noise_);
      this->pose_msg_.twist.twist.angular.y = veul.Y() +
                                              this->GaussianKernel(0, this->gaussian_noise_);
      this->pose_msg_.twist.twist.angular.z = veul.Z() +
                                              this->GaussianKernel(0, this->gaussian_noise_);

      // fill in covariance matrix
      double gn2 = this->gaussian_noise_*this->gaussian_noise_;
      this->pose_msg_.pose.covariance[0] = gn2;
      this->pose_msg_.pose.covariance[7] = gn2;
      this->pose_msg_.pose.covariance[14] = gn2;
      this->pose_msg_.pose.covariance[21] = gn2;
      this->pose_msg_.pose.covariance[28] = gn2;
      this->pose_msg_.pose.covariance[35] = gn2;

      this->pose_msg_.twist.covariance[0] = gn2;
      this->pose_msg_.twist.covariance[7] = gn2;
      this->pose_msg_.twist.covariance[14] = gn2;
      this->pose_msg_.twist.covariance[21] = gn2;
      this->pose_msg_.twist.covariance[28] = gn2;
      this->pose_msg_.twist.covariance[35] = gn2;

      // publish to ros
      this->pub_Queue->push(this->pose_msg_, this->pub_);


      if (this->publish_tf_) {
        odomTf_.header.stamp = ros::Time::now();
        odomTf_.header.frame_id = this->odom_frame_;
        odomTf_.child_frame_id = this->child_odom_frame_;
        odomTf_.transform.translation.x = pose.Pos().X();
        odomTf_.transform.translation.y = pose.Pos().Y();
        odomTf_.transform.translation.z = pose.Pos().Z();
        odomTf_.transform.rotation.x = pose.Rot().X();
        odomTf_.transform.rotation.y = pose.Rot().Y();
        odomTf_.transform.rotation.z = pose.Rot().Z();
        odomTf_.transform.rotation.w = pose.Rot().W();
        tfBroadcaster_.sendTransform(odomTf_);
      }

      this->lock.unlock();

      // save last time stamp
      this->last_time_ = cur_time;
    }
  }
}

// Utility for adding noise
double OdometrySource::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = ignition::math::Rand::DblUniform();

  // normalized uniform random variable
  double V = ignition::math::Rand::DblUniform();

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

// Put laser data to the interface
void OdometrySource::P3DQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->p3d_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(OdometrySource);
} // namespace gazebo