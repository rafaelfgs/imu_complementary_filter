/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

	@section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "imu_complementary_filter/complementary_filter_ros.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

namespace imu_tools {



ComplementaryFilterROS::ComplementaryFilterROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  
  nh_(nh),
  nh_private_(nh_private),
  initialized_filter_(false)

{

  ROS_INFO("Startingads ComplementaryFilterROS");
  initializeParams();

  int queue_size = 5;

  // Register publishers
  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(ros::names::resolve("imu") + "/data", queue_size);

  if (publish_debug_topics_)
  {
      rpy_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(ros::names::resolve("imu") + "/rpy/filtered", queue_size);

      if (filter_.getDoBiasEstimation())
      {
        state_publisher_ = nh_.advertise<std_msgs::Bool>(ros::names::resolve("imu") + "/steady_state", queue_size);
      }
  }

  // Register splitted data subscriber
  if (use_split_)
  {
    acc_subscriber_.reset(new ImuSubscriber(nh_, ros::names::resolve("imu") + "/accel", queue_size));
    gyr_subscriber_.reset(new ImuSubscriber(nh_, ros::names::resolve("imu") + "/gyro", queue_size));
    split_sync_.reset(new SplitSynchronizer(SplitSyncPolicy(queue_size), *acc_subscriber_, *gyr_subscriber_));
    split_sync_->registerCallback(boost::bind(&ComplementaryFilterROS::imuSplitCallback, this, _1, _2));
  }
  else
  {
    // Register magnetic data subscriber
    if (use_mag_)
    {
      imu_subscriber_.reset(new ImuSubscriber(nh_, ros::names::resolve("imu") + "/data_raw", queue_size));
      mag_subscriber_.reset(new MagSubscriber(nh_, ros::names::resolve("imu") + "/mag", queue_size));
      mag_sync_.reset(new MagSynchronizer(MagSyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
      mag_sync_->registerCallback(boost::bind(&ComplementaryFilterROS::imuMagCallback, this, _1, _2));
    }
    else
    {
      // Register IMU raw data subscriber
      imu_subscriber_.reset(new ImuSubscriber(nh_, ros::names::resolve("imu") + "/data_raw", queue_size));
      imu_subscriber_->registerCallback(&ComplementaryFilterROS::imuCallback, this);
    }
  }
}



ComplementaryFilterROS::~ComplementaryFilterROS()
{
  ROS_INFO("Destroying ComplementaryFilterROS");
}



void ComplementaryFilterROS::initializeParams()
{
  double gain_acc;
  double gain_mag;
  bool do_bias_estimation;
  double bias_alpha;
  bool do_adaptive_gain;

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam ("frame_id", frame_id_))
    frame_id_ = "";
  if (!nh_private_.getParam ("use_mag", use_mag_))
    use_mag_ = false;
  if (!nh_private_.getParam ("use_split", use_split_))
    use_split_ = false;
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = false;
  if (!nh_private_.getParam ("reverse_tf", reverse_tf_))
    reverse_tf_ = false;
  if (!nh_private_.getParam ("constant_dt", constant_dt_))
    constant_dt_ = 0.0;
  if (!nh_private_.getParam ("publish_debug_topics", publish_debug_topics_))
    publish_debug_topics_ = false;
  if (!nh_private_.getParam ("gain_acc", gain_acc))
    gain_acc = 0.01;
  if (!nh_private_.getParam ("gain_mag", gain_mag))
    gain_mag = 0.01;
  if (!nh_private_.getParam ("do_bias_estimation", do_bias_estimation))
    do_bias_estimation = true;
  if (!nh_private_.getParam ("bias_alpha", bias_alpha))
    bias_alpha = 0.01;
  if (!nh_private_.getParam ("do_adaptive_gain", do_adaptive_gain))
    do_adaptive_gain = true;

  double orientation_stddev;
  if (!nh_private_.getParam ("orientation_stddev", orientation_stddev))
    orientation_stddev = 0.0;

  orientation_variance_ = orientation_stddev * orientation_stddev;

  filter_.setDoBiasEstimation(do_bias_estimation);
  filter_.setDoAdaptiveGain(do_adaptive_gain);

  if(!filter_.setGainAcc(gain_acc))
    ROS_WARN("Invalid gain_acc passed to ComplementaryFilter.");
  if (use_mag_)
  {
    if(!filter_.setGainMag(gain_mag))
      ROS_WARN("Invalid gain_mag passed to ComplementaryFilter.");
  }
  if (do_bias_estimation)
  {
    if(!filter_.setBiasAlpha(bias_alpha))
      ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
  }

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0)
  {
    // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
    // otherwise, it will be constant
    ROS_WARN("constant_dt parameter is %f, must be >= 0.0. Setting to 0.0", constant_dt_);
    constant_dt_ = 0.0;
  }
}



// Update filter for IMU raw data
void ComplementaryFilterROS::imuCallback(const ImuMsg::ConstPtr& imu_msg_raw)
{
  const geometry_msgs::Vector3& a = imu_msg_raw->linear_acceleration;
  const geometry_msgs::Vector3& w = imu_msg_raw->angular_velocity;
  const ros::Time& time = imu_msg_raw->header.stamp;

  // Initialize
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // determine dt: either constant, or from IMU timestamp
  double dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - time_prev_).toSec();

  time_prev_ = time;

  // Update the filter
  filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

  // Publish state
  publish(imu_msg_raw);
}



void ComplementaryFilterROS::imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                                            const MagMsg::ConstPtr& mag_msg)
{
  const geometry_msgs::Vector3& a = imu_msg_raw->linear_acceleration;
  const geometry_msgs::Vector3& w = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& m = mag_msg->magnetic_field;
  const ros::Time& time = imu_msg_raw->header.stamp;

  // Initialize
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // Calculate dt
  double dt = (time - time_prev_).toSec();
  time_prev_ = time;
  
  // Update the filter
  if (std::isnan(m.x) || std::isnan(m.y) || std::isnan(m.z))
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
  else
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, m.x, m.y, m.z, dt);
  
  // Publish state
  publish(imu_msg_raw);
}



void ComplementaryFilterROS::imuSplitCallback(const ImuMsg::ConstPtr& acc_msg,
                                              const ImuMsg::ConstPtr& gyr_msg)
{
  // Variables within ang_vel, lin_acc and its covariance
  geometry_msgs::Vector3 av = gyr_msg->angular_velocity;
  geometry_msgs::Vector3 la = acc_msg->linear_acceleration;
  boost::array<double, 9ul> lac = acc_msg->linear_acceleration_covariance;
  
  // Fix wrong orientation (w.x, w.y and a.z)
  if (la.z < 0)
  {
    av.x = -av.x;
    av.y = -av.y;
    la.z = -la.z;
  }
  
  // Build IMU raw data
  ImuMsg::ConstPtr imu_msg_raw = gyr_msg;
  *(geometry_msgs::Vector3*)(&(imu_msg_raw->angular_velocity)) = av;
  *(geometry_msgs::Vector3*)(&(imu_msg_raw->linear_acceleration)) = la;
  *(boost::array<double, 9ul>*)(&(imu_msg_raw->linear_acceleration_covariance)) = lac;
  
  // After here it is the same as imuCallback
  const geometry_msgs::Vector3& a = imu_msg_raw->linear_acceleration;
  const geometry_msgs::Vector3& w = imu_msg_raw->angular_velocity;
  const ros::Time& time = imu_msg_raw->header.stamp;
  
  for (int i=0; i<9; i++)
  {
    orient_cov_matrix_[i] = sqrt(imu_msg_raw->linear_acceleration_covariance[i]*
                                 imu_msg_raw->linear_acceleration_covariance[i] + 
                                 imu_msg_raw->angular_velocity_covariance[i]*
                                 imu_msg_raw->angular_velocity_covariance[i]);
  }

  // Initialize
  if (!initialized_filter_)
  {
    time_prev_ = time;
    initialized_filter_ = true;
    return;
  }

  // Calculate dt
  double dt = (time - time_prev_).toSec();
  time_prev_ = time;

  // Update the filter
  filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
  
  // Publish state
  publish(imu_msg_raw);
}



tf::Quaternion ComplementaryFilterROS::hamiltonToTFQuaternion(
    double q0, double q1, double q2, double q3) const
{
  // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
  // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
  return tf::Quaternion(q1, q2, q3, q0);
}



void ComplementaryFilterROS::publish(
    const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  // Get the orientation:
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  tf::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

  // Create and publish fitlered IMU message.
  boost::shared_ptr<sensor_msgs::Imu> imu_msg =
      boost::make_shared<sensor_msgs::Imu>(*imu_msg_raw);
  tf::quaternionTFToMsg(q, imu_msg->orientation);
  
  if (frame_id_ != "")
  {
    imu_msg->header.frame_id = frame_id_;
  }
  
  if (orientation_variance_ != 0.0)
  {
  imu_msg->orientation_covariance[0] = orientation_variance_;
  imu_msg->orientation_covariance[1] = 0.0;
  imu_msg->orientation_covariance[2] = 0.0;
  imu_msg->orientation_covariance[3] = 0.0;
  imu_msg->orientation_covariance[4] = orientation_variance_;
  imu_msg->orientation_covariance[5] = 0.0;
  imu_msg->orientation_covariance[6] = 0.0;
  imu_msg->orientation_covariance[7] = 0.0;
  imu_msg->orientation_covariance[8] = orientation_variance_;
  }
  else
  {
    imu_msg->orientation_covariance = orient_cov_matrix_;
  }
  
  // Account for biases.
  if (filter_.getDoBiasEstimation())
  {
    imu_msg->angular_velocity.x -= filter_.getAngularVelocityBiasX();
    imu_msg->angular_velocity.y -= filter_.getAngularVelocityBiasY();
    imu_msg->angular_velocity.z -= filter_.getAngularVelocityBiasZ();
  }

  imu_publisher_.publish(imu_msg);

  if (publish_debug_topics_)
  {
      // Create and publish roll, pitch, yaw angles
      geometry_msgs::Vector3Stamped rpy;
      rpy.header = imu_msg_raw->header;

      tf::Matrix3x3 M;
      M.setRotation(q);
      M.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
      rpy_publisher_.publish(rpy);

      // Publish whether we are in the steady state, when doing bias estimation
      if (filter_.getDoBiasEstimation())
      {
        std_msgs::Bool state_msg;
        state_msg.data = filter_.getSteadyState();
        state_publisher_.publish(state_msg);
      }
  }

  if (publish_tf_)
  {
      // Create and publish the ROS tf.
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
      transform.setRotation(q);

      if (reverse_tf_)
      {
          tf_broadcaster_.sendTransform(
              tf::StampedTransform(transform.inverse(),
                                   imu_msg_raw->header.stamp,
                                   imu_msg_raw->header.frame_id,
                                   fixed_frame_));
      }
      else
      {
          tf_broadcaster_.sendTransform(
              tf::StampedTransform(transform,
                                   imu_msg_raw->header.stamp,
                                   fixed_frame_,
                                   imu_msg_raw->header.frame_id));
      }
  }
}

}  // namespace imu_tools
