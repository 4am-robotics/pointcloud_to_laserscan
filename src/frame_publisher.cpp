
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Fraunhofer IPA.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Sofie Nilsson
 * Author: Thomas Lindemeier
 */

#include <pointcloud_to_laserscan/frame_publisher.h>

FramePublisher::FramePublisher() :
  tf_listener_(tf_buffer_)
{}

FramePublisher::~FramePublisher()
{}

bool FramePublisher::initialize()
{
  priv_nh_ = ros::NodeHandle("~");

  priv_nh_.param<double>("update_rate", update_rate_, 0.01);  // 100Hz

  priv_nh_.param<std::string>("from_frame", from_frame_, "base_link");
  priv_nh_.param<std::string>("to_frame", to_frame_, "torso_center_link");
  priv_nh_.param<std::string>("frame_name", frame_name_, "torso_rotated_base_link");

  priv_nh_.param<bool>("use_to_frame_translation", use_to_frame_translation_, true);

  priv_nh_.param<bool>("rot_z", rot_z_, false);
  priv_nh_.param<bool>("rot_x", rot_x_, false);
  priv_nh_.param<bool>("rot_y", rot_y_, false);

  frame_broadcast_timer_ = nh_.createTimer(ros::Duration(update_rate_), &FramePublisher::frameBroadcastCallback, this);

  ros::Duration(1.0).sleep();  // give tf_listener some time

  return true;
}

/// Broadcast a new frame based on a given transformation from_frame -> to_frame - resetting either translation and/or individual rotation axes to zero
void FramePublisher::frameBroadcastCallback(const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped transform_msg;
  try
  {
    transform_msg = tf_buffer_.lookupTransform(from_frame_, to_frame_, ros::Time(0), ros::Duration(0.1));
    ROS_DEBUG_STREAM("FramePublisher::frameBroadcastCallback: transform_msg:\n" << transform_msg);
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(5.0, "FramePublisher::frameBroadcastCallback ConnectivityException: \n%s", ex.what());
    return;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(5.0, "FramePublisher::frameBroadcastCallback ExtrapolationException: \n%s", ex.what());
    return;
  }
  catch (tf2::InvalidArgumentException& ex)
  {
    ROS_ERROR_THROTTLE(5.0, "FramePublisher::frameBroadcastCallback InvalidArgumentException: \n%s", ex.what());
    return;
  }
  catch (tf2::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(5.0, "FramePublisher::frameBroadcastCallback LookupException: \n%s", ex.what());
    return;
  }
  catch (tf2::TimeoutException& ex)
  {
    ROS_ERROR_THROTTLE(5.0, "FramePublisher::frameBroadcastCallback TimeoutException: \n%s", ex.what());
    return;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_THROTTLE(5.0, "FramePublisher::frameBroadcastCallback TransformException: \n%s", ex.what());
    return;
  }

  tf2::Stamped<tf2::Transform> transform_tf;
  tf2::fromMsg(transform_msg, transform_tf);
  double rot_frame_roll, rot_frame_pitch, rot_frame_yaw;
  transform_tf.getBasis().getRPY(rot_frame_roll, rot_frame_pitch, rot_frame_yaw);

  // Use rotations according to settings
  double published_frame_roll = 0;
  double published_frame_pitch = 0;
  double published_frame_yaw = 0;
  if (rot_x_){ published_frame_roll = rot_frame_roll; }
  if (rot_y_){ published_frame_pitch = rot_frame_pitch; }
  if (rot_z_){ published_frame_yaw = rot_frame_yaw; }

  tf2::Stamped<tf2::Transform> published_tf(transform_tf);  // keep header and translation
  if (!use_to_frame_translation_){ published_tf.setOrigin(tf2::Vector3(0,0,0)); }
  published_tf.getBasis().setRPY(published_frame_roll, published_frame_pitch, published_frame_yaw);

  // Broadcast new frame
  geometry_msgs::TransformStamped published_msg = tf2::toMsg(published_tf);
  published_msg.child_frame_id = frame_name_;
  if (published_msg.header.stamp != published_msg_.header.stamp)  //only publish on updated transform to avoid TF_REPEATED_DATA (noetic)
  {
    tf_broadcaster_.sendTransform(published_msg);
    ROS_DEBUG_STREAM("FramePublisher::frameBroadcastCallback: published_msg:\n" << published_msg);
  }
  published_msg_ = published_msg;
}
