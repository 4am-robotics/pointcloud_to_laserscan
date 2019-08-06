
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
 */


#include <pointcloud_to_laserscan/roi_outlier_removal.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// includes for pcl filtering (kept for possibility to make as pcl filter later on)
#include <pcl/filters/filter.h>

#include <iostream>

using namespace pointcloud_to_laserscan;

RoiOutlierRemovalNodelet::RoiOutlierRemovalNodelet()
{
  NODELET_INFO_STREAM("constructor");
}

void RoiOutlierRemovalNodelet::onInit()
{
  NODELET_INFO_STREAM("on init");
  private_nh_ = getPrivateNodeHandle();

  int concurrency_level;
  private_nh_.param<int>("concurrency_level", concurrency_level, 1);

  configure_roi_settings();

  //Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = concurrency_level;
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!roi_def_frame_.empty())
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
  }
  sub_ = nh_.subscribe("cloud_in", input_queue_size_, &RoiOutlierRemovalNodelet::cloudCb, this );

  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out", 10);
}

void RoiOutlierRemovalNodelet::configure_roi_settings()
{
  NODELET_DEBUG_STREAM("configuring filter");
    // Get filter related parameters
  private_nh_.param<std::string>("roi_def_frame", roi_def_frame_, "");
  private_nh_.param<double>("transform_tolerance", tolerance_, 0.1);

  private_nh_.param<double>("min_height", min_height_, 0.0);
  private_nh_.param<double>("max_height", max_height_, 1.0);
  private_nh_.param<double>("angle_min", angle_min_, -M_PI / 2.0);
  private_nh_.param<double>("angle_max", angle_max_, M_PI / 2.0);
  private_nh_.param<double>("range_min", range_min_, 0.45);
  private_nh_.param<double>("range_max", range_max_, 4.0);
}

void RoiOutlierRemovalNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // remove leading / on frame id in case present, which is not supported by tf2
  // does not do anything if the problem dies not occur -> leave for compatibility
  std::string cloud_frame_id = cloud_msg->header.frame_id;
  if(cloud_frame_id.find_first_of("/") == 0)
  {
    cloud_frame_id.erase(0,1);
  }

  // Get frame tranformation
  tf2::Transform T;

  if ((!roi_def_frame_.empty()) && !(roi_def_frame_ == cloud_frame_id))
  {
    try
    {
      geometry_msgs::TransformStamped T_geom = tf2_->lookupTransform(cloud_frame_id, roi_def_frame_, cloud_msg->header.stamp, ros::Duration(0.1));
      // Convert geometry msgs transform to tf2 transform.
      tf2::fromMsg(T_geom.transform, T);
    }
    catch (tf2::TransformException ex)
    {
      NODELET_WARN_STREAM("Transform failure: " << ex.what());
      return;
    }
  }
  else
  {
    // target and source frame are the same
    T.setIdentity();
  }
  NODELET_DEBUG_STREAM("Got transform between " << cloud_msg->header.frame_id << " and " << roi_def_frame_);

  // create pcl cloud objects
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ> ());

  // convert pointcloud to pcl pointcloud
  pcl::fromROSMsg (*cloud_msg, *pcl_cloud);

  NODELET_DEBUG_STREAM("pcl cloud assigned");
  // Reduce pointcloud
  reduce_point_cloud_to_roi(pcl_cloud, reduced_pcl_cloud, T);
  NODELET_DEBUG_STREAM("cloud reduced");

  //add at least one point so the pcl statistical outlier filter
  //gets an not empty point cloud. On empty points clouds the console
  //is flooded with error messages
  if(reduced_pcl_cloud->points.empty())
  {
    reduced_pcl_cloud->points.push_back(pcl::PointXYZ(0.0,0.0,0.0));
  }

  // assign output message
  sensor_msgs::PointCloud2 output;

  pcl::toROSMsg(*reduced_pcl_cloud, output);

  output.header = cloud_msg->header;

  // Publish output
  pub_.publish(output);

  NODELET_DEBUG_STREAM("Transform and publish finished");
}

/**
 * Function to find the points within the defined region of interest and add those to the reduced_cloud.
 * The borders given in specified fame is transformed to the pc frame to avoid unnessecaric point transformations.
 * The reduced cloud in in the same frame as the original cloud, no transformation included!
 */
void RoiOutlierRemovalNodelet::reduce_point_cloud_to_roi(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                                                         pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_pcl_cloud,
                                                         const tf2::Transform &T)
{
  // Transform borders and target plane to original coordinates (saved resources to not have to transform the whole point cloud)
  // A plane is described by all points fulfilling p= A + l1*e1 + l2*e2.
  // Transformation to other coordinate frame with transformation T gives: p'= T(A) + l1*T(e1) + l2*T(e2)
  // It is here assumed that both the border planes and target plane are paralell to each other -> same e1 and e2
  // Planes for max and min height: A = [0; 0; max_height]
  tf2::Vector3 A_max_t_frame(0, 0, max_height_);
  tf2::Vector3 A_max_o_frame = T(A_max_t_frame);
  NODELET_DEBUG_STREAM("A max: " << A_max_o_frame.getX() <<", "<< A_max_o_frame.getY() <<", "<< A_max_o_frame.getZ());

  tf2::Vector3 A_min_t_frame(0, 0, min_height_);
  tf2::Vector3 A_min_o_frame = T(A_min_t_frame); // want to get the orientation vector in the new coordinates
  NODELET_DEBUG_STREAM("A min: " << A_min_o_frame.getX() <<", "<< A_min_o_frame.getY() <<", "<< A_min_o_frame.getZ());

  // Transform plane basis vectors -> use only rotation
  tf2::Vector3 ex_t_frame(1, 0, 0);
  tf2::Vector3 ex_o_frame = tf2::quatRotate(T.getRotation(), ex_t_frame);
  NODELET_DEBUG_STREAM("ex min: " << ex_o_frame.getX() <<", "<< ex_o_frame.getY() <<", "<< ex_o_frame.getZ());

  tf2::Vector3 ey_t_frame(0, 1, 0);
  tf2::Vector3 ey_o_frame = tf2::quatRotate(T.getRotation(), ey_t_frame);
  NODELET_DEBUG_STREAM("ey min: " << ey_o_frame.getX() <<", "<< ey_o_frame.getY() <<", "<< ey_o_frame.getZ());

  // Plane for target scan:
  tf2::Vector3 A_target_t_frame(0, 0, 0);
  tf2::Vector3 A_target_o_frame = T(A_target_t_frame);

  NODELET_DEBUG_STREAM("Removing nans");
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pcl_cloud,*pcl_cloud, indices);
  indices.clear();

  int n_points = pcl_cloud->size();

  // Declare help variables for point selection
  float* iter_x;
  float* iter_y;
  float* iter_z;
  tf2::Vector3 P;
  double lambda_x, lambda_y;
  tf2::Vector3 P_max;
  tf2::Vector3 P_min;
  double border_distance_sqared;
  double range;
  double angle;
  NODELET_DEBUG_STREAM("filtering pc with " << n_points << " points ");
  // Iterate through pointcloud to select points
  for(int i = 0; i < n_points; i++)
  {
    iter_x = &pcl_cloud->points[i].x;
    iter_y = &pcl_cloud->points[i].y;
    iter_z = &pcl_cloud->points[i].z;

    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    //get reflection point in hight limiting planes in order to check that point lies between borders(above or below is not clearly def):
    P.setValue(*iter_x, *iter_y, *iter_z);

    /**
     * lambda x and y describes the location within the planes, which are the same for all paralell planes with
     * aligned origin -> calculate only once for the plane at height of target frame.
     * If assumption not hold, use one set of lambda per plane (use the A value of that specific plane like this:
     *
     * double lambda_x_max =  (P - A_max_o_frame).dot(ex_o_frame);
     *
     * For now we assume tahat a common set of lambdas hold:
     */
    lambda_x =  (P - A_target_o_frame).dot(ex_o_frame);
    lambda_y =  (P - A_target_o_frame).dot(ey_o_frame);
    P_max = A_max_o_frame + lambda_x*ex_o_frame + lambda_y*ey_o_frame;
    P_min = A_min_o_frame + lambda_x*ex_o_frame + lambda_y*ey_o_frame;

    border_distance_sqared = P_max.distance2(P_min);

    if ((P.distance2(P_max) > border_distance_sqared) || (P.distance2(P_min) > border_distance_sqared))
    {
      continue;
    }

    range = hypot(lambda_x, lambda_y);
    if ((range < range_min_) || (range > range_max_))
    {
      continue;
    }

    angle = atan2(lambda_y, lambda_x);
    if (angle < angle_min_ || angle > angle_max_)
    {
      continue;
    }

    // write to reduced pointcloud
    reduced_pcl_cloud->points.push_back(pcl_cloud->points[i]);
  }
}

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::RoiOutlierRemovalNodelet, nodelet::Nodelet);
