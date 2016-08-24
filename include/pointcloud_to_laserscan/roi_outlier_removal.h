/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef ROI_OUTLIER_REMOVAL_NODELET
#define ROI_OUTLIER_REMOVAL_NODELET

#include "ros/ros.h"
#include "boost/thread/mutex.hpp"

#include "nodelet/nodelet.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/PointCloud2.h"

// includes for pcl filtering
#include <pcl_ros/point_cloud.h>

namespace pointcloud_to_laserscan
{
  typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> MessageFilter;
/**
* The ROI outlier removal removes all points outside the specified region of interest (ROI) and publishes the remaining points into a new pointcloud.
* The roi is defined as a z value range, angle range in the xy plane, and a max and min range in the xyy plane, for a chosen frame.
* The filter converts the roi borders to to the original pointcloud frame in order to not have to waist time on transforming points 
* that are not to be used later on.
* The points within the roi are transformed to and published in the same frame as the borders were given.
* The filter is relevant to the pointcloud to laserscan transformation since it creates a possibility to apply additional statistical filtering
* to a reduced pointcloud before creating the scan. The loclation of this filter within the pointcloud to laserscan package is however questionable.
*/
  class RoiOutlierRemovalNodelet : public nodelet::Nodelet
  {

  public:
    RoiOutlierRemovalNodelet();
    void configure_filter();

  private:
    virtual void onInit();

    void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    void reduce_point_cloud_to_roi(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,  
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_pcl_cloud,
                                  const tf2::Transform &T);

    ros::NodeHandle nh_, private_nh_;
    ros::Publisher pub_;

    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

	ros::Subscriber sub_;

    // ROS Parameters
    unsigned int input_queue_size_;
    std::string roi_def_frame_;
    double tolerance_;
    double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, range_min_, range_max_;    
  };

}  // roi_outlier_removal

#endif  // ROI_OUTLIER_REMOVAL_NODELET
