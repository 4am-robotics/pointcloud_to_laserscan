#ifndef FRAME_PUB_H
#define FRAME_PUB_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class FramePublisher
{
public:
    bool initialize();

private:
    void frameBroadcastCallback(const ros::TimerEvent& event);

private:
    ros::NodeHandle nh_, priv_nh_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::Timer frame_broadcast_timer_;
    double update_rate_;
	std::string base_frame_;
    std::string rotation_frame_;
    std::string target_frame_;
	bool rot_z_, rot_x_, rot_y_;
};

#endif  // FRAME_PUB_H
