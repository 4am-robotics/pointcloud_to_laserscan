#include <pointcloud_to_laserscan/frame_publisher.h>

bool FramePublisher::initialize()
{
	priv_nh_ = ros::NodeHandle("~");

	priv_nh_.param<double>("update_rate", update_rate_, 0.01); // 100Hz

	priv_nh_.param<std::string>("base_frame", base_frame_, "base_link");
	priv_nh_.param<std::string>("rotation_frame", rotation_frame_, "torso_center_link");
	priv_nh_.param<std::string>("target_frame", target_frame_, "torso_rotated_base_link");
    
	priv_nh_.param<bool>("rot_z", rot_z_, false);
	priv_nh_.param<bool>("rot_x", rot_x_, false);
	priv_nh_.param<bool>("rot_y", rot_y_, false);
        
    frame_broadcast_timer_ = nh_.createTimer(ros::Duration(update_rate_), &FramePublisher::frameBroadcastCallback, this);

    ros::Duration(1.0).sleep(); //give tf_listener some time

    return true;
}

/// Broadcast a frame aligned with the base frame but rotated around specified axes as rotation_frame 
void FramePublisher::frameBroadcastCallback(const ros::TimerEvent& event)
{   
    tf::StampedTransform frame_transform;
    try
    {
        tf_listener_.waitForTransform(base_frame_, rotation_frame_, event.current_real, ros::Duration(0.1));
        tf_listener_.lookupTransform(base_frame_, rotation_frame_, ros::Time(0), frame_transform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("FramePublisher::getTransform: \n%s", ex.what());
        return;
    }

    double rot_frame_roll, rot_frame_pitch, rot_frame_yaw;
    frame_transform.getBasis().getRPY(rot_frame_roll, rot_frame_pitch, rot_frame_yaw);
	tf::Transform target_transform(frame_transform.getRotation());

	// Use rotations according to settings
	double target_frame_roll = 0;
	double target_frame_pitch = 0;
	double target_frame_yaw = 0; 
	if (rot_z_){ target_frame_yaw = rot_frame_yaw;}
	if (rot_x_){ target_frame_roll = rot_frame_roll;}
	if (rot_y_){ target_frame_pitch = rot_frame_pitch;}

	target_transform.getBasis().setRPY(target_frame_roll, target_frame_pitch, target_frame_yaw);

	// Broadcast new frame
    tf_broadcaster_.sendTransform(tf::StampedTransform(target_transform, frame_transform.stamp_, base_frame_, target_frame_));
}
