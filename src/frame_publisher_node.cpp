#include <ros/ros.h>
#include <pointcloud_to_laserscan/frame_publisher.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msh_frame_publisher_node");
    FramePublisher* ad = new FramePublisher();

    if (!ad->initialize())
    {
        ROS_ERROR("Failed to initialize FramePublisher");
        return -1;
    }

    ros::spin();
    return 0;
}
