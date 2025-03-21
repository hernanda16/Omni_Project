#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "comm_node");
    ros::NodeHandle nh;

    ROS_INFO("comm_node has started.");

    ros::spin();

    return 0;
}