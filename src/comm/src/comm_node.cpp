#include <arpa/inet.h>
#include <cstring>
#include <geometry_msgs/Pose2D.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <unistd.h>

uint8_t robot_num = 1;

#define MULTICAST_GROUP "239.255.0.1"
#define MULTICAST_PORT 12345

void send_data(int sockfd, struct sockaddr_in& multicast_addr, const geometry_msgs::Pose2D& pose)
{
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));

    memcpy(buffer, &robot_num, sizeof(robot_num));
    memcpy(buffer + sizeof(robot_num), &pose.x, sizeof(pose.x));
    memcpy(buffer + sizeof(robot_num) + sizeof(pose.x), &pose.y, sizeof(pose.y));
    memcpy(buffer + sizeof(robot_num) + sizeof(pose.x) + sizeof(pose.y), &pose.theta, sizeof(pose.theta));

    sendto(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&multicast_addr, sizeof(multicast_addr));
}

void callback_sub_pose(const geometry_msgs::Pose2D::ConstPtr& msg, int sockfd, struct sockaddr_in& multicast_addr)
{
    send_data(sockfd, multicast_addr, *msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "comm_node");
    ros::NodeHandle nh;

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        ROS_ERROR("Failed to create socket");
        return -1;
    }

    robot_num = nh.param<int>("robot_num", 1);

    printf("======================================\n");
    printf("    COMMUNICATION NODE PARAMETERS     \n");
    printf("======================================\n");
    printf("Robot Number\t: %d\n", robot_num);
    printf("======================================\n");

    struct sockaddr_in multicast_addr;
    memset(&multicast_addr, 0, sizeof(multicast_addr));
    multicast_addr.sin_family = AF_INET;
    multicast_addr.sin_addr.s_addr = inet_addr(MULTICAST_GROUP);
    multicast_addr.sin_port = htons(MULTICAST_PORT);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose2D>("/robot/pose", 10, boost::bind(callback_sub_pose, _1, sockfd, multicast_addr));

    ros::spin();

    close(sockfd);
    return 0;
}