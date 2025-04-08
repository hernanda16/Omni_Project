#ifndef ROBOT_NODE_HPP
#define ROBOT_NODE_HPP

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <sys/ioctl.h>
#include <termios.h>
#include <tf/tf.h>

#define KEYBOARD 0
#define JOYSTICK 1

#define MANUAL 0
#define AUTO 1
#define DEBUG 2

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)

typedef struct {
    float x;
    float y;
} axis_t;

typedef struct {
    int8_t x;
    int8_t o;
    int8_t square;
    int8_t triangle;
} button_t;

typedef struct {
    float x;
    float y;
    float theta;
} pose_t;

typedef struct
{
    float x;
    float y;
} point2d_t;

// Global variables
float MAX_LIN_VEL = 0.5f; // in m/s
float MAX_ANG_VEL = 0.25f; // in rad/s
float MAX_LIN_ACC = 0.01f; // in m/s^2
float MAX_ANG_ACC = 1.0f; // in rad/s^2
float Kp = 0.1f;
float Ki = 0.0f;
float Kd = 0.0f;
float tf_lidar2base_x = 0.0f;
float tf_lidar2base_y = 0.0f;
float tf_lidar2base_theta = 0.0f;

pose_t initial_pose = { 0.0, 0.0, 0.0 };

pose_t robot_pose = { 0.0, 0.0, 0.0 }; // x, y, theta
pose_t robot_vel = { 0.0, 0.0, 0.0 }; // vx, vy, omega
uint16_t robot_state = 0;
uint8_t controlled_by = 0; // 0: keyboard, 1: joystick

uint8_t use_slam = 0;
uint8_t use_sim = 1;

std::vector<point2d_t> lidar_data;

axis_t axis_left;
axis_t axis_right;
button_t buttons;

// ROS objects
ros::Timer timer_main;
ros::Subscriber sub_joy;
ros::Subscriber sub_imu;
ros::Subscriber sub_lidar;
ros::Subscriber sub_encoder;
ros::Subscriber sub_amcl_pose;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_robot_pose;
ros::Publisher pub_robot_odom;
ros::Publisher pub_marker;

tf::TransformBroadcaster* tf_broadcaster;
tf::TransformListener* tf_listener;

geometry_msgs::PoseWithCovarianceStamped amcl_pose;

// Function prototypes
void timer_callback(const ros::TimerEvent&);
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void encoder_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

void dummy_odom();
void update_robot_pose();
float compute_amcl_trust();
void keyboard_handler();
void joystick_handler();
void state_control();
void velocity_control(float vx, float vy, float vtheta);
uint8_t position_control(float x, float y, float theta);
void publish_all();

void set_initial_pose(float x, float y, float theta)
{
    robot_pose.x = x;
    robot_pose.y = y;
    robot_pose.theta = theta;
}

int8_t kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized) {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

#endif