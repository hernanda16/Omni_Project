#include "robot/robot_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);

    // Get parameters from the parameter server
    MAX_LIN_VEL = nh.param<float>("max_lin_vel", 0.5f);
    MAX_ANG_VEL = nh.param<float>("max_ang_vel", 0.2f);
    MAX_LIN_ACC = nh.param<float>("max_lin_acc", 0.2f);
    MAX_ANG_ACC = nh.param<float>("max_ang_acc", 0.1f);
    Kp = nh.param<float>("kp", 1.0f);
    Ki = nh.param<float>("ki", 0.0f);
    Kd = nh.param<float>("kd", 0.0f);
    tf_lidar2base_x = nh.param<float>("tf_lidar2base_x", 0.0f);
    tf_lidar2base_y = nh.param<float>("tf_lidar2base_y", 0.0f);
    tf_lidar2base_theta = nh.param<float>("tf_lidar2base_theta", 0.0f);

    initial_pose.x = nh.param<float>("initial_pose_x", 0.0f);
    initial_pose.y = nh.param<float>("initial_pose_y", 0.0f);
    initial_pose.theta = nh.param<float>("initial_pose_theta", 0.0f);

    printf("======================================\n");
    printf("        ROBOT NODE PARAMETERS         \n");
    printf("======================================\n");
    printf("Initial Pose\t: (%.2f, %.2f, %.2f)\n", initial_pose.x, initial_pose.y, initial_pose.theta);
    printf("MAX_LIN_VEL\t: %.2f\n", MAX_LIN_VEL);
    printf("MAX_ANG_VEL\t: %.2f\n", MAX_ANG_VEL);
    printf("MAX_LIN_ACC\t: %.2f\n", MAX_LIN_ACC);
    printf("MAX_ANG_ACC\t: %.2f\n", MAX_ANG_ACC);
    printf("Kp\t\t: %.2f\n", Kp);
    printf("Ki\t\t: %.2f\n", Ki);
    printf("Kd\t\t: %.2f\n", Kd);
    printf("Lidar to Base\t: (%.2f, %.2f, %.2f)\n", tf_lidar2base_x, tf_lidar2base_y, tf_lidar2base_theta);
    printf("======================================\n");

    set_initial_pose(initial_pose.x, initial_pose.y, initial_pose.theta);

    sub_joy = nh.subscribe<sensor_msgs::Joy>("/device/joy", 1, joy_callback);
    sub_imu = nh.subscribe<sensor_msgs::Imu>("/device/imu/data", 1, imu_callback);
    sub_lidar = nh.subscribe<sensor_msgs::LaserScan>("/device/lidar/scan", 1, lidar_callback);
    sub_encoder = nh.subscribe<std_msgs::Int32MultiArray>("/device/motor/raw_enc", 1, encoder_callback);
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
    pub_robot_pose = nh.advertise<geometry_msgs::Pose2D>("/robot/pose", 1);
    timer_main = nh.createTimer(ros::Duration(0.02), timer_callback);

    spinner.spin();
    return 0;
}

void timer_callback(const ros::TimerEvent&)
{
    state_control();
    publish_all();
}

void keyboard_handler()
{
    static uint16_t state = 0;
    if (kbhit()) {
        char c = getchar();
        switch (c) {
        case 'w':
            state = 'w';
            break;
        case 's':
            state = 's';
            break;
        case 'a':
            state = 'a';
            break;
        case 'd':
            state = 'd';
            break;
        case 'q':
            state = 'q';
            break;
        case 'e':
            state = 'e';
            break;
        case ' ':
            state = ' ';
            break;
        default:
            break;
        }
    }

    switch (state) {
    case 'w':
        velocity_control(0.0, MAX_LIN_VEL, 0.0);
        break;
    case 's':
        velocity_control(0.0, -MAX_LIN_VEL, 0.0);
        break;
    case 'a':
        velocity_control(-MAX_LIN_VEL, 0.0, 0.0);
        break;
    case 'd':
        velocity_control(MAX_LIN_VEL, 0.0, 0.0);
        break;
    case 'q':
        velocity_control(0.0, 0.0, MAX_ANG_VEL);
        break;
    case 'e':
        velocity_control(0.0, 0.0, -MAX_LIN_VEL);
        break;
    case ' ':
        velocity_control(0.0, 0.0, 0.0);
        break;
    default:
        break;
    }
}

void joystick_handler()
{
    velocity_control(axis_left.x * MAX_LIN_VEL, axis_left.y * MAX_LIN_VEL, axis_right.x * MAX_ANG_VEL);
}

void state_control()
{
    if (controlled_by == KEYBOARD)
        keyboard_handler();
    else if (controlled_by == JOYSTICK)
        joystick_handler();
}

void velocity_control(float vx, float vy, float vtheta)
{
    static pose_t prev_robot_vel = { 0.0, 0.0, 0.0 };
    static double prev_time = ros::Time::now().toSec();
    double dt = ros::Time::now().toSec() - prev_time;

    if (vx > prev_robot_vel.x + MAX_LIN_ACC * dt) {
        robot_vel.x = prev_robot_vel.x + MAX_LIN_ACC * dt;
    } else if (vx < prev_robot_vel.x - MAX_LIN_ACC * dt) {
        robot_vel.x = prev_robot_vel.x - MAX_LIN_ACC * dt;
    }

    if (vy > prev_robot_vel.y + MAX_LIN_ACC * dt) {
        robot_vel.y = prev_robot_vel.y + MAX_LIN_ACC * dt;
    } else if (vy < prev_robot_vel.y - MAX_LIN_ACC * dt) {
        robot_vel.y = prev_robot_vel.y - MAX_LIN_ACC * dt;
    }

    if (vtheta > prev_robot_vel.theta + MAX_ANG_ACC * dt) {
        robot_vel.theta = prev_robot_vel.theta + MAX_ANG_ACC * dt;
    } else if (vtheta < prev_robot_vel.theta - MAX_ANG_ACC * dt) {
        robot_vel.theta = prev_robot_vel.theta - MAX_ANG_ACC * dt;
    }

    prev_time = ros::Time::now().toSec();
    prev_robot_vel = robot_vel;
}

uint8_t position_control(float x, float y, float theta)
{
    static float integral_x = 0.0, integral_y = 0.0, integral_theta = 0.0;
    static float prev_error_x = 0.0, prev_error_y = 0.0, prev_error_theta = 0.0;
    static double prev_time = ros::Time::now().toSec();

    float error_x = x - robot_pose.x;
    float error_y = y - robot_pose.y;
    float error_theta = theta - robot_pose.theta;

    double current_time = ros::Time::now().toSec();
    double dt = current_time - prev_time;

    if (dt > 0.0) {
        integral_x += error_x * dt;
        integral_y += error_y * dt;
        integral_theta += error_theta * dt;

        float derivative_x = (error_x - prev_error_x) / dt;
        float derivative_y = (error_y - prev_error_y) / dt;
        float derivative_theta = (error_theta - prev_error_theta) / dt;

        float output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x;
        float output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y;
        float output_theta = Kp * error_theta + Ki * integral_theta + Kd * derivative_theta;

        velocity_control(output_x, output_y, output_theta);

        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_theta = error_theta;
        prev_time = current_time;

        if (fabs(error_x) < 0.005 && fabs(error_y) < 0.005 && fabs(error_theta) < 0.005) {
            return 1;
        }
    }
    return 0;
}

void publish_all()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = robot_vel.x;
    cmd_vel.linear.y = robot_vel.y;
    cmd_vel.angular.z = robot_vel.theta;
    pub_cmd_vel.publish(cmd_vel);

    geometry_msgs::Pose2D pose_msg;
    pose_msg.x = robot_pose.x;
    pose_msg.y = robot_pose.y;
    pose_msg.theta = robot_pose.theta;
    pub_robot_pose.publish(pose_msg);
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    axis_left.x = -msg->axes[0];
    axis_left.y = msg->axes[1];
    axis_right.x = msg->axes[2];
    axis_right.y = -msg->axes[3];

    buttons.x = msg->buttons[0];
    buttons.o = msg->buttons[1];
    buttons.square = msg->buttons[2];
    buttons.triangle = msg->buttons[3];

    if (buttons.x == 1)
        controlled_by = KEYBOARD;
    else
        controlled_by = JOYSTICK;

    // printf("left: (x: %.2f, y: %.2f), right: (x: %.2f, y: %.2f)\n", axis_left.x, axis_left.y, axis_right.x, axis_right.y);
    // printf("buttons: (x: %d, o: %d, sq: %d, tr: %d)\n", buttons.x, buttons.o, buttons.square, buttons.triangle);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    float buffer_theta = tf::getYaw(msg->orientation) * 180.0 / M_PI;
    static float prev_buffer_theta = buffer_theta;
    robot_pose.theta += buffer_theta - prev_buffer_theta;
    prev_buffer_theta = buffer_theta;
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    point2d_t temp;
    lidar_data.clear();

    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < msg->range_max) {
            float angle = msg->angle_min + i * msg->angle_increment + tf_lidar2base_theta;
            temp.x = msg->ranges[i] * cos(angle);
            temp.y = msg->ranges[i] * sin(angle);
            lidar_data.push_back(temp);
        }
    }

    for (int i = 0; i < lidar_data.size(); i++) {
        float x = lidar_data[i].x;
        float y = lidar_data[i].y;
        // Apply the transformation from lidar to base
        float x_base = x * cos(tf_lidar2base_theta) - y * sin(tf_lidar2base_theta) + tf_lidar2base_x;
        float y_base = x * sin(tf_lidar2base_theta) + y * cos(tf_lidar2base_theta) + tf_lidar2base_y;
        // Apply the transformation from base to world
        lidar_data[i].x = x_base * cos(robot_pose.theta) - y_base * sin(robot_pose.theta) + robot_pose.x;
        lidar_data[i].y = x_base * sin(robot_pose.theta) + y_base * cos(robot_pose.theta) + robot_pose.y;
    }
}

void encoder_callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    static const float angle[4] = { 45, 135, 225, 315 };
    int32_t enc_buffer[4] = { msg->data[0], msg->data[1], msg->data[2], msg->data[3] };
    static int32_t enc_prev_buffer[4] = { enc_buffer[0], enc_buffer[1], enc_buffer[2], enc_buffer[3] };
    static int32_t enc_diff[4] = { 0, 0, 0, 0 };
    static double prev_time = ros::Time::now().toSec();

    double current_time = ros::Time::now().toSec();
    double dt = current_time - prev_time;

    for (int i = 0; i < 4; i++) {
        enc_diff[i] = enc_buffer[i] - enc_prev_buffer[i];
        enc_prev_buffer[i] = enc_buffer[i];
    }

    float dx = 0.0, dy = 0.0, dtheta = 0.0;
    for (int i = 0; i < 4; i++) {
        dx += enc_diff[i] * cos(angle[i] * M_PI / 180.0);
        dy += enc_diff[i] * sin(angle[i] * M_PI / 180.0);
    }

    robot_pose.x += (dx * cos(robot_pose.theta) - dy * sin(robot_pose.theta)) * dt;
    robot_pose.y += (dx * sin(robot_pose.theta) + dy * cos(robot_pose.theta)) * dt;

    prev_time = current_time;
}