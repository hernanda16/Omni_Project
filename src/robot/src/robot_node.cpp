#include "robot/robot_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);

    // Get parameters from the parameter server
    nh.param("/robot/max_lin_vel", MAX_LIN_VEL, 0.5f);
    nh.param("/robot/max_ang_vel", MAX_ANG_VEL, 0.25f);
    nh.param("/robot/max_lin_acc", MAX_LIN_ACC, 0.01f);
    nh.param("/robot/max_ang_acc", MAX_ANG_ACC, 0.1f);
    nh.param("/robot/kp", Kp, 1.0f);
    nh.param("/robot/ki", Ki, 0.0f);
    nh.param("/robot/kd", Kd, 0.0f);

    printf("======================================\n");
    printf("        ROBOT NODE PARAMETERS         \n");
    printf("======================================\n");
    printf("\tMAX_LIN_VEL\t: %.2f\n", MAX_LIN_VEL);
    printf("\tMAX_ANG_VEL\t: %.2f\n", MAX_ANG_VEL);
    printf("\tMAX_LIN_ACC\t: %.2f\n", MAX_LIN_ACC);
    printf("\tMAX_ANG_ACC\t: %.2f\n", MAX_ANG_ACC);
    printf("\tKp\t\t: %.2f\n", Kp);
    printf("\tKi\t\t: %.2f\n", Ki);
    printf("\tKd\t\t: %.2f\n", Kd);
    printf("======================================\n");

    sub_joy = nh.subscribe<sensor_msgs::Joy>("/device/joy", 1, joy_callback);
    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
    timer_main = nh.createTimer(ros::Duration(0.1), timer_callback);

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
    if (ros::Time::now().toSec() - joystick_timer > 1) {
        controlled_by = KEYBOARD;
    } else {
        controlled_by = JOYSTICK;
    }

    if (controlled_by == KEYBOARD) {
        keyboard_handler();
    } else if (controlled_by == JOYSTICK) {
        joystick_handler();
    }
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

    printf("vx: %.2f, vy: %.2f, vtheta: %.2f\n", robot_vel.x, robot_vel.y, robot_vel.theta);

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
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    axis_left.x = -msg->axes[0];
    axis_left.y = msg->axes[1];
    axis_right.x = -msg->axes[2];
    axis_right.y = msg->axes[3];

    buttons.x = msg->buttons[0];
    buttons.o = msg->buttons[1];
    buttons.square = msg->buttons[2];
    buttons.triangle = msg->buttons[3];

    // printf("left: (x: %.2f, y: %.2f), right: (x: %.2f, y: %.2f)\n", axis_left.x, axis_left.y, axis_right.x, axis_right.y);
    // printf("buttons: (x: %d, o: %d, sq: %d, tr: %d)\n", buttons.x, buttons.o, buttons.square, buttons.triangle);

    joystick_timer = ros::Time::now().toSec();
}