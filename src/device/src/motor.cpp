#include "device/motor.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);

    WHEEL_RADIUS = nh.param<float>("R", 0.05);
    DISTANCE_W2MID = nh.param<float>("D", 0.175);
    Kp = nh.param<float>("motor_kp", 1.0f);
    Ki = nh.param<float>("motor_ki", 0.0f);
    Kd = nh.param<float>("motor_kd", 0.0f);
    minOutput = nh.param<float>("pid_min_out", -150.0f);
    maxOutput = nh.param<float>("pid_max_out", 150.0f);

    pub_cmd_motor = nh.advertise<std_msgs::Int16MultiArray>("/device/cmd_motor", 1);
    sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("/robot/cmd_vel", 1, callback_cmd_vel);
    sub_enc_vel = nh.subscribe<std_msgs::Int16MultiArray>("/device/raw_enc", 1, callback_enc_vel);

    // myPID_wheel_0 = new PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
    // myPID_wheel_1 = new PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
    // myPID_wheel_2 = new PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
    // myPID_wheel_3 = new PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);

    my_PID = new PIDControl[4] {
        PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT),
        PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT),
        PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT),
        PIDControl(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT)
    };

    spinner.spin();

    delete myPID_wheel_0;
    delete myPID_wheel_1;
    delete myPID_wheel_2;
    delete myPID_wheel_3;

    return 0;
}

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    static int16_t prev_vel_act[4] = { 0 };
    for (uint8_t i = 0; i < msg->data.size(); i++) {
        vel_act[i] = msg->data[i];
    }
}
void callback_enc_vel(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    for (uint8_t i = 0; i < msg->data.size(); i++) {
        vel_fb[i] = msg->data[i] * CPP2RADPS;
        my_PID[i]->PIDInputSet(vel_fb[i]);
        my_PID[i]->PIDCompute();
    }

    // myPID_wheel_0->PIDInputSet(vel_fb[0]);
    // myPID_wheel_0->PIDCompute();
    // short motor_command_0 = (short)round(myPID_wheel_0->PIDOutputGet());

    // myPID_wheel_1->PIDInputSet(vel_fb[1]);
    // myPID_wheel_1->PIDCompute();
    // short motor_command_1 = (short)round(myPID_wheel_1->PIDOutputGet());

    // myPID_wheel_2->PIDInputSet(vel_fb[2]);
    // myPID_wheel_2->PIDCompute();
    // short motor_command_2 = (short)round(myPID_wheel_2->PIDOutputGet());

    // myPID_wheel_3->PIDInputSet(vel_fb[3]);
    // myPID_wheel_3->PIDCompute();
    // short motor_command_3 = (short)round(myPID_wheel_3->PIDOutputGet());

    std_msgs::Int16MultiArray cmd_motor_msg;

    cmd_motor_msg.push_back(motor_command_0);
    cmd_motor_msg.push_back(motor_command_1);
    cmd_motor_msg.push_back(motor_command_2);
    cmd_motor_msg.push_back(motor_command_3);

    cmd_motor_pub.publish(cmd_motor_msg);
}

// NexusMotorController::NexusMotorController()
//     : wheelSpeed({ 0, 0, 0, 0 })
//     , wheelDeg({ 315, 45, 135, 225 })
//     , wheelSin({ sin(wheelDeg[0] * DEG2RAD), sin(wheelDeg[1] * DEG2RAD), sin(wheelDeg[2] * DEG2RAD), sin(wheelDeg[3] * DEG2RAD) })
//     , wheelCos({ cos(wheelDeg[0] * DEG2RAD), cos(wheelDeg[1] * DEG2RAD), cos(wheelDeg[2] * DEG2RAD), cos(wheelDeg[3] * DEG2RAD) })
//     , vel_fb({ 0, 0, 0, 0 })
//     , prev_vel_fb({ 0, 0, 0, 0 })
//     , lin_x(0)
//     , lin_y(0)
//     , ang_z(0)
//     , dt(0)
// {

//     for (int i = 0; i < 4; i++) {
//         myPID_wheel.emplace_back(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
//     }

//     cmd_motor_pub = nh.advertise<std_msgs::Int16MultiArray>("/device/cmd_motor", QUEUE_SIZE);
//     cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/robot/cmd_vel", QUEUE_SIZE, &NexusMotorController::velocityCallback, this);
//     enc_sub = nh.subscribe<std_msgs::Int16MultiArray>("/device/raw_enc", QUEUE_SIZE, &NexusMotorController::encoderCallback, this);

//     last_time = ros::Time::now();
// }

// void NexusMotorController::velocityCallback(const geometry_msgs::Twist::ConstPtr& twist_aux)
// {
//     lin_x = -twist_aux->linear.x;
//     lin_y = -twist_aux->linear.y;
//     ang_z = -twist_aux->angular.z;

//     // [ INFO] [1742819912.256170617]: lin_x: 0.000000, lin_y: 0.000000, ang_z: 0.000000, WHEEL_RADIUS: 0.050000, DISTANCE_W2MID: 0.175000, wheelSin: [0.707107, 0.707107, -0.707107, -0.707107], wheelCos: [0.707107, -0.707107, -0.707107, 0.707107]

//     wheelSpeed[0] = 1 / WHEEL_RADIUS * (lin_x * wheelCos[0] + lin_y * wheelSin[0] + DISTANCE_W2MID * ang_z);
//     wheelSpeed[1] = 1 / WHEEL_RADIUS * (lin_x * wheelCos[1] + lin_y * wheelSin[1] + DISTANCE_W2MID * ang_z);
//     wheelSpeed[2] = 1 / WHEEL_RADIUS * (lin_x * wheelCos[2] + lin_y * wheelSin[2] + DISTANCE_W2MID * ang_z);
//     wheelSpeed[3] = 1 / WHEEL_RADIUS * (lin_x * wheelCos[3] + lin_y * wheelSin[3] + DISTANCE_W2MID * ang_z);

//     ROS_INFO("Wheel speeds: [%f, %f, %f, %f]", wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
// }

// void NexusMotorController::encoderCallback(const std_msgs::Int16MultiArray::ConstPtr& enc_aux)
// {
//     std_msgs::Int16MultiArray cmd_motor;

//     ros::Time current_time = ros::Time::now();
//     dt = (current_time - last_time).toSec();
//     last_time = current_time;

//     for (int i = 0; i < 4; i++) {
//         vel_fb[i] = enc_aux->data[i] * CPP2RADPS;
//     }

// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "nmc");
//     NexusMotorController nmc;

//     ros::MultiThreadedSpinner spinner(0);
//     spinner.spin();
//     // ros::spin();
//     return 0;
// }