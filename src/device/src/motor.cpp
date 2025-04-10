#include <ros/ros.h>
#include <serial/serial.h>
// #include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
// #include <std_msgs/String.h>

// Serial port object
serial::Serial ser;

float linx = 0.0;
float liny = 0.0;
float angz = 0.0;

// Callback untuk mengirim data ke Arduino
void sendVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Format data: 'e l k a' + 3 float
    uint8_t buffer[16];
    buffer[0] = 'e';
    buffer[1] = 'l';
    buffer[2] = 'k';
    buffer[3] = 'a';

    // Copy float data (linear.x, linear.y, angular.z) to buffer
    linx = msg->linear.x;
    liny = msg->linear.y;
    angz = -msg->angular.z;

    memcpy(&buffer[4], &linx, sizeof(float)); // linear.x
    memcpy(&buffer[8], &liny, sizeof(float)); // linear.y
    memcpy(&buffer[12], &angz, sizeof(float)); // angular.z

    // Kirim data ke Arduino
    ser.write(buffer, 16);
    ROS_INFO("Sent to Arduino: x=%.2f, y=%.2f, z=%.2f", msg->linear.x, msg->linear.y, msg->angular.z);
}
// void sendDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
//     if (msg->data.size() == 3) {
//         // Format data: 'e l k a' + 3 float
//         uint8_t buffer[16];
//         buffer[0] = 'e';
//         buffer[1] = 'l';
//         buffer[2] = 'k';
//         buffer[3] = 'a';

//         // Copy float data (x, y, z) ke buffer
//         memcpy(&buffer[4], &msg->data[0], sizeof(float)); // x
//         memcpy(&buffer[8], &msg->data[1], sizeof(float)); // y
//         memcpy(&buffer[12], &msg->data[2], sizeof(float)); // z

//         // Kirim data ke Arduino
//         ser.write(buffer, 16);
//         ROS_INFO("Sent to Arduino: x=%.2f, y=%.2f, z=%.2f", msg->data[0], msg->data[1], msg->data[2]);
//     } else {
//         ROS_WARN("Invalid data size. Expected 3 floats.");
//     }
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_serial_node");
    ros::NodeHandle nh;

    // Publisher untuk menerima data dari Arduino
    // ros::Publisher feedback_pub = nh.advertise<std_msgs::Float32MultiArray>("arduino_feedback", 10);
    ros::Publisher feedback_pub = nh.advertise<std_msgs::Int32MultiArray>("/device/motor/raw_enc", 1); // Change message type to Int32MultiArray

    // Subscriber untuk mengirim data ke Arduino
    // ros::Subscriber setpoint_sub = nh.subscribe("/robot/cmd_vel", 1, sendDataCallback);
    ros::Subscriber setpoint_sub = nh.subscribe("/robot/cmd_vel", 1, sendVelocityCallback);

    // Serial port setup
    try {
        ser.setPort("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI05UAHH-if00-port0"); // Ganti dengan port serial Arduino
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open port.");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO("Serial port initialized.");
    } else {
        return -1;
    }

    ros::Rate loop_rate(20); // Sesuaikan dengan LOOP_FREQUENCY Arduino (20 Hz)

    uint8_t buffer[20]; // Buffer untuk membaca data
    size_t bufferIndex = 0; // Indeks buffer

    while (ros::ok()) {
        // Baca data dari Arduino byte per byte
        while (ser.available() > 0) {
            uint8_t incomingByte = ser.read()[0];

            // Tambahkan byte ke buffer
            buffer[bufferIndex] = incomingByte;
            bufferIndex++;

            // Jika buffer penuh, periksa apakah itu header yang valid
            if (bufferIndex == 4) {
                if (buffer[0] == 'e' && buffer[1] == 'l' && buffer[2] == 'k' && buffer[3] == 'a') {
                    // Header valid ditemukan, baca data float
                    if (ser.available() >= 16) { // Pastikan ada cukup data untuk 4 float
                        ser.read(buffer + 4, 16); // Baca sisa data

                        int32_t vel_fb[4];
                        memcpy(&vel_fb[0], &buffer[4], sizeof(int32_t)); // Motor 0
                        memcpy(&vel_fb[1], &buffer[8], sizeof(int32_t)); // Motor 1
                        memcpy(&vel_fb[2], &buffer[12], sizeof(int32_t)); // Motor 2
                        memcpy(&vel_fb[3], &buffer[16], sizeof(int32_t)); // Motor 3

                        // Publikasikan data feedback
                        std_msgs::Int32MultiArray feedback_msg; // Change message type to Int32MultiArray
                        feedback_msg.data.resize(4);
                        feedback_msg.data[0] = static_cast<int32_t>(vel_fb[0]); // Convert float to int32
                        feedback_msg.data[1] = static_cast<int32_t>(vel_fb[1]);
                        feedback_msg.data[2] = static_cast<int32_t>(vel_fb[2]);
                        feedback_msg.data[3] = static_cast<int32_t>(vel_fb[3]);
                        feedback_pub.publish(feedback_msg);

                        // ROS_INFO("Received from Arduino: vel_fb[0]=%.2f, vel_fb[1]=%.2f, vel_fb[2]=%.2f, vel_fb[3]=%.2f",
                        //          vel_fb[0], vel_fb[1], vel_fb[2], vel_fb[3]);
                    }
                    bufferIndex = 0; // Reset buffer setelah data berhasil dibaca
                } else {
                    // Geser buffer untuk mencari header berikutnya
                    buffer[0] = buffer[1];
                    buffer[1] = buffer[2];
                    buffer[2] = buffer[3];
                    bufferIndex = 3; // Tetap di 3 karena kita geser buffer
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}