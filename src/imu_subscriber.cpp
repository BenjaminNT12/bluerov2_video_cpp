#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

/**
 * Subscriber callbacks
 */

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

/*
void imuTempCallback(const sensor_msgs::Temperature::ConstPtr& msg) {
    ROS_INFO( "IMU temperature: %.2f [C]",
              msg->temperature);
}


void leftTempCallback(const sensor_msgs::Temperature::ConstPtr& msg) {
    ROS_INFO( "Left CMOS temperature: %.2f [C]",
              msg->temperature);
}


void rightTempCallback(const sensor_msgs::Temperature::ConstPtr& msg) {
    ROS_INFO( "Right CMOS temperature: %.2f [C]",
              msg->temperature);
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
    ROS_INFO( "Mag. Field: %.3f,%.3f,%.3f [uT]",
              msg->magnetic_field.x*1e-6, msg->magnetic_field.y*1e-6, msg->magnetic_field.z*1e-6);
}

void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) {
    ROS_INFO( "Atmospheric Pressure: %.2f [hPa]",
              msg->fluid_pressure*100.f);
}
*/
/**
 * Node main function
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "BlueRov2_subscriber");
    ros::NodeHandle n;

    ros::Subscriber subImu = n.subscribe("/BlueRov2/imu/data", 10, imuCallback);
    // ros::Subscriber subImuTemp = n.subscribe("/zed/zed_node/imu/temperature", 10, imuTempCallback);
    // ros::Subscriber subLeftTemp = n.subscribe("/zed/zed_node/temperature/left", 10, leftTempCallback);
    // ros::Subscriber subRightTemp = n.subscribe("/zed/zed_node/temperature/right", 10, rightTempCallback);
    // ros::Subscriber subPress = n.subscribe("/zed/zed_node/atm_press", 10, pressureCallback);
    // ros::Subscriber subMag = n.subscribe("/zed/zed_node/imu/mag", 10, magCallback);

    ros::spin();

    return 0;
}
