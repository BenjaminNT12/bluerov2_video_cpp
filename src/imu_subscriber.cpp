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
/**
 * Node main function
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "BlueRov2_subscriber");
    ros::NodeHandle n;

    ros::Subscriber subImu = n.subscribe("/BlueRov2/imu/data", 10, imuCallback);
    ros::spin();

    return 0;
}
