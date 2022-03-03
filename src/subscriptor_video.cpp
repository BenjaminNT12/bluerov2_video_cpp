#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
// #include <sensor_msgs/MagneticField.h>
// #include <sensor_msgs/FluidPressure.h>
// #include <sensor_msgs/Temperature.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("BlueRov2_view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("No es posible convertir '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

    ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "BlueRov2_image");
  ros::init(argc, argv, "BlueRov2_imu");

  ros::NodeHandle nh;
  ros::NodeHandle n;
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/BlueRov2/camera/image_raw", 1, imageCallback);
  ros::Subscriber subImu = n.subscribe("/BlueRov2/imu/data", 10, imuCallback);
  ros::spin();
  cv::destroyWindow("BlueRov2_view");

}
