#include <ros/ros.h>

#include "openrobot_vesc_driver/vesc_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "openrobot_vesc_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  openrobot_vesc_driver::VescDriver openrobot_vesc_driver(nh, private_nh);

  ros::spin();

  return 0;
}
