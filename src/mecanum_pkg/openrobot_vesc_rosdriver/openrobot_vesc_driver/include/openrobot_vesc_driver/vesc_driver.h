// -*- mode:c++; fill-column: 100; -*-

#ifndef OPENROBOT_VESC_DRIVER_VESC_DRIVER_H_
#define OPENROBOT_VESC_DRIVER_VESC_DRIVER_H_

#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <boost/optional.hpp>

#include "openrobot_vesc_driver/vesc_interface.h"
#include "openrobot_vesc_driver/vesc_packet.h"
#include "openrobot_vesc_msgs/VescSetCommand.h"	//cdi
//#include "vesc_msgs/VescGetSpeeds.h"	//cdi
#include "openrobot_vesc_msgs/VescStateStamped.h"	//cdi
#include "openrobot_vesc_msgs/VescGetCustomApp.h"	//cdi
#include "openrobot_vesc_msgs/VescSetCustomApp.h"	//cdi

namespace openrobot_vesc_driver
{

class VescDriver
{
public:

  VescDriver(ros::NodeHandle nh,
             ros::NodeHandle private_nh);

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                 const boost::optional<double>& min_lower = boost::optional<double>(),
                 const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // ROS services
  ros::Publisher state_pub_;
  ros::Publisher customs_pub_;		//cdi
  ros::Publisher servo_sensor_pub_;
  ros::Subscriber duty_cycle_sub_;
  ros::Subscriber current_sub_;
  ros::Subscriber brake_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber servo_sub_;
  ros::Subscriber alive_sub_;		//cdi
  ros::Subscriber get_customs_sub_;	//cdi
  ros::Subscriber set_customs_sub_; //cdi
  ros::Timer timer_;

  // driver modes (possible states)
  typedef enum {
    MODE_INITIALIZING,
    MODE_ENABLE,
	  MODE_DISABLE,
  } driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc

  // ROS callbacks
  void getCustomsCallback(const std_msgs::Bool::ConstPtr& msg);
  void setCustomsCallback(const openrobot_vesc_msgs::VescSetCustomApp::ConstPtr& custom_set_msg);
  void aliveCallback(const std_msgs::Bool::ConstPtr& alive);
  void timerCallback(const ros::TimerEvent& event);
  void dutyCycleCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& duty_cycle);
  void currentCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& current);
  void speedCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& speed);
  void brakeCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& brake);
  void positionCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& position);
  void servoCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& servo);
};

} // namespace openrobot_vesc_driver

#endif // OPENROBOT_VESC_DRIVER_VESC_DRIVER_H_
