// -*- mode:c++; fill-column: 100; -*-

#include "openrobot_vesc_driver/vesc_driver.h"

#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>

namespace openrobot_vesc_driver
{
static std::string port;
static std::string baudrate;
static std::string topic_name;

VescDriver::VescDriver(ros::NodeHandle nh,
                       ros::NodeHandle private_nh) :
  vesc_(std::string(),
        std::string(),
        boost::bind(&VescDriver::vescPacketCallback, this, _1),
        boost::bind(&VescDriver::vescErrorCallback, this, _1)),
  duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0), current_limit_(private_nh, "current"),
  brake_limit_(private_nh, "brake"), speed_limit_(private_nh, "speed"),
  position_limit_(private_nh, "position"), servo_limit_(private_nh, "servo", 0.0, 1.0),
  driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
  // get vesc serial port address
  if (!private_nh.getParam("port", port)) {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
    return;
  }

  if (!private_nh.getParam("baudrate", baudrate)) {
    ROS_INFO("VESC communication baudrate is not given, we set the baudrate as default 115200bps.");
    baudrate = "115200bps";
  }

  // attempt to connect to the serial port
  try {
    vesc_.connect(port, baudrate);
  }
  catch (SerialException e) {
    ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
    ros::shutdown();
    return;
  }

  // create vesc state (telemetry) publisher
  topic_name = port + "/sensors/core";
  state_pub_ = nh.advertise<openrobot_vesc_msgs::VescStateStamped>(topic_name.c_str(), 10);
  topic_name = port + "/sensors/customs";
  customs_pub_ = nh.advertise<openrobot_vesc_msgs::VescGetCustomApp>(topic_name.c_str(), 10);

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  topic_name = port + "/sensors/servo_position_command";
  servo_sensor_pub_ = nh.advertise<std_msgs::Float64>(topic_name.c_str(), 10);

  // subscribe to motor and servo command topics
  topic_name = port + "/commands/motor/get_customs";
  get_customs_sub_ = nh.subscribe(topic_name.c_str(), 10, &VescDriver::getCustomsCallback, this); //cdi
  topic_name = port + "/commands/motor/set_customs";
  set_customs_sub_ = nh.subscribe(topic_name.c_str(), 10, &VescDriver::setCustomsCallback, this); //cdi
  topic_name = port + "/commands/motor/alive";
  alive_sub_ = nh.subscribe(topic_name.c_str(), 10, &VescDriver::aliveCallback, this); //cdi
  topic_name = port + "/commands/motor/duty_cycle";
  duty_cycle_sub_ = nh.subscribe(topic_name.c_str(), 10, &VescDriver::dutyCycleCallback, this);
  topic_name = port + "/commands/motor/current";
  current_sub_ = nh.subscribe(topic_name.c_str(), 10, &VescDriver::currentCallback, this);
  topic_name = port + "/commands/motor/brake";
  brake_sub_ = nh.subscribe(topic_name.c_str(), 10, &VescDriver::brakeCallback, this);
  topic_name = port + "/commands/motor/speed";
  speed_sub_ = nh.subscribe(topic_name.c_str(), 10, &VescDriver::speedCallback, this);
  topic_name = port + "/commands/motor/position";
  position_sub_ = nh.subscribe(topic_name.c_str(), 10, &VescDriver::positionCallback, this);
  topic_name = port + "/commands/servo/position";
  servo_sub_ = nh.subscribe(topic_name.c_str(), 10, &VescDriver::servoCallback, this);

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createTimer(ros::Duration(1.0/50.0), &VescDriver::timerCallback, this);
}

  /* TODO or TO-THINKABOUT LIST
    - what should we do on startup? send brake or zero command?
    - what to do if the vesc interface gives an error?
    - check version number against know compatable?
    - should we wait until we receive telemetry before sending commands?
    - should we track the last motor command
    - what to do if no motor command received recently?
    - what to do if no servo command received recently?
    - what is the motor safe off state (0 current?)
    - what to do if a command parameter is out of range, ignore?
    - try to predict vesc bounds (from vesc config) and command detect bounds errors
  */

void VescDriver::timerCallback(const ros::TimerEvent& event)
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d, %s, %dbps",
               fw_version_major_, fw_version_minor_, port.c_str(), std::stoi(baudrate));
      driver_mode_ = MODE_ENABLE;
    }
  }
  else if (driver_mode_ == MODE_ENABLE) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
    //vesc_.setAlive();	//cdi
  }
  else if (driver_mode_ == MODE_DISABLE) {
      // poll for vesc state (telemetry)
      vesc_.requestState();
  }
  else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    openrobot_vesc_msgs::VescStateStamped::Ptr state_msg(new openrobot_vesc_msgs::VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_fet_filtered();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed = values->rpm();
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = values->tachometer();
    state_msg->state.distance_traveled = values->tachometer_abs();
    state_msg->state.fault_code = values->fault_code();
    state_msg->state.pid_pos_now = values->pid_pos_now();
    state_msg->state.controller_id = values->controller_id();

    state_pub_.publish(state_msg);
  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();

    //
    //ROS_INFO("fw_version - length of data : %d", fw_version->length());
  }
  else if (packet->name() == "CustomApp") {
    boost::shared_ptr<VescPacketCustomApp const> custom_data =
        boost::dynamic_pointer_cast<VescPacketCustomApp const>(packet);

    //
    //ROS_INFO("CustomApp - length of data : %d", custom_data->length());

    // todo: publish here
    openrobot_vesc_msgs::VescGetCustomApp::Ptr custom_msg(new openrobot_vesc_msgs::VescGetCustomApp);
    custom_msg->header.stamp = ros::Time::now();
    custom_msg->can_devs_num = custom_data->can_devs_num();

    custom_msg->can_id.clear();
    custom_msg->voltage_input.clear();
    custom_msg->temperature_pcb.clear();
    custom_msg->temperature_motor.clear();
    custom_msg->current_motor.clear();
    custom_msg->current_input.clear();
    custom_msg->duty_cycle.clear();
    custom_msg->watt_hours.clear();
    custom_msg->watt_hours_charged.clear();
    custom_msg->accum_deg_now.clear();
    custom_msg->diff_deg_now.clear();

    for(int i=0; i<=custom_msg->can_devs_num; i++) {
      custom_msg->can_id.push_back(custom_data->can_id(i));
      custom_msg->voltage_input.push_back(custom_data->voltage_input(i));
      custom_msg->temperature_pcb.push_back(custom_data->temp_fet_filtered(i));
      custom_msg->temperature_motor.push_back(custom_data->temp_motor_filtered(i));
      custom_msg->current_motor.push_back(custom_data->current_motor(i));
      custom_msg->current_input.push_back(custom_data->current_in(i));
      custom_msg->duty_cycle.push_back(custom_data->duty_now(i));
      custom_msg->watt_hours.push_back(custom_data->watt_hours(i));
      custom_msg->watt_hours_charged.push_back(custom_data->watt_hours_charged(i));
      custom_msg->accum_deg_now.push_back(custom_data->accum_deg_now(i));
      custom_msg->diff_deg_now.push_back(custom_data->diff_deg_now(i));
    }
	  customs_pub_.publish(custom_msg);
  }
  else if (packet->name() == "CommPrint") {
    boost::shared_ptr<VescPacketCommPrint const> comm_print = boost::dynamic_pointer_cast<VescPacketCommPrint const>(packet);
    // todo: might need lock here
    //ROS_INFO("CommPrint - length of data : %d", comm_print->length());
    //ROS_INFO("%d %d %d %d %d", comm_print->rxmsg1(),comm_print->rxmsg2(),comm_print->rxmsg3(),comm_print->rxmsg4(),comm_print->rxmsg5());
  }
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}

//cdi
void VescDriver::getCustomsCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data==true) {
	  vesc_.getCustomApp();
  }
}

//can_id
void VescDriver::setCustomsCallback(const openrobot_vesc_msgs::VescSetCustomApp::ConstPtr& custom_set_msg)
{
  if (custom_set_msg->num_of_id!=0) {
	  vesc_.setCustomApp(custom_set_msg);
  }
}

//cdi
void VescDriver::aliveCallback(const std_msgs::Bool::ConstPtr& alive)
{
  if (alive->data)
	  driver_mode_ = MODE_ENABLE;
  else
	  driver_mode_ = MODE_DISABLE;
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
//void VescDriver::dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle)
//{
//  if (driver_mode_ = MODE_ENABLE) {
//    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
//  }
//}
void VescDriver::dutyCycleCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& duty_cycle)
{
  if (driver_mode_ = MODE_ENABLE) {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data), duty_cycle->send_can, duty_cycle->can_id);
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
//void VescDriver::currentCallback(const std_msgs::Float64::ConstPtr& current)
//{
//  if (driver_mode_ = MODE_ENABLE) {
//	vesc_.setCurrent(current_limit_.clip(current->data));
//  }
//}

void VescDriver::currentCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& current)
{
  if (driver_mode_ = MODE_ENABLE) {
	vesc_.setCurrent(current_limit_.clip(current->data), current->send_can, current->can_id);
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
//void VescDriver::brakeCallback(const std_msgs::Float64::ConstPtr& brake)
//{
//  if (driver_mode_ = MODE_ENABLE) {
//    vesc_.setBrake(brake_limit_.clip(brake->data));
//  }
//}
//cdi
void VescDriver::brakeCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& brake)
{
  if (driver_mode_ = MODE_ENABLE) {
    vesc_.setBrake(brake_limit_.clip(brake->data), brake->send_can, brake->can_id);
  }
}

/**
 * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
//void VescDriver::speedCallback(const std_msgs::Float64::ConstPtr& speed)
//{
//  if (driver_mode_ = MODE_ENABLE) {
//    vesc_.setSpeed(speed_limit_.clip(speed->data));
//  }
//}

//cdi
void VescDriver::speedCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& speed)
{
  if (driver_mode_ = MODE_ENABLE) {
    vesc_.setSpeed(speed_limit_.clip(speed->data), speed->send_can, speed->can_id);
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
//void VescDriver::positionCallback(const std_msgs::Float64::ConstPtr& position)
//{
//  if (driver_mode_ = MODE_ENABLE) {
//    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
//    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
//    vesc_.setPosition(position_deg);
//  }
//}

void VescDriver::positionCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& position)
{
  if (driver_mode_ = MODE_ENABLE) {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg, position->send_can, position->can_id);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
//void VescDriver::servoCallback(const std_msgs::Float64::ConstPtr& servo)
//{
//  if (driver_mode_ = MODE_ENABLE) {
//    double servo_clipped(servo_limit_.clip(servo->data));
//    vesc_.setServo(servo_clipped);
//    // publish clipped servo value as a "sensor"
//    std_msgs::Float64::Ptr servo_sensor_msg(new std_msgs::Float64);
//    servo_sensor_msg->data = servo_clipped;
//    servo_sensor_pub_.publish(servo_sensor_msg);
//  }
//}

void VescDriver::servoCallback(const openrobot_vesc_msgs::VescSetCommand::ConstPtr& servo)
{
  if (driver_mode_ = MODE_ENABLE) {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped, servo->send_can, servo->can_id);
    // publish clipped servo value as a "sensor"
    std_msgs::Float64::Ptr servo_sensor_msg(new std_msgs::Float64);
    servo_sensor_msg->data = servo_clipped;
    servo_sensor_pub_.publish(servo_sensor_msg);
  }
}

VescDriver::CommandLimit::CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                                       const boost::optional<double>& min_lower,
                                       const boost::optional<double>& max_upper) :
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (nh.getParam(name + "_min", param_min)) {
    if (min_lower && param_min < *min_lower) {
      lower = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min > *max_upper) {
      lower = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      lower = param_min;
    }
  }
  else if (min_lower) {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (nh.getParam(name + "_max", param_max)) {
    if (min_lower && param_max < *min_lower) {
      upper = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max > *max_upper) {
      upper = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      upper = param_max;
    }
  }
  else if (max_upper) {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper) {
    ROS_WARN_STREAM("Parameter " << name << "_max (" << *upper
                    << ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower) oss << *lower << " "; else oss << "(none) ";
  if (upper) oss << *upper; else oss << "(none)";
  ROS_DEBUG_STREAM(oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.",
                      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.",
                      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}


} // namespace openrobot_vesc_driver
