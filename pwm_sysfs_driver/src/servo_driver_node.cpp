/*
 * Copyright (C) 2014, Massachusetts Institute of Technology
 * All rights reserved.
 *
 * This work is sponsored by the Department of the Air Force Air Force
 * contract number: FA8721-05-C-0002. The opinions, interpretations,
 * recommendations, and conclusions are those of the author and are
 * not necessarily endorsed by the United Stated Government.
 */

#include "pwm_sysfs_driver/servo_driver_node.h"

#include <string>
#include <climits>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <boost/make_shared.hpp>

#include "pwm_sysfs_driver/pwm_sysfs_driver.h"

namespace pwm_sysfs_driver
{

ServoDriverNode::ServoDriverNode(ros::NodeHandle const& nh,
                                       ros::NodeHandle const& nh_rel):
  nh_(nh), nh_rel_(nh_rel), enabled_(false), timeout_active_(false)
{
  // get reqired path to sysfs pwm directory parameter
  std::string pwm_sysfs_dir;
  if (!nh_rel_.getParam("pwm_sysfs_dir", pwm_sysfs_dir)) {
    ROS_FATAL("Path to sysfs pwm directory required.");
    ros::shutdown();
    return;
  }

  // servo "update_rate" / "frame rate" / frequency in hz
  double update_rate;
  nh_rel_.param("update_rate", update_rate, 100.0);
  if (update_rate < 40 || update_rate > 400) {
    ROS_FATAL("Update rate out of range, must be between 40 and 400 Hz.");
    ros::shutdown();
    return;
  }
  unsigned int period(1e9L/update_rate);

  // maximum / minimum pulse width, in miliseconds
  double max_pulse_width_ms, min_pulse_width_ms;
  nh_rel_.param("min_pulse_width", min_pulse_width_ms, 1.0);
  if (min_pulse_width_ms < 0) {
    ROS_FATAL("Minimum pulse width is out of range, must be >= zero.");
    ros::shutdown();
    return;
  }
  nh_rel_.param("max_pulse_width", max_pulse_width_ms, 2.0);
  if (min_pulse_width_ms >= max_pulse_width_ms) {
    ROS_FATAL("Minimum pulse width must be less than maximum pulse width.");
    ros::shutdown();
    return;
  }
  if (max_pulse_width_ms * 1e6L > (double)UINT_MAX) {
    ROS_FATAL("Maximum pulse width out of range.");
    ros::shutdown();
    return;
  }
  max_pulse_width_ns_ = max_pulse_width_ms * 1e6L;
  min_pulse_width_ns_ = min_pulse_width_ms * 1e6L;
  if (max_pulse_width_ns_ > (double)period) {
    ROS_FATAL("Maximum pulse width is longer than the PWM period.");
    ros::shutdown();
    return;
  }

  // invert signal polarity
  bool invert_polarity;
  nh_rel_.param("invert_polarity", invert_polarity, false);

  // initial command value
  bool set_initial_command(false);
  double initial_command;
  if (nh_rel_.getParam("initial_command", initial_command)) {
    if (initial_command < -1 || initial_command > 1) {
      ROS_FATAL("Initial command value out of range, must be between -1 and 1.");
      ros::shutdown();
      return;
    }
    set_initial_command = true;
  }

  // no command input timeout in seconds; no timeout if value is zero
  nh_rel_.param("timeout", timeout_, 0.0);
  if (timeout_ < 0.1 && timeout_ != 0) {
    ROS_FATAL("Invalid timeout value, must be zero (disabled) or greater-than 0.100.");
    ros::shutdown();
    return;
  }

  // if timeout_disable is true, the node will disable PWM upon timeout condition
  nh_rel_.param("disable_on_timeout", disable_on_timeout_, false);

  // command will be set to timeout_command upon timeout condition
  nh_rel_.param("timeout_command", timeout_command_, 0.0);
  if (timeout_command_ < -1 || timeout_command_ > 1) {
    ROS_FATAL("Timeout command value out of range, must be between -1 and 1.");
    ros::shutdown();
    return;
  }

  // if true, node will disable the PWM when it is terminated
  nh_rel_.param("disable_on_exit", disable_on_exit_, true);

  // attemp to create driver
  try {
    driver_.reset(new PwmSysfsDriver(pwm_sysfs_dir));
  }
  catch (PwmSysfsDriverException e) {
    driver_.reset();
    ROS_FATAL("Failed to initialize driver: %s", e.what());
    ros::shutdown();
    return;
  }

  // set driver initial values
  PwmSysfsDriver::PwmPolarity polarity;
  if (invert_polarity)
    polarity = PwmSysfsDriver::PWM_POLARITY_INVERSED;
  else
    polarity = PwmSysfsDriver::PWM_POLARITY_NORMAL;
  ROS_DEBUG("Setting PWM invert polarity state to: %s.",
            invert_polarity ? "true" : "false");
  driver_->polarity(polarity);      
  ROS_DEBUG("Setting PWM period to: %u", period);
  driver_->period(period);
  if (set_initial_command) {
    setCommand(initial_command);
  }

  // timeout timer
  if (timeout_ > 0) {
    // duration is 10% of timeout_ value, but no faster than 50Hz
    double timer_duration(timeout_ * 0.1);
    if (timer_duration > 0.02)
      timer_duration = 0.02;
    timer_ = nh_.createTimer(ros::Duration(timer_duration),
                             &ServoDriverNode::timerCallback, this);
  }

  // input command topic subscription
  command_sub_ = nh_.subscribe("command", 10,
                               &ServoDriverNode::commandCallback, this);
}
  
ServoDriverNode::~ServoDriverNode()
{
  // disable PWM
  if (driver_ && disable_on_exit_) {
    driver_->enable(false);
    enabled_ = false;
  }
}

bool ServoDriverNode::setCommand(double command)
{
  ROS_ASSERT(driver_);
  if (command < -1 || command > 1) {
    ROS_WARN("Servo command out of range, must be between -1 and 1, ignoring.");
    return false;
  }
  unsigned int command_pulse_width_ns = 
    (command + 1.0)/2.0*(max_pulse_width_ns_-min_pulse_width_ns_)+min_pulse_width_ns_;
  ROS_DEBUG("Setting PWM duty period to: %u", command_pulse_width_ns);
  driver_->duty_period(command_pulse_width_ns);

  // enable PWM output if not enabled
  if (!enabled_) {
    driver_->enable(true);
    enabled_ = true;
  }

  return true;
}

void ServoDriverNode::commandCallback(std_msgs::Float64::ConstPtr const& msg)
{
  if (setCommand(msg->data)) {
    if (timeout_active_) {
      ROS_WARN("Servo driver received a command, timeout condition is no "
               "longer active.");
    }
    timeout_active_ = false;
    last_command_msg_time_ = ros::Time::now();
  }
}

void ServoDriverNode::timerCallback(ros::TimerEvent const& event)
{
  ros::Duration elapsed(ros::Time::now() - last_command_msg_time_);
  if (timeout_active_ == false && ros::Duration(timeout_) < elapsed) {
    // enter timeout
    ROS_WARN("Servo driver has not received a command, timeout elapsed.");
    timeout_active_ = true;
    if (disable_on_timeout_) {
      ROS_ASSERT(driver_);
      driver_->enable(false);
      enabled_ = false;
    }
    else {
      setCommand(timeout_command_);
    }
  }
}

} // namespace pwm_sysfs_driver
