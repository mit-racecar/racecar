/*
 * Copyright (C) 2014, Massachusetts Institute of Technology
 * All rights reserved.
 *
 * This work is sponsored by the Department of the Air Force Air Force
 * contract number: FA8721-05-C-0002. The opinions, interpretations,
 * recommendations, and conclusions are those of the author and are
 * not necessarily endorsed by the United Stated Government.
 */

#ifndef PWM_SYSFS_DRIVER_SERVO_DRIVER_NODE_H_
#define PWM_SYSFS_DRIVER_SERVO_DRIVER_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <boost/shared_ptr.hpp>

namespace pwm_sysfs_driver
{

class PwmSysfsDriver;

class ServoDriverNode
{
 public:
  ServoDriverNode(ros::NodeHandle const& nh=ros::NodeHandle(),
                  ros::NodeHandle const& nh_rel=ros::NodeHandle("~"));
  ~ServoDriverNode();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_rel_; // private / relative node handle

  boost::shared_ptr<PwmSysfsDriver> driver_;

  // node parameters
  double timeout_;
  bool disable_on_timeout_;
  double timeout_command_;
  bool disable_on_exit_;
  unsigned int max_pulse_width_ns_, min_pulse_width_ns_;

  // internal state variables
  bool enabled_;
  bool timeout_active_;
  ros::Time last_command_msg_time_;
  
  // timeout timer
  ros::Timer timer_;
  void timerCallback(ros::TimerEvent const& event);

  // command subscriber
  ros::Subscriber command_sub_;
  bool setCommand(double command);
  void commandCallback(std_msgs::Float64::ConstPtr const& msg);
};

} // namespace pwm_sysfs_driver

#endif // #ifndef PWM_SYSFS_DRIVER_SERVO_DRIVER_NODE_H_
