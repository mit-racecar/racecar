/*
 * Copyright (C) 2014, Massachusetts Institute of Technology
 * All rights reserved.
 *
 * This work is sponsored by the Department of the Air Force Air Force
 * contract number: FA8721-05-C-0002. The opinions, interpretations,
 * recommendations, and conclusions are those of the author and are
 * not necessarily endorsed by the United Stated Government.
 */

#ifndef PWM_SYSFS_DRIVER_PWM_SYSFS_DRIVER_NODE_H_
#define PWM_SYSFS_DRIVER_PWM_SYSFS_DRIVER_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <boost/shared_ptr.hpp>

namespace pwm_sysfs_driver
{

class PwmSysfsDriver;

class PwmSysfsDriverNode
{
 public:
  PwmSysfsDriverNode(ros::NodeHandle const& nh=ros::NodeHandle(),
                     ros::NodeHandle const& nh_rel=ros::NodeHandle("~"));
  ~PwmSysfsDriverNode();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_rel_; // private / relative node handle

  boost::shared_ptr<PwmSysfsDriver> driver_;

  void setEnable(bool enable);
  void setInvertPolarity(bool invert_polarity);
  void setPeriod(double period);
  void setDuty(double duty);

  ros::Subscriber enable_sub_;
  ros::Subscriber invert_polarity_sub_;
  ros::Subscriber period_sub_;
  ros::Subscriber duty_sub_;

  void enableCallback(std_msgs::Bool::ConstPtr const& msg);
  void invertPolarityCallback(std_msgs::Bool::ConstPtr const& msg);
  void periodCallback(std_msgs::Float64::ConstPtr const& msg);
  void dutyCallback(std_msgs::Float64::ConstPtr const& msg);
};

} // namespace pwm_sysfs_driver

#endif // #ifndef PWM_SYSFS_DRIVER_PWM_SYSFS_DRIVER_NODE_H_
