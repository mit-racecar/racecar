/*
 * Copyright (C) 2014, Massachusetts Institute of Technology
 * All rights reserved.
 *
 * This work is sponsored by the Department of the Air Force Air Force
 * contract number: FA8721-05-C-0002. The opinions, interpretations,
 * recommendations, and conclusions are those of the author and are
 * not necessarily endorsed by the United Stated Government.
 */

#include "pwm_sysfs_driver/pwm_sysfs_driver_node.h"

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

PwmSysfsDriverNode::PwmSysfsDriverNode(ros::NodeHandle const& nh,
                                       ros::NodeHandle const& nh_rel):
  nh_(nh), nh_rel_(nh_rel)
{
  // get reqired path to sysfs pwm directory parameter
  std::string pwm_sysfs_dir;
  if (!nh_rel_.getParam("pwm_sysfs_dir", pwm_sysfs_dir)) {
    ROS_FATAL("Path to sysfs pwm directory required.");
    ros::shutdown();
    return;
  }

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

  // set initial pwm values
  bool enable;
  if (nh_rel_.getParam("initial_enable", enable))
    setEnable(enable);

  bool invert_polarity;
  if (nh_rel_.getParam("initial_invert_polarity", invert_polarity))
    setInvertPolarity(invert_polarity);

  double period;
  if (nh_rel_.getParam("initial_period", period))
    setPeriod(period);

  double duty;
  if (nh_rel_.getParam("initial_duty", duty))
    setDuty(duty);

  // boolean pwm enable
  enable_sub_ = nh_.subscribe("enable", 10,
                              &PwmSysfsDriverNode::enableCallback, this);

  // boolean pwm invert polarity
  invert_polarity_sub_ =
    nh_.subscribe("invert_polarity", 10,
                  &PwmSysfsDriverNode::invertPolarityCallback, this);

  // pwm cycle period, in nanoseconds
  period_sub_ = nh_.subscribe("period", 10,
                              &PwmSysfsDriverNode::periodCallback, this);

  // pwm duty cycle fraction, 0 to 1
  duty_sub_ = nh_.subscribe("duty", 10,
                            &PwmSysfsDriverNode::dutyCallback, this);
}
  
PwmSysfsDriverNode::~PwmSysfsDriverNode()
{
  // disable PWM
  if (driver_)
    driver_->enable(false);
}

void PwmSysfsDriverNode::setEnable(bool enable)
{
  ROS_ASSERT(driver_);
  ROS_DEBUG("Setting PWM enable state to: %s", enable ? "true" : "false");
  driver_->enable(enable);
}

void PwmSysfsDriverNode::enableCallback(std_msgs::Bool::ConstPtr const& msg)
{
  setEnable(msg->data);
}

void PwmSysfsDriverNode::setInvertPolarity(bool invert_polarity)
{
  ROS_ASSERT(driver_);

  PwmSysfsDriver::PwmPolarity polarity;
  if (invert_polarity)
    polarity = PwmSysfsDriver::PWM_POLARITY_INVERSED;
  else
    polarity = PwmSysfsDriver::PWM_POLARITY_NORMAL;

  ROS_DEBUG("Setting PWM invert polarity state to: %s.",
            invert_polarity ? "true" : "false");
  driver_->polarity(polarity);      
}

void PwmSysfsDriverNode::invertPolarityCallback(std_msgs::Bool::ConstPtr const& msg)
{
  setInvertPolarity(msg->data);
}

void PwmSysfsDriverNode::setPeriod(double period)
{
  ROS_ASSERT(driver_);
  if (period > 0 && period <= UINT_MAX) {
    ROS_DEBUG("Setting PWM period to: %f", period);
    driver_->period(period);
  }
  else
    ROS_WARN("Commanded PWM period value (%f) invalid, ignoring.", period);
}

void PwmSysfsDriverNode::periodCallback(std_msgs::Float64::ConstPtr const& msg)
{
  setPeriod(msg->data);
}

void PwmSysfsDriverNode::setDuty(double duty)
{
  ROS_ASSERT(driver_);
  if (duty >= 0 && duty <= 1) {
    ROS_DEBUG("Setting PWM duty cycle to: %f", duty);
    driver_->duty_fraction(duty);
  }
  else
    ROS_WARN("Commanded PWM duty cycle value (%f) invalid, ignoring.", duty);
}

void PwmSysfsDriverNode::dutyCallback(std_msgs::Float64::ConstPtr const& msg)
{
  setDuty(msg->data);
}

} // namespace pwm_sysfs_driver
