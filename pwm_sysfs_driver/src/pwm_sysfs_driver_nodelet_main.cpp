/*
 * Copyright (C) 2014, Massachusetts Institute of Technology
 * All rights reserved.
 *
 * This work is sponsored by the Department of the Air Force Air Force
 * contract number: FA8721-05-C-0002. The opinions, interpretations,
 * recommendations, and conclusions are those of the author and are
 * not necessarily endorsed by the United Stated Government.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <boost/shared_ptr.hpp>

#include "pwm_sysfs_driver/pwm_sysfs_driver_node.h"

namespace pwm_sysfs_driver
{

class PwmSysfsDriverNodelet: public nodelet::Nodelet
{
public:
  PwmSysfsDriverNodelet() {}

private:
  virtual void onInit()
  {
    node_.reset(new PwmSysfsDriverNode(getNodeHandle(),
                                       getPrivateNodeHandle()));
  }

  boost::shared_ptr<PwmSysfsDriverNode> node_;
};

} // namespace pwm_sysfs_driver

PLUGINLIB_EXPORT_CLASS(pwm_sysfs_driver::PwmSysfsDriverNodelet, nodelet::Nodelet);
