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

#include "pwm_sysfs_driver/servo_driver_node.h"

namespace pwm_sysfs_driver
{

class ServoDriverNodelet: public nodelet::Nodelet
{
public:
  ServoDriverNodelet() {}

private:
  virtual void onInit()
  {
    node_.reset(new ServoDriverNode(getNodeHandle(),
                                       getPrivateNodeHandle()));
  }

  boost::shared_ptr<ServoDriverNode> node_;
};

} // namespace pwm_sysfs_driver

PLUGINLIB_EXPORT_CLASS(pwm_sysfs_driver::ServoDriverNodelet, nodelet::Nodelet);
