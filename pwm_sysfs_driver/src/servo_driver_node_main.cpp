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

#include "pwm_sysfs_driver/servo_driver_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "servo_driver_node");

  pwm_sysfs_driver::ServoDriverNode node;

  ros::spin();

  return 0;
}
