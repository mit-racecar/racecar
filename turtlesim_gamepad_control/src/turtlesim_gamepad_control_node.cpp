// This work is sponsored by the Department of the Air Force under Air Force
// Contract FA8721-05-C-0002. Opinions, interpretations, conclusions, and
// recommendations are those of the author and are not necessarily endorsed by
// the United States Government.

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

class TurtlesimGamepadNode
{
public:
  TurtlesimGamepadNode();

private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber joy_sub_;

  double linear_scale_;
  double angular_scale_;

  void joyCallback(sensor_msgs::Joy::ConstPtr const& reset);
};

TurtlesimGamepadNode::TurtlesimGamepadNode() :
  linear_scale_(2.0), angular_scale_(2.0)
{
  // subscribe to the joy topic
  joy_sub_ = nh_.subscribe("joy", 1, &TurtlesimGamepadNode::joyCallback, this);

  // advertise that we'll publish on the turtlesim command velocity topic
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void TurtlesimGamepadNode::joyCallback(sensor_msgs::Joy::ConstPtr const& joy_msg)
{
  // simple check for invalid joy_msg
  if (joy_msg->axes.size() < 2) {
    ROS_ERROR("joy_msg axes array length (%u) has less than expected length (2)",
	      (unsigned)joy_msg->axes.size());
    return;
  }

  // convert the joystick message to a velocity
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = joy_msg->axes[1]*linear_scale_;
  cmd_vel_msg.linear.y = 0;
  cmd_vel_msg.linear.z = 0;
  cmd_vel_msg.angular.x = 0; 
  cmd_vel_msg.angular.y = 0; 
  cmd_vel_msg.angular.z = joy_msg->axes[0]*angular_scale_;

  // publish the velocity command message
  cmd_vel_pub_.publish(cmd_vel_msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "turtlesim_gamepad");
  TurtlesimGamepadNode node;
  ros::spin();
  return 0;
}
