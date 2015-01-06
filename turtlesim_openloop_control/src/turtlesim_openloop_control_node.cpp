// This work is sponsored by the Department of the Air Force under Air Force
// Contract FA8721-05-C-0002. Opinions, interpretations, conclusions, and
// recommendations are those of the author and are not necessarily endorsed by
// the United States Government.

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class TurtlesimOpenLoopNode
{
public:
  TurtlesimOpenLoopNode();

private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Timer timer_;

  double period_;
  unsigned counter_; // state variable

  void timerCallback(ros::TimerEvent const& event);
};

TurtlesimOpenLoopNode::TurtlesimOpenLoopNode() :
  counter_(0)
{
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // create a periodic timer at a fixed rate
  period_ = 0.1; // 10 Hz
  timer_ = nh_.createTimer(ros::Duration(period_), &TurtlesimOpenLoopNode::timerCallback, this);
}

void TurtlesimOpenLoopNode::timerCallback(ros::TimerEvent const& event)
{
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0;
  cmd_vel_msg.linear.y = 0;
  cmd_vel_msg.linear.z = 0;
  cmd_vel_msg.angular.x = 0;
  cmd_vel_msg.angular.y = 0;
  cmd_vel_msg.angular.z = 0;

  /* go forward for 3 seconds (30 counts at 10hz) at 1.1 meters/second,
   * then turn at 45 deg/sec for 2 seconds (20 counts) */
  if (counter_ < 30)
    cmd_vel_msg.linear.x = 1.1;
  else
    cmd_vel_msg.angular.z = 0.785398163;

  // check for end of cycle
  counter_++;
  if (counter_ >= 50)
    counter_ = 0;

  cmd_vel_pub_.publish(cmd_vel_msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "turtlesim_openloop_control_node");
  TurtlesimOpenLoopNode node;
  ros::spin();
  return 0;
}
