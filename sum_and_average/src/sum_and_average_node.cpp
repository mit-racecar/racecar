// This work is sponsored by the Department of the Air Force under Air Force
// Contract FA8721-05-C-0002. Opinions, interpretations, conclusions, and
// recommendations are those of the author and are not necessarily endorsed by
// the United States Government.

#include "ros/ros.h" // main ROS include
#include "std_msgs/Float32.h" // number message datatype
#include <cmath> // needed for nan

// simple class to contain the node's variables and code
class SumAndAverageNode
{
	public:
		SumAndAverageNode(); // constructor

	private:
		ros::NodeHandle nh_; // interface to this node
		ros::Subscriber number_stream_sub_; // interface to the topic subscription
		ros::Publisher sum_pub_; // interface to the sum topic publication
		ros::Publisher moving_average_pub_; // interface to the moving average topic pub
		ros::Timer timer_; // the timer object

		double running_sum_; // the running sum since start
		double moving_average_period_; // how often to sample the moving average
		double moving_average_sum_; // sum since the last moving average update
		int moving_average_count_; // number of samples since the last moving average update

		// callback function declarations
		void numberStreamCallback(std_msgs::Float32::ConstPtr const& msg);
		void timerCallback(ros::TimerEvent const& event);
};

// program entry point
int main(int argc, char *argv[])
{
	// initialize the ROS client API, giving the default node name
	ros::init(argc, argv, "sum_and_average_node");

	SumAndAverageNode node;

	// enter the ROS main loop
	ros::spin();

	return 0;
}

// class constructor; subscribe to topics and advertise intent to publish
SumAndAverageNode::SumAndAverageNode() :
 	running_sum_(0), moving_average_period_(30), moving_average_sum_(0), moving_average_count_(0)
{
	// subscribe to the number stream topic
	number_stream_sub_ = nh_.subscribe("number_stream", 1, &SumAndAverageNode::numberStreamCallback, this);

	// advertise that we'll publish on the sum and moving_average topics
	sum_pub_ = nh_.advertise<std_msgs::Float32>("sum", 1);
	moving_average_pub_ = nh_.advertise<std_msgs::Float32>("moving_average", 1);

	// get moving average period from parameter server (or use default value if not present)
	ros::NodeHandle private_nh("~");
	private_nh.param("moving_average_period", moving_average_period_, moving_average_period_);
	if (moving_average_period_ < 0.5)
		moving_average_period_ = 0.5;
	
	// create the Timer with period moving_average_period_
	timer_ = nh_.createTimer(ros::Duration(moving_average_period_), &SumAndAverageNode::timerCallback, this);

	ROS_INFO("Created timer with period of %f seconds", moving_average_period_);
}

// the callback function for the number stream topic subscription
void SumAndAverageNode::numberStreamCallback(std_msgs::Float32::ConstPtr const& msg)
{
	// add the data to the running sums
	running_sum_ = running_sum_ + msg->data;
	moving_average_sum_ = moving_average_sum_ + msg->data;

	// increment the moving average counter
	moving_average_count_++;

	// create a message containing the running total
	std_msgs::Float32 sum_msg;
	sum_msg.data = running_sum_;

	// publish the running sum message
	sum_pub_.publish(sum_msg);
}

// the callback function for the timer event
void SumAndAverageNode::timerCallback(ros::TimerEvent const& event)
{
	// create the message containing the moving average
	std_msgs::Float32 moving_average_msg;
	if (moving_average_count_ > 0)
		moving_average_msg.data = moving_average_sum_ / moving_average_count_;
	else
		moving_average_msg.data = nan("");

	// publish the moving average
	moving_average_pub_.publish(moving_average_msg);

	// reset the moving average
	moving_average_sum_ = 0;
	moving_average_count_ = 0;
}

