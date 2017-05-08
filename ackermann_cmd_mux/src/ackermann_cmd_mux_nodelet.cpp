/**
 * @file /src/ackermann_cmd_mux_nodelet.cpp
 *
 * @brief  Implementation for the ackermann command multiplexer
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_ackermann_cmd_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>

#include "ackermann_cmd_mux/ackermann_cmd_mux_nodelet.hpp"
#include "ackermann_cmd_mux/exceptions.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ackermann_cmd_mux {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void AckermannCmdMuxNodelet::ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg, unsigned int idx)
{
	// Reset timer for this source
	ackermann_cmd_sub[idx].timer.stop();
	ackermann_cmd_sub[idx].timer.start();

	ackermann_cmd_sub[idx].active = true;   // obviously his source is sending commands, so active

	// Give permit to publish to this source if it's the only active or is
	// already allowed or has higher priority that the currently allowed
	if ((ackermann_cmd_sub.allowed == VACANT) ||
			(ackermann_cmd_sub.allowed == idx)    ||
			(ackermann_cmd_sub[idx].priority > ackermann_cmd_sub[ackermann_cmd_sub.allowed].priority))
	{
		if (ackermann_cmd_sub.allowed != idx)
		{
			ackermann_cmd_sub.allowed = idx;

			// Notify the world that a new ackermann_cmd source took the control
			std_msgs::StringPtr acv_msg(new std_msgs::String);
			acv_msg->data = ackermann_cmd_sub[idx].name;
		if (active_subscriber) {
				active_subscriber.publish(acv_msg);
			}
		}

	if (mux_ackermann_cmd_pub){
			mux_ackermann_cmd_pub.publish(msg);
		}
	}
}

void AckermannCmdMuxNodelet::timerCallback(const ros::TimerEvent& event, unsigned int idx)
{
	if (ackermann_cmd_sub.allowed == idx)
	{
		// No ackermann_cmd messages timeout happened to currently active source, so...
		ackermann_cmd_sub.allowed = VACANT;

		// ...notify the world that nobody is publishing on ackermann_cmd; its vacant
		std_msgs::StringPtr acv_msg(new std_msgs::String);
		acv_msg->data = "idle";
		if (active_subscriber){
			active_subscriber.publish(acv_msg);
		}
	}

	ackermann_cmd_sub[idx].active = false;
}

void AckermannCmdMuxNodelet::onInit()
{
	ros::NodeHandle &nh = this->getPrivateNodeHandle();

	/*********************
	** Dynamic Reconfigure
	**********************/
	dynamic_reconfigure_cb = boost::bind(&AckermannCmdMuxNodelet::reloadConfiguration, this, _1, _2);
	dynamic_reconfigure_server = new dynamic_reconfigure::Server<ackermann_cmd_mux::reloadConfig>(nh);
	dynamic_reconfigure_server->setCallback(dynamic_reconfigure_cb);

	active_subscriber = nh.advertise <std_msgs::String> ("active", 1, true); // latched topic

	// Notify the world that by now nobody is publishing on ackermann_cmd yet
	std_msgs::StringPtr active_msg(new std_msgs::String);
	active_msg->data = "idle";
	if (active_subscriber) {
		active_subscriber.publish(active_msg);
	}

	// could use a call to reloadConfiguration here, but it seems to automatically call it once with defaults anyway.
	NODELET_DEBUG("AckermannCmdMux : successfully initialised");
}

void AckermannCmdMuxNodelet::reloadConfiguration(ackermann_cmd_mux::reloadConfig &config, uint32_t unused_level)
{
	std::string yaml_cfg_file;
	ros::NodeHandle &nh = this->getNodeHandle();
	ros::NodeHandle &nh_priv = this->getPrivateNodeHandle();
	if( config.yaml_cfg_file == "" )
	{
		// typically fired on startup, so look for a parameter to set a default
		nh_priv.getParam("yaml_cfg_file", yaml_cfg_file);
	}
	else
	{
		yaml_cfg_file = config.yaml_cfg_file;
	}

	/*********************
	** Yaml File Parsing
	**********************/
	std::ifstream ifs(yaml_cfg_file.c_str(), std::ifstream::in);
	if (ifs.good() == false)
	{
		NODELET_ERROR_STREAM("AckermannCmdMux : configuration file not found [" << yaml_cfg_file << "]");
		return;
	}
	// probably need to bring the try catches back here
	YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
	doc = YAML::Load(ifs);
#else
	YAML::Parser parser(ifs);
	parser.GetNextDocument(doc);
#endif

	/*********************
	** Output Publisher
	**********************/
	std::string output_name("output");
#ifdef HAVE_NEW_YAMLCPP
	if ( doc["publisher"] ) {
		doc["publisher"] >> output_name;
	}
#else
	const YAML::Node *node = doc.FindValue("publisher");
	if ( node != NULL ) {
		*node >> output_name;
	}
#endif
	mux_ackermann_cmd_pub = nh_priv.advertise <ackermann_msgs::AckermannDriveStamped> (output_name, 10);

	/*********************
	** Input Subscribers
	**********************/
	try {
		ackermann_cmd_sub.configure(doc["subscribers"]);
	}
	catch(EmptyCfgException& e) {
		NODELET_WARN("AckermannCmdMux : yaml configured zero subscribers, check yaml content.");
	}
	catch(YamlException& e) {
		NODELET_ERROR_STREAM("AckermannCmdMux : yaml parsing problem [" << std::string(e.what()) + "]");
	}

	// Publishers and subscribers
	for (unsigned int i = 0; i < ackermann_cmd_sub.size(); i++)
	{
		ackermann_cmd_sub[i].subs =
				nh_priv.subscribe<ackermann_msgs::AckermannDriveStamped>(ackermann_cmd_sub[i].topic, 10, AckermannCmdFunctor(i, this));

		// Create (stopped by now) a one-shot timer for every subscriber
		ackermann_cmd_sub[i].timer =
				nh_priv.createTimer(ros::Duration(ackermann_cmd_sub[i].timeout), TimerFunctor(i, this), true, false);

		NODELET_DEBUG("AckermannCmdMux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
							ackermann_cmd_sub[i].name.c_str(), ackermann_cmd_sub[i].topic.c_str(),
							ackermann_cmd_sub[i].priority, ackermann_cmd_sub[i].timeout);
	}

	NODELET_INFO_STREAM("AckermannCmdMux : (re)configured [" << yaml_cfg_file << "]");
	ifs.close();
}

} // namespace ackermann_cmd_mux

PLUGINLIB_EXPORT_CLASS(ackermann_cmd_mux::AckermannCmdMuxNodelet, nodelet::Nodelet);
