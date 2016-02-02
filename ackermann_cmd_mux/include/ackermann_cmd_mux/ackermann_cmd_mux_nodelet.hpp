/**
 * @file /include/ackermann_cmd_mux/ackermann_cmd_mux_nodelet.hpp
 *
 * @brief Structure for the ackermann_cmd_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_ackermann_cmd_mux/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef RACECAR_ACKERMANN_CMD_MUX_HPP_
#define RACECAR_ACKERMANN_CMD_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include "ackermann_cmd_mux/reloadConfig.h"
#include "ackermann_cmd_mux/ackermann_cmd_subscribers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ackermann_cmd_mux {

/*****************************************************************************
 ** AckermannCmdMux
 *****************************************************************************/

class AckermannCmdMuxNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

  AckermannCmdMuxNodelet()
  {
    dynamic_reconfigure_server = NULL;
  }

  ~AckermannCmdMuxNodelet()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

private:
  AckermannCmdSubscribers ackermann_cmd_sub; /**< Pool of ackermann_cmd topics subscribers */
  ros::Publisher mux_ackermann_cmd_pub; /**< Multiplexed ackermann command topic */
  ros::Publisher active_subscriber; /**< Currently allowed ackermann_cmd subscriber */

  void timerCallback(const ros::TimerEvent& event, unsigned int idx);
  void ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg, unsigned int idx);

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<ackermann_cmd_mux::reloadConfig> * dynamic_reconfigure_server;
  dynamic_reconfigure::Server<ackermann_cmd_mux::reloadConfig>::CallbackType dynamic_reconfigure_cb;
  void reloadConfiguration(ackermann_cmd_mux::reloadConfig &config, uint32_t unused_level);

  /*********************
   ** Private Classes
   **********************/
  // Functor assigned to each incoming ackermann command topic to bind it to ackermann_cmd callback
  class AckermannCmdFunctor
  {
  private:
    unsigned int idx;
    AckermannCmdMuxNodelet* node;

  public:
    AckermannCmdFunctor(unsigned int idx, AckermannCmdMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {
      node->ackermannCmdCallback(msg, idx);
    }
  };

  // Functor assigned to each ackermann command messages source to bind it to timer callback
  class TimerFunctor
  {
  private:
    unsigned int idx;
    AckermannCmdMuxNodelet* node;

  public:
    TimerFunctor(unsigned int idx, AckermannCmdMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const ros::TimerEvent& event)
    {
      node->timerCallback(event, idx);
    }
  };
};

} // namespace ackermann_cmd_mux

#endif /* RACECAR_ACKERMANN_CMD_MUX_HPP_ */
