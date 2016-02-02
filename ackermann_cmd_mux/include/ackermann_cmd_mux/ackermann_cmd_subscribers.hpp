/**
 * @file /include/ackermann_cmd_mux/ackermann_cmd_subscribers.hpp
 *
 * @brief Structure for the ackermann_cmd_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_ackermann_cmd_mux/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef RACECAR_ACKERMANN_CMD_SUBSCRIBERS_HPP_
#define RACECAR_ACKERMANN_CMD_SUBSCRIBERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

/*****************************************************************************
** Preprocessing
*****************************************************************************/

// move to a static const?
#define VACANT  std::numeric_limits<unsigned int>::max()

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ackermann_cmd_mux {

/*****************************************************************************
** AckermannCmdSubscribers
*****************************************************************************/

/**
 * Pool of ackermann_cmd topics subscribers
 */
class AckermannCmdSubscribers
{
public:

  /**
   * Inner class describing an individual subscriber to a ackermann_cmd topic
   */
  class AckermannCmdSubs
  {
  public:
    unsigned int           idx;          /**< Index; assigned according to the order on YAML file */
    std::string            name;         /**< Descriptive name */
    ros::Subscriber        subs;         /**< The subscriber itself */
    std::string            topic;        /**< The name of the topic */
    ros::Timer             timer;        /**< No incoming messages timeout */
    double                 timeout;      /**< Timer's timeout, in seconds  */
    unsigned int           priority;     /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
    std::string            short_desc;   /**< Short description (optional) */
    bool                   active;       /**< Whether this source is active */

    AckermannCmdSubs(unsigned int idx) : idx(idx), active(false) {};

    void operator << (const YAML::Node& node);
  };

  AckermannCmdSubscribers() : allowed(VACANT) { }
  ~AckermannCmdSubscribers() { }

  std::vector<AckermannCmdSubs>::size_type size() { return list.size(); };
  AckermannCmdSubs& operator [] (unsigned int idx) { return list[idx]; };

  /**
   * @brief Configures the subscribers from a yaml file.
   *
   * @exception FileNotFoundException : yaml file not found
   * @exception YamlException : problem parsing the yaml
   * @exception EmptyCfgException : empty configuration file
   * @param node : node holding all the subscriber configuration
   */
  void configure(const YAML::Node& node);

  unsigned int allowed;

private:
  std::vector<AckermannCmdSubs> list;
};

} // namespace ackermann_cmd_mux

#endif /* ACKERMANN_CMD_SUBSCRIBERS_HPP_ */
