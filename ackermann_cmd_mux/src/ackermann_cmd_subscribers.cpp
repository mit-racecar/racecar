/**
 * @file /src/ackermann_cmd_subscribers.cpp
 *
 * @brief  Subscriber handlers for the ackermann_cmd_mux
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_ackermann_cmd_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>

#include "ackermann_cmd_mux/ackermann_cmd_subscribers.hpp"
#include "ackermann_cmd_mux/exceptions.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ackermann_cmd_mux {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void AckermannCmdSubscribers::AckermannCmdSubs::operator << (const YAML::Node& node)
{
  node["name"]       >> name;
  node["topic"]      >> topic;
  node["timeout"]    >> timeout;
  node["priority"]   >> priority;
#ifdef HAVE_NEW_YAMLCPP
  if (node["short_desc"]) {
#else
  if (node.FindValue("short_desc") != NULL) {
#endif
    node["short_desc"] >> short_desc;
  }
}

void AckermannCmdSubscribers::configure(const YAML::Node& node) {

  list.clear();
  try
  {
    if ( node.size() == 0 ) {
      throw EmptyCfgException();
    }

    for (unsigned int i = 0; i < node.size(); i++)
    {
      // Parse every entries on YAML
      AckermannCmdSubs subscriber(i);
      subscriber << node[i];
      list.push_back(subscriber);
    }
  }
  catch(EmptyCfgException& e) {
    throw e;
  }
  catch(YAML::ParserException& e)
  {
    throw YamlException(e.what());
  }
  catch(YAML::RepresentationException& e)
  {
    throw YamlException(e.what());
  }
}


} // namespace ackermann_cmd_mux
