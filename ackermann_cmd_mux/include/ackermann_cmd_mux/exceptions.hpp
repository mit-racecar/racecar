/**
 * @file /ackermann_cmd_mux/include/ackermann_cmd_mux/exceptions.hpp
 *
 * @brief Exception classes for ackermann_cmd_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_ackermann_cmd_mux/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef RACECAR_ACKERMANN_CMD_EXCEPTIONS_HPP_
#define RACECAR_ACKERMANN_CMD_EXCEPTIONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <exception>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ackermann_cmd_mux {

/*****************************************************************************
** Exceptions
*****************************************************************************/

class FileNotFoundException: public std::runtime_error {
public:
  FileNotFoundException(const std::string& msg)
        : std::runtime_error(msg) {}
        virtual ~FileNotFoundException() throw() {}
};

class EmptyCfgException: public std::runtime_error {
public:
  EmptyCfgException()
        : std::runtime_error("") {}
        virtual ~EmptyCfgException() throw() {}
};

class YamlException: public std::runtime_error {
public:
  YamlException(const std::string& msg)
        : std::runtime_error(msg) {}
        virtual ~YamlException() throw() {}
};

} // namespace ackermann_cmd_mux

#endif /* RACECAR_ACKERMANN_CMD_EXCEPTIONS_HPP_ */
