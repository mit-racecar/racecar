/*
 * Copyright (C) 2014, Massachusetts Institute of Technology
 * All rights reserved.
 *
 * This work is sponsored by the Department of the Air Force Air Force
 * contract number: FA8721-05-C-0002. The opinions, interpretations,
 * recommendations, and conclusions are those of the author and are
 * not necessarily endorsed by the United Stated Government.
 */

#ifndef PWM_SYSFS_DRIVER_PWM_SYSFS_DRIVER_H_
#define PWM_SYSFS_DRIVER_PWM_SYSFS_DRIVER_H_

#include <string>
#include <stdexcept>
#include <map>

#include <boost/shared_ptr.hpp>

namespace pwm_sysfs_driver
{

class PwmSysfsFile;

/** \brief Driver for sysfs-based pwm control.
 */
class PwmSysfsDriver
{
 public:
  /** \brief Constructs PwmSysfsDriver
   *
   *  \param sysfs_dir_path Path to sysfs pwm directory,
   *         e.g. /sys/class/pwm/pwmchip0
   *
   *  \throws PwmSysfsDriverException Failed to construct object, likely
   *          because it was unable to open sysfs files.
   */
  PwmSysfsDriver(std::string sysfs_dir_path);
  
  /// Returns current PWM enable state.
  bool enable();
  /// Set PWM enable state.
  void enable(bool enable);

  /// Polarity options
  enum PwmPolarity { PWM_POLARITY_NORMAL, PWM_POLARITY_INVERSED };

  /// Returns current PWM polarity state.
  PwmPolarity polarity();
  /// Set PWM polarity state.
  void polarity(PwmPolarity polarity);

  /// Returns current PWM period in nanoseconds.
  unsigned int period();
  /// Set PWM period in nanoseconds.
  void period(unsigned int period);

  /// Returns current PWM duty cycle fraction, 0 to 1.
  double duty_fraction();
  /// Set PWM duty cycle fraction, 0 to 1.
  void duty_fraction(double duty);

  /// Returns current PWM active time in nanoseconds.
  unsigned int duty_period();
  /// Set PWM active time in nanoseconds.
  void duty_period(unsigned int duty);

 private:
  boost::shared_ptr<PwmSysfsFile> enable_, polarity_, period_, duty_;
  std::map<PwmPolarity, std::string> polarity_to_string_;
  bool sw_invert_polarity_; // use sw to invery polarity
  unsigned int last_period_; // store value of last-set period
};

class PwmSysfsDriverException: public std::runtime_error
{
 public:
  PwmSysfsDriverException(std::string const& msg): std::runtime_error(msg) {}
};

} // namespace pwm_sysfs_driver

#endif // #ifndef PWM_SYSFS_DRIVER_PWM_SYSFS_DRIVER_H_
