/*
 * Copyright (C) 2014, Massachusetts Institute of Technology
 * All rights reserved.
 *
 * This work is sponsored by the Department of the Air Force Air Force
 * contract number: FA8721-05-C-0002. The opinions, interpretations,
 * recommendations, and conclusions are those of the author and are
 * not necessarily endorsed by the United Stated Government.
 */

/**
 * For Linux kernel sysfs and pwm documentation, see:
 *   https://www.kernel.org/doc/Documentation/filesystems/sysfs.txt
 *   https://www.kernel.org/doc/Documentation/pwm.txt
 *   http://lxr.free-electrons.com/source/drivers/pwm/sysfs.c
 */

#include "pwm_sysfs_driver/pwm_sysfs_driver.h"

// using low-level file IO to control buffering
#include "unistd.h"   // for read(), write(), lseek()
#include "sys/types.h" // for SEEK_SET
#include "fcntl.h"    // for open()

#include <cassert>    // for assert()
#include <cerrno>     // for errno
#include <cstring>    // for strerror()
#include <locale>     // for isprint()
#include <algorithm>  // for find(), copy()
#include <iterator>   // for distance()
#include <functional> // for not1(), ptr_fun()
#include <sstream>    // for ostringstream, istringstream
#include <string>     // for string, stoul()
#ifdef PWM_SYSFS_DRIVER_DEBUG
  #include <iostream> // for cout
#endif

#include <boost/algorithm/string.hpp> // for trim()
#include <boost/array.hpp> // for array

namespace pwm_sysfs_driver
{

/// Small, simple file buffer tailored for PWM Sysfs files
class PwmSysfsFile
{
public:
  /// Open PWM sysfs file at dir/path
  /// @throws PwmSysfsDriverException on error
  PwmSysfsFile(std::string const& dir, std::string const& file);
  ~PwmSysfsFile();

  /// Get newline-terminated printable string at start of file
  /// @throws PwmSysfsDriverException on error
  void operator>>(std::string& output);

  /// Returns true and extracts unsigned int if buffer is convertable to a
  /// base-10 unsigned integer followed by "\n"
  /// @throws PwmSysfsDriverException on error
  void operator>>(unsigned int& output);

  /// Write string followed by newline
  /// @throws PwmSysfsDriverException on error
  void operator<<(std::string const& input);

  /// Write unsigned integer followed by newline
  /// @throws PwmSysfsDriverException on error
  void operator<<(unsigned int const& input);

 private:
  std::string dir_;
  std::string file_;
  int fd_;

  /// Return newline-terminated printable string at start of file
  /// @throws PwmSysfsDriverException on error
  void readNewlineTerminatedPrintableString(std::string& output);
};

PwmSysfsDriver::PwmSysfsDriver(std::string sysfs_dir_path) :
  sw_invert_polarity_(false)
{
  boost::trim(sysfs_dir_path);

#ifdef PWM_SYSFS_DRIVER_DEBUG
  std::cout << "Initializing PwmSysfsDriver for SYSFS path '"
            << sysfs_dir_path << "'." << std::endl;
#endif

  // map polarity values to expected strings
  polarity_to_string_[PWM_POLARITY_NORMAL] = std::string("normal");
  polarity_to_string_[PWM_POLARITY_INVERSED] = std::string("inversed");

  // open sysfs files
  enable_.reset(new PwmSysfsFile(sysfs_dir_path, "enable"));
  polarity_.reset(new PwmSysfsFile(sysfs_dir_path, "polarity"));
  period_.reset(new PwmSysfsFile(sysfs_dir_path, "period"));
  duty_.reset(new PwmSysfsFile(sysfs_dir_path, "duty"));

  // save period internally
  last_period_ = period();
}

bool PwmSysfsDriver::enable()
{
  assert(enable_);

  // read from enable file, expect "0" or "1"
  std::string enable;
  *enable_ >> enable;

  if (enable.compare("0") == 0)
    return false;
  else if (enable.compare("1") == 0)
    return true;
  else {
    std::ostringstream e;
    e << "unexpected content in PWM enable file: '" << enable << "'.";
    throw PwmSysfsDriverException(e.str());
  }
}

void PwmSysfsDriver::enable(bool enable)
{
  assert(enable_);

  if (enable)
    *enable_ << std::string("1");
  else
    *enable_ << std::string("0");

#ifdef PWM_SYSFS_DRIVER_WRITE_AND_VERIFY
  assert(this->enable() == enable);
#endif
#ifdef PWM_SYSFS_DRIVER_DEBUG
  std::cout << "Set PWM enable to " << enable << std::endl;
#endif
}

PwmSysfsDriver::PwmPolarity PwmSysfsDriver::polarity()
{
  assert(polarity_);

  // read from polarity file, expect "normal" or "inversed"
  std::string polarity;
  *polarity_ >> polarity;

  if (polarity.compare(polarity_to_string_[PWM_POLARITY_NORMAL]) == 0 &&
      sw_invert_polarity_ == false)
    return PWM_POLARITY_NORMAL;
  else if (polarity.compare(polarity_to_string_[PWM_POLARITY_INVERSED]) == 0 ||
           (polarity.compare(polarity_to_string_[PWM_POLARITY_NORMAL]) == 0 &&
            sw_invert_polarity_ == true))
    return PWM_POLARITY_INVERSED;
  else  {
    std::ostringstream e;
    e << "unexpected content in PWM polarity file: '" << polarity << "'.";
    throw PwmSysfsDriverException(e.str());
  }
}

void PwmSysfsDriver::polarity(PwmSysfsDriver::PwmPolarity polarity)
{
  assert(polarity_);
  assert(polarity == PWM_POLARITY_NORMAL ||
         polarity == PWM_POLARITY_INVERSED);

  // some pwm hardware does not support hw polarity control
  sw_invert_polarity_ = false;
  try {
    *polarity_ << polarity_to_string_[polarity];
  }
  catch (PwmSysfsDriverException e) {
    // write to polarity file failed, assume no hardware support; so we must
    // invert using software
    if (polarity == PWM_POLARITY_INVERSED)
      sw_invert_polarity_ = true;
  }

#ifdef PWM_SYSFS_DRIVER_WRITE_AND_VERIFY
  assert(this->polarity() == polarity ||
         (sw_invert_polarity_ && polarity == PWM_POLARITY_INVERSED));
#endif
#ifdef PWM_SYSFS_DRIVER_DEBUG
  std::cout << "Set PWM polarity to '" << polarity_to_string_[polarity]
            << "'" << std::endl;
#endif
}

unsigned int PwmSysfsDriver::period()
{
  assert(period_);

  // read from period file, expect unsigned integer
  unsigned int period;
  *period_ >> period;

  // store value of period internally
  last_period_ = period;

  return period;
}

void PwmSysfsDriver::period(unsigned int period)
{
  assert(period_);

  *period_ << period;

  // store value of period internally
  last_period_ = period;

#ifdef PWM_SYSFS_DRIVER_WRITE_AND_VERIFY
  assert(this->period() == period);
#endif
#ifdef PWM_SYSFS_DRIVER_DEBUG
  std::cout << "Set PWM period to " << period << " nanoseconds." << std::endl;
#endif
}

double PwmSysfsDriver::duty_fraction()
{
  return (double)duty_period() / last_period_;
}

void PwmSysfsDriver::duty_fraction(double duty)
{
  duty_period(duty * last_period_);
}

unsigned int PwmSysfsDriver::duty_period()
{
  assert(duty_);

  // read from duty file, expect unsigned integer
  unsigned int duty;
  *duty_ >> duty;

  return duty;
}

void PwmSysfsDriver::duty_period(unsigned int duty)
{
  assert(duty_);

  unsigned int sw_duty(duty);
  if (sw_invert_polarity_) {
    sw_duty = last_period_ - duty;
  }

  *duty_ << sw_duty;

#ifdef PWM_SYSFS_DRIVER_WRITE_AND_VERIFY
  assert(this->duty_period() == sw_duty);
#endif
#ifdef PWM_SYSFS_DRIVER_DEBUG
  std::cout << "Set PWM duty cycle active time to " << sw_duty << " nanoseconds." << std::endl;
#endif
}

PwmSysfsFile::PwmSysfsFile(std::string const& dir, std::string const& file) :
  dir_(dir), file_(file), fd_(-1)
{
  std::string full_path(dir_ + "/" + file_);
  fd_ = open(full_path.c_str(), O_RDWR);
  if (fd_ < 0) {
    int errnum(errno);
    std::ostringstream e;
    e << "unable to open PWM " << file_ << " file at '" << full_path
      << "', error is: " << std::strerror(errnum) << ".";
    throw PwmSysfsDriverException(e.str());
  }
}

PwmSysfsFile::~PwmSysfsFile()
{
  assert(fd_ >= 0);
  close(fd_);
  fd_ = -1;
}

void PwmSysfsFile::operator>>(std::string& output)
{
  readNewlineTerminatedPrintableString(output);
}

void PwmSysfsFile::operator>>(unsigned int& output)
{
  std::string str;
  readNewlineTerminatedPrintableString(str);

  // attempt conversion to unsigned int
  unsigned int val;
  std::istringstream iss(str);
  if (iss >> val) {
    // expect end-of-file if the unsigned long was immediately followed by "\n"
    if (iss.eof()) {
      output = val;
    }
    else {
      std::ostringstream e;
      e << "expected to read newline-terminated unsigned integer from PWM "
        << file_ << " file, string '" << str << "' was not followed by a "
        << "newline character.";
      throw PwmSysfsDriverException(e.str());
    }
  }
  else {
      std::ostringstream e;
      e << "expected to read newline-terminated unsigned integer from PWM "
        << file_ << " file, unable to convert '" << str << "' string to "
        << "an unsigned integer.";
      throw PwmSysfsDriverException(e.str());
  }
}

void PwmSysfsFile::operator<<(std::string const& input)
{
  std::ostringstream oss;
  oss << input << "\n";

  int count = write(fd_, oss.str().c_str(), oss.str().size());

  if (count != oss.str().size()) {
    int errnum(errno);
    std::ostringstream e;
    e << "expected to write " << oss.str().size() << " bytes to PWM " << file_
      << " file, actually wrote " << count << " bytes, error is: "
      << std::strerror(errnum) << ".";
    throw PwmSysfsDriverException(e.str());
  }
}

void PwmSysfsFile::operator<<(unsigned int const& input)
{
  std::ostringstream oss;
  oss << input;
  *this << oss.str();
}

void PwmSysfsFile::readNewlineTerminatedPrintableString(std::string& output)
{
  assert(fd_ >= 0);

  typedef boost::array<char, 64> buffer_t;
  buffer_t buffer;

  int offset = lseek(fd_, 0, SEEK_SET);
  if (offset < 0) {
    int errnum(errno);
    std::ostringstream e;
    e << "unable to seek to start of PWM " << file_ << " file, error is: "
      << std::strerror(errnum) << ".";
    throw PwmSysfsDriverException(e.str());
  }

  int count = read(fd_, buffer.c_array(), buffer.size());
  if (count < 0) {
    int errnum(errno);
    std::ostringstream e;
    e << "unable to read PWM " << file_ << " file, error is: "
      << std::strerror(errnum) << ".";
    throw PwmSysfsDriverException(e.str());
  }

  // need iterator to end of read
  buffer_t::iterator buffer_read_end(buffer.begin() + count);

  // note that C++11 std::copy_if would be useful below

  // find first non-printable character
  buffer_t::iterator it;
  it = std::find_if(buffer.begin(), buffer_read_end,
                    std::not1(std::ptr_fun(static_cast<int(*)(int)>(std::isprint))));

  // if first non-printable character is a newline, copy string to output
  if (it != buffer_read_end) {
    if (*it == '\n') {
      output.resize(std::distance(buffer.begin(), it));
      std::copy(buffer.begin(), it, output.begin());
    }
    else {
      std::ostringstream e;
      e << "expected to read newline-terminated string from PWM " << file_
        << " file, read " << count << " bytes, first non-printing character is "
        << std::hex << static_cast<int>(*it);
      throw PwmSysfsDriverException(e.str());
    }
  }
  else {
    std::ostringstream e;
    e << "expected to read newline-terminated string from PWM " << file_
      << " file, read " << count << " bytes, no non-printing character found.";
    throw PwmSysfsDriverException(e.str());
  }
}

} // namespace pwm_sysfs_driver
