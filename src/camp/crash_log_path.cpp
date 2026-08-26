#include "crash_log_path.h"

#include <unistd.h>

#include <exception>
#include <string>

#include "rclcpp/logger.hpp"

namespace camp_crash
{

std::string crash_log_path()
{
  std::string dir;
  try
  {
    // Throws rclcpp::exceptions::RCLError if the directory cannot be
    // resolved. Caught here so a bad log dir degrades to stderr-only rather
    // than terminating CAMP before it has a window — a diagnostics feature
    // must not become a startup failure on the hosts it serves.
    dir = rclcpp::get_logging_directory().string();
  }
  catch (const std::exception&)
  {
    return std::string();
  }

  if (dir.empty())
    return std::string();

  return dir + "/camp_crash_" + std::to_string(::getpid()) + ".log";
}

} // namespace camp_crash
