#ifndef DATAHUB_LOG_UTILS_HH__
#define DATAHUB_LOG_UTILS_HH__

#include <ros/console.h>
#include <string>

namespace datahub {
std::string cutParenthesesNTail(std::string&& pretty_func);

}

#define __STR_FUNCTION__ \
  datahub::cutParenthesesNTail(std::string(__PRETTY_FUNCTION__)).c_str()
#define ROS_ERROR_STREAM_FUNC(log) \
  ROS_ERROR_STREAM(__STR_FUNCTION__ << ": " << log)
#define ROS_INFO_STREAM_FUNC(log) \
  ROS_INFO_STREAM(__STR_FUNCTION__ << ": " << log)

#endif  // DATAHUB_LOG_UTILS_HH__
