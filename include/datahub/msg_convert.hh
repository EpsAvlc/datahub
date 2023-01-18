#ifndef DATAHUB_MSG_CONVERT_HH__
#define DATAHUB_MSG_CONVERT_HH__

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "datahub/message.hh"

namespace datahub {
Header fromROS(const std_msgs::Header& header) {
  Header ret;
  ret.frame_id = header.frame_id;
  ret.timestamp = header.stamp.toNSec() / 1000.;
  return ret;
};

std_msgs::Header toROS(const Header& header) {
  std_msgs::Header ret;
  ret.frame_id = header.frame_id;
  ret.stamp.fromNSec(header.timestamp.usec() * 1000);
  return ret;
};
}  // namespace datahub

#endif  // MSG_CONVERT_HH
