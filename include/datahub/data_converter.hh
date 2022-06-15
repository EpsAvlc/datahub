#ifndef DATAHUB_DATA_CONVERTER_HH__
#define DATAHUB_DATA_CONVERTER_HH__

#include <memory>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "datahub/message.hh"

namespace datahub {
void toDatahubHeader(const std_msgs::Header &ros_header,
                   datahub::Header *std_header);

std::shared_ptr<datahub::Message> toNalioMessage(
    const sensor_msgs::ImuConstPtr &imu_msg, const std::string &name);

std::shared_ptr<datahub::Message> toNalioMessage(
    const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg,
    const std::string &name);
}  // namespace datahub


#endif  // NALIO_WS_DATA_DATA_CONVERTER_HH__
