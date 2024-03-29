#ifndef DATAHUB_MESSAGE_HH__
#define DATAHUB_MESSAGE_HH__

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "datahub/data.hh"

namespace datahub {
class TimeStamp {
 public:
  TimeStamp() {}

  double sec() const { return us_ / 1000000.; }

  int64_t usec() const { return us_; }

  bool operator<(const TimeStamp& rhs) { return us_ < rhs.us_; }

  bool operator>(const TimeStamp& rhs) { return us_ > rhs.us_; }

  void fromSec(const double s) { us_ = s * 1e6; }

  void fromUSec(const int64_t us) { us_ = us; }

 private:
  // time stamp in micro seconds
  int64_t us_;
};

struct Header {
  TimeStamp timestamp;
  std::string frame_id;
};

struct Message {
  using Ptr = std::shared_ptr<Message>;
  Header header;
  std::string name;
  std::shared_ptr<Data> data;
  struct Type {
    uint8_t val;
    enum : uint8_t { kImu, kLidar };
  } type;
};

using MessagePackage = std::vector<std::vector<Message::Ptr>>;

}  // namespace datahub

#endif  // NALIO_WS_DATA_MESSAGE_HH__
