#ifndef DATAHUB_DATA_HH
#define DATAHUB_DATA_HH

#include <memory>

namespace datahub {
struct Data {
  using Ptr = std::shared_ptr<Data>;
  virtual ~Data() {}
};
}  // namespace datahub

#endif  // DATA_HH
