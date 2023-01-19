#ifndef DATAHUB_DATA_HH
#define DATAHUB_DATA_HH

#include <memory>

namespace datahub {
struct Data {
  using Ptr = std::shared_ptr<Data>;
  virtual ~Data() {}
};

#define ADD_DATA_WRAPPER(DATA_TYPE, CLASS_NAME)               \
  struct CLASS_NAME##Wrapper : public datahub::Data {         \
    using Ptr = std::shared_ptr<CLASS_NAME##Wrapper>;         \
    DATA_TYPE var;                                            \
    CLASS_NAME##Wrapper(const DATA_TYPE& _var) : var(_var) {} \
  }

#define CREATE_DATA(VAR_NAME, DATA_NAME, INIT_VAR...) \
  std::shared_ptr<DATA_NAME##Wrapper> VAR_NAME =      \
      std::make_shared<DATA_NAME##Wrapper>(INIT_VAR)

#define DOWN_CAST_DATA(VAR_NAME, VAR_TYPE, DATA_NAME) \
  VAR_TYPE##Wrapper::Ptr VAR_NAME =                   \
      std::dynamic_pointer_cast<VAR_TYPE##Wrapper>(DATA_NAME)
}  // namespace datahub

#endif  // DATA_HH
