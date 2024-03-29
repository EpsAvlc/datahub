#ifndef NALIO_DATA_DATAHUB_HH__
#define NALIO_DATA_DATAHUB_HH__

#include <condition_variable>
#include <list>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "datahub/message.hh"
#include "datahub/read_write_mutex.hh"

namespace datahub {

struct DatahubConfig {
  bool print_log = false;
};

using DatahubCallback =
    std::function<void(const std::vector<std::vector<Message::Ptr>>&)>;
class DataBuffer;

class DataSyncer {
  friend class DataBuffer;

 public:
  using Ptr = std::shared_ptr<DataSyncer>;
  struct SyncType {
    uint8_t val;
    enum : uint8_t { kNearest = 0, kBilateral, kMultiple };
    SyncType() {}
    SyncType(const uint8_t type) : val(type) {}
  };

  DataSyncer(const std::vector<std::string>& message_names,
             const std::vector<SyncType> sync_types, DataBuffer* buffer);

  ~DataSyncer();

  bool getSyncMessages(std::vector<std::vector<Message::Ptr>>* synced_messages,
                       const double t_thres = 0.2);

  bool getMessageId(const std::string& message_name, uint16_t* message_id);

  void registerCallback(const DatahubCallback& cb);

  void syncThread();

 private:
  DataSyncer(){};
  struct MessageInfo {
    uint16_t id;
    SyncType sync_type;
    std::string name;
  };

  std::vector<DatahubCallback> callbacks_;

  DataBuffer* buffer_;
  std::map<std::string, uint16_t> syncer_message_ids_;
  std::vector<uint16_t> buffer_message_ids_;
  std::vector<MessageInfo> message_infos_;
  std::vector<std::list<Message::Ptr>::iterator> message_iters_;
  std::thread sync_thread_;
  std::mutex sync_mutex_;
  std::condition_variable sync_cv_;
  bool running_;
};

class DataBuffer {
  friend class DataSyncer;

 public:
  using IteratorType = std::list<Message::Ptr>::iterator;
  using Ptr = std::shared_ptr<DataBuffer>;
  DataBuffer(const DatahubConfig& config = DatahubConfig());
  ~DataBuffer();

  DataSyncer::Ptr createDataSyncer(
      const std::vector<std::string>& message_names,
      const std::vector<DataSyncer::SyncType> sync_types);

  void registerMessage(const std::string& message_name,
                       const size_t buffer_size);

  void receiveMessage(const Message::Ptr& message);

  bool getMessageId(const std::string& message_name, uint16_t* message_id);

  IteratorType messageBuffEnd(const uint16_t id);

  void pruneBuffs();

  IteratorType getMessagesBetween(const IteratorType& iter_begin,
                                  const IteratorType& iter_end,
                                  const int64_t tl, const int64_t tr,
                                  std::vector<Message::Ptr>* synced_msgs);

  IteratorType getMessagesBefore(const IteratorType& iter_begin,
                                 const IteratorType& iter_end, const int64_t tr,
                                 std::vector<Message::Ptr>* synced_msgs);

 private:
  const DatahubConfig& config() const { return config_; }

  struct MessageInfo {
    uint16_t id;
    std::string name;
    uint16_t buff_size;
  };

  std::map<std::string, uint16_t> message_ids_;
  std::vector<MessageInfo> message_infos_;
  std::vector<std::list<Message::Ptr>> message_buffs_;
  std::vector<DataSyncer::Ptr> data_syncers_;
  std::thread prune_buffs_thread_;
  ReadWriteMutex mutex_;
  DatahubConfig config_;
  bool running_;
};

}  // namespace datahub

#endif  // NALIO_WS_DATA_DATAHUB_HH__
