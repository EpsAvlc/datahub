#include <gtest/gtest.h>
#include <chrono>
#include <thread>

#include "datahub/datahub.hh"

class DatahubTest : public ::testing::Test {
 protected:
  void SetUp() override {
    running_ = true;
    buffer_.registerMessage("/sensor/IMU", 400);
    buffer_.registerMessage("/sensor/Image", 40);
    buffer_.registerMessage("/sensor/LiDAR", 20);
    buffer_.registerMessage("/egopose", 400);

    threads_[0] = std::thread(&DatahubTest::msgLoop, this, "/sensor/IMU", 200);
    threads_[1] = std::thread(&DatahubTest::msgLoop, this, "/sensor/Image", 20);
    threads_[2] = std::thread(&DatahubTest::msgLoop, this, "/sensor/LiDAR", 10);
    threads_[3] = std::thread(&DatahubTest::msgLoop, this, "/egopose", 200);
  }

  void TearDown() override {
    running_ = false;
    for (size_t ti = 0; ti < 4; ++ti) {
      if (threads_[ti].joinable()) {
        threads_[ti].join();
      }
    }
  }

  void msgLoop(const std::string& msg_name, double freq) {
    uint duration = 1000. / freq;
    while (running_) {
      std::chrono::time_point<std::chrono::steady_clock> t_n =
          std::chrono::steady_clock::now();
      datahub::Message::Ptr msg(new datahub::Message);
      msg->name = msg_name;
      msg->header.timestamp =
          std::chrono::time_point_cast<std::chrono::microseconds>(t_n)
              .time_since_epoch()
              .count();
      buffer_.receiveMessage(msg);
      std::chrono::time_point<std::chrono::steady_clock> t_e =
          t_n + std::chrono::milliseconds(duration);
      std::this_thread::sleep_until(t_e);
    }
  }

  std::chrono::_V2::steady_clock::time_point awakeTime(uint16_t m) {
    return std::chrono::steady_clock::now() + std::chrono::milliseconds(m);
  }

  datahub::DataBuffer buffer_;
  bool running_;
  std::thread threads_[4];
};

TEST_F(DatahubTest, Nearest) {
  std::vector<std::string> msg_names = {"/sensor/LiDAR", "/sensor/Image"};
  std::vector<datahub::DataSyncer::SyncType> msg_sync_types = {
      datahub::DataSyncer::SyncType::kNearest,
      datahub::DataSyncer::SyncType::kNearest};
  datahub::DataSyncer::Ptr lidar_img_syncer =
      buffer_.createDataSyncer(msg_names, msg_sync_types);
  std::vector<std::vector<datahub::Message::Ptr>> synced_msgs;
  std::vector<int64_t> lidar_times;
  std::vector<int64_t> image_times;
  for (uint i = 0; i < 10; ++i) {
    while (!lidar_img_syncer->getSyncMessages(&synced_msgs)) {
      std::this_thread::yield();
    }
    lidar_times.push_back(synced_msgs[0][0]->header.timestamp.usec());
    image_times.push_back(synced_msgs[0][0]->header.timestamp.usec());

    std::cout << "[----------] lidar time: "
              << synced_msgs[0][0]->header.timestamp.usec()
              << ", image time:" << synced_msgs[1][0]->header.timestamp.usec()
              << std::endl;
  }
  for (int i = 0; i < lidar_times.size() - 1; ++i) {
    EXPECT_TRUE(lidar_times[i + 1] - lidar_times[i] < 0.15 * 1000000);
  }

  for (int i = 0; i < lidar_times.size(); ++i) {
    EXPECT_TRUE(std::abs(lidar_times[i] - image_times[i]) < 0.1 * 1000000);
  }
}

TEST_F(DatahubTest, Bilateral) {
  std::vector<std::string> msg_names = {"/sensor/LiDAR", "/egopose"};
  std::vector<datahub::DataSyncer::SyncType> msg_sync_types = {
      datahub::DataSyncer::SyncType::kNearest,
      datahub::DataSyncer::SyncType::kBilateral};
  datahub::DataSyncer::Ptr lidar_img_syncer =
      buffer_.createDataSyncer(msg_names, msg_sync_types);
  std::vector<std::vector<datahub::Message::Ptr>> synced_msgs;
  std::vector<int64_t> pivot_timestamps;
  for (uint i = 0; i < 10; ++i) {
    while (!lidar_img_syncer->getSyncMessages(&synced_msgs)) {
      std::this_thread::yield();
    }
    pivot_timestamps.push_back(synced_msgs[0][0]->header.timestamp.usec());
    std::cout << "[----------] lidar time: "
              << synced_msgs[0][0]->header.timestamp.usec()
              << ", ego time:" << synced_msgs[1][0]->header.timestamp.usec()
              << ", " << synced_msgs[1][1]->header.timestamp.usec()
              << std::endl;
    EXPECT_TRUE(synced_msgs[1][0]->header.timestamp.usec() <
                    synced_msgs[0][0]->header.timestamp.usec() &&
                synced_msgs[1][1]->header.timestamp.usec() >
                    synced_msgs[0][0]->header.timestamp.usec());
  }
}

TEST_F(DatahubTest, Multiple) {
  std::vector<std::string> msg_names = {"/sensor/LiDAR", "/sensor/IMU"};
  std::vector<datahub::DataSyncer::SyncType> msg_sync_types = {
      datahub::DataSyncer::SyncType::kNearest,
      datahub::DataSyncer::SyncType::kMultiple};
  datahub::DataSyncer::Ptr lidar_img_syncer =
      buffer_.createDataSyncer(msg_names, msg_sync_types);
  std::vector<std::vector<datahub::Message::Ptr>> synced_msgs;
  std::vector<int64_t> pivot_timestamps;
  for (uint i = 0; i < 10; ++i) {
    while (!lidar_img_syncer->getSyncMessages(&synced_msgs)) {
      std::this_thread::yield();
    }
    pivot_timestamps.push_back(synced_msgs[0][0]->header.timestamp.usec());
    std::cout << "[----------] lidar time: "
              << synced_msgs[0][0]->header.timestamp.usec()
              << ", imu size:" << synced_msgs[1].size() << std::endl;
  }
}

TEST_F(DatahubTest, LVIO) {
  std::vector<std::string> msg_names = {"/sensor/LiDAR", "/sensor/Image",
                                        "/sensor/IMU"};
  std::vector<datahub::DataSyncer::SyncType> msg_sync_types = {
      datahub::DataSyncer::SyncType::kNearest,
      datahub::DataSyncer::SyncType::kNearest,
      datahub::DataSyncer::SyncType::kMultiple};
  datahub::DataSyncer::Ptr lidar_img_syncer =
      buffer_.createDataSyncer(msg_names, msg_sync_types);
  std::vector<std::vector<datahub::Message::Ptr>> synced_msgs;
  std::vector<int64_t> pivot_timestamps;
  for (uint i = 0; i < 10; ++i) {
    while (!lidar_img_syncer->getSyncMessages(&synced_msgs)) {
      std::this_thread::yield();
    }
    pivot_timestamps.push_back(synced_msgs[0][0]->header.timestamp.usec());
    std::cout << "[----------] lidar time: "
              << synced_msgs[0][0]->header.timestamp.usec()
              << ", Image time: " << synced_msgs[1][0]->header.timestamp.usec()
              << ", imu size:" << synced_msgs[2].size() << std::endl;
  }
}
