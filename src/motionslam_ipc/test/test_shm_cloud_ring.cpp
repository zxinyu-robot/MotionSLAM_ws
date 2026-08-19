#include "motionslam_ipc/shm_cloud_ring.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

namespace {

using motionslam_ipc::CloudFrame;
using motionslam_ipc::PointXYZI;
using motionslam_ipc::ReadResult;
using motionslam_ipc::ShmCloudRingReader;
using motionslam_ipc::ShmCloudRingWriter;
using motionslam_ipc::UnlinkShmCloudRing;
using motionslam_ipc::WriterConfig;

class ShmCloudRingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    name_ = "/motionslam_ipc_test_" + std::to_string(::getpid()) + "_" +
            std::to_string(counter_.fetch_add(1));
    UnlinkShmCloudRing(name_);
  }

  void TearDown() override { UnlinkShmCloudRing(name_); }

  std::string name_;
  static std::atomic<unsigned> counter_;
};

std::atomic<unsigned> ShmCloudRingTest::counter_{0};

TEST_F(ShmCloudRingTest, LatestFrameOverwritesFixedSlots) {
  WriterConfig config;
  config.slot_count = 2;
  config.max_points_per_slot = 4;
  config.session_id = 91;
  auto writer = ShmCloudRingWriter::Create(name_, config);
  auto reader = ShmCloudRingReader::Open(name_);

  for (std::uint64_t sequence = 1; sequence <= 7; ++sequence) {
    const PointXYZI point{static_cast<float>(sequence), 2.0F, 3.0F, 4.0F};
    EXPECT_EQ(writer.Publish(&point, 1, sequence * 100, 12), sequence);
  }

  CloudFrame frame;
  ASSERT_EQ(reader.ReadLatest(frame), ReadResult::kSuccess);
  EXPECT_EQ(frame.sequence, 7U);
  EXPECT_EQ(frame.session_id, 91U);
  EXPECT_EQ(frame.tile_id, 12U);
  EXPECT_EQ(frame.timestamp_ns, 700);
  ASSERT_EQ(frame.points.size(), 1U);
  EXPECT_FLOAT_EQ(frame.points.front().x, 7.0F);
}

TEST_F(ShmCloudRingTest, GenerationIncrementsWhenProducerRestarts) {
  WriterConfig config;
  config.slot_count = 2;
  config.max_points_per_slot = 4;

  {
    auto first = ShmCloudRingWriter::Create(name_, config);
    EXPECT_EQ(first.generation(), 1U);
  }
  {
    auto second = ShmCloudRingWriter::Create(name_, config);
    EXPECT_EQ(second.generation(), 2U);
    auto reader = ShmCloudRingReader::Open(name_);
    EXPECT_EQ(reader.generation(), 2U);
    CloudFrame frame;
    EXPECT_EQ(reader.ReadLatest(frame), ReadResult::kNoData);
  }
}

TEST_F(ShmCloudRingTest, ConcurrentReaderNeverAcceptsTornCloud) {
  WriterConfig config;
  config.slot_count = 3;
  config.max_points_per_slot = 64;
  auto writer = ShmCloudRingWriter::Create(name_, config);
  auto reader = ShmCloudRingReader::Open(name_);
  std::atomic<bool> done{false};
  std::atomic<unsigned> successes{0};
  std::atomic<unsigned> failures{0};

  std::thread consumer([&] {
    CloudFrame frame;
    while (!done.load(std::memory_order_acquire) ||
           reader.latest_sequence() < 10000) {
      if (reader.ReadLatest(frame, 10) != ReadResult::kSuccess) {
        continue;
      }
      ++successes;
      for (const auto& point : frame.points) {
        if (point.x != static_cast<float>(frame.sequence) ||
            point.y != static_cast<float>(frame.sequence) ||
            point.z != static_cast<float>(frame.sequence) ||
            point.intensity != static_cast<float>(frame.sequence)) {
          ++failures;
          break;
        }
      }
    }
  });

  std::vector<PointXYZI> points(64);
  for (std::uint64_t sequence = 1; sequence <= 10000; ++sequence) {
    for (auto& point : points) {
      const float value = static_cast<float>(sequence);
      point = PointXYZI{value, value, value, value};
    }
    writer.Publish(points, static_cast<std::int64_t>(sequence));
  }
  done.store(true, std::memory_order_release);
  consumer.join();

  EXPECT_GT(successes.load(), 0U);
  EXPECT_EQ(failures.load(), 0U);
}

TEST_F(ShmCloudRingTest, RejectsBadHeader) {
  const int fd = ::shm_open(name_.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
  ASSERT_GE(fd, 0);
  ASSERT_EQ(::ftruncate(fd, sizeof(motionslam_ipc::RingHeader)), 0);
  void* memory = ::mmap(nullptr, sizeof(motionslam_ipc::RingHeader),
                        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  ASSERT_NE(memory, MAP_FAILED);
  std::memset(memory, 0xA5, sizeof(motionslam_ipc::RingHeader));
  ASSERT_EQ(::munmap(memory, sizeof(motionslam_ipc::RingHeader)), 0);
  ASSERT_EQ(::close(fd), 0);

  EXPECT_THROW(ShmCloudRingReader::Open(name_), std::runtime_error);
}

TEST_F(ShmCloudRingTest, OptionalEventFdWakesReader) {
  const int event_fd = motionslam_ipc::CreateEventFd();
  WriterConfig config;
  config.slot_count = 2;
  config.max_points_per_slot = 4;
  config.notification_fd = event_fd;
  auto writer = ShmCloudRingWriter::Create(name_, config);
  auto reader = ShmCloudRingReader::Open(name_, event_fd);

  const PointXYZI point{1.0F, 2.0F, 3.0F, 4.0F};
  writer.Publish(&point, 1, 123);
  EXPECT_TRUE(reader.WaitForNotification(100));
  EXPECT_FALSE(reader.WaitForNotification(0));
  EXPECT_EQ(::close(event_fd), 0);
}

}  // namespace
