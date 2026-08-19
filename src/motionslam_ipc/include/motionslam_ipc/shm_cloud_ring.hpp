#pragma once

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace motionslam_ipc {

inline constexpr char kShmCloudMagic[8] = {'M', 'S', 'L', 'C', 'L', 'D', 'R', '1'};
inline constexpr std::uint32_t kShmCloudSchemaVersion = 1;

struct PointXYZI {
  float x;
  float y;
  float z;
  float intensity;
};
static_assert(sizeof(PointXYZI) == 16);

struct alignas(64) RingHeader {
  char magic[8];
  std::uint32_t schema_version;
  std::uint32_t header_bytes;
  std::uint32_t slot_count;
  std::uint32_t max_points_per_slot;
  std::uint32_t point_stride;
  std::uint32_t slot_header_bytes;
  std::uint64_t slot_stride;
  std::atomic<std::uint64_t> generation;
  std::atomic<std::uint64_t> published_sequence;
  std::uint64_t session_id;
  std::uint64_t created_timestamp_ns;
  std::uint32_t producer_pid;
  std::uint32_t flags;
  std::uint8_t reserved[48];
};
static_assert(sizeof(RingHeader) == 128);

struct alignas(64) SlotHeader {
  // 2*N is committed sequence N; 2*N+1 means the writer is updating it.
  std::atomic<std::uint64_t> sequence_guard;
  std::uint64_t generation;
  std::uint64_t session_id;
  std::uint64_t tile_id;
  std::int64_t timestamp_ns;
  std::uint32_t point_count;
  std::uint32_t flags;
  std::uint8_t reserved[16];
};
static_assert(sizeof(SlotHeader) == 64);

struct WriterConfig {
  std::uint32_t slot_count{4};
  std::uint32_t max_points_per_slot{200000};
  std::uint64_t session_id{0};
  // Zero means increment the generation found in an existing valid segment.
  std::uint64_t generation{0};
  // Borrowed descriptor. eventfd descriptors must be inherited or transferred
  // (for example with SCM_RIGHTS); eventfd has no globally openable name.
  int notification_fd{-1};
  mode_t permissions{0660};
};

struct CloudFrame {
  std::uint64_t sequence{0};
  std::uint64_t generation{0};
  std::uint64_t session_id{0};
  std::uint64_t tile_id{0};
  std::int64_t timestamp_ns{0};
  std::uint32_t flags{0};
  std::vector<PointXYZI> points;
};

enum class ReadResult {
  kSuccess,
  kNoData,
  kOverwritten,
  kCorrupt,
};

namespace detail {

inline std::runtime_error SystemError(const char* operation) {
  return std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}

inline void ValidateName(const std::string& name) {
  if (name.size() < 2 || name.front() != '/' ||
      name.find('/', 1) != std::string::npos) {
    throw std::invalid_argument(
        "POSIX shared-memory name must be '/name' without another slash");
  }
}

inline std::uint64_t DefaultSessionId() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return static_cast<std::uint64_t>(now) ^
         (static_cast<std::uint64_t>(::getpid()) << 32);
}

inline std::uint64_t NowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

inline std::size_t CheckedMappingSize(std::uint32_t slots,
                                      std::uint32_t max_points) {
  if (slots < 2 || max_points == 0) {
    throw std::invalid_argument("slot_count must be >= 2 and max_points > 0");
  }
  constexpr std::size_t point_size = sizeof(PointXYZI);
  if (max_points >
      (std::numeric_limits<std::size_t>::max() - sizeof(SlotHeader)) /
          point_size) {
    throw std::overflow_error("shared-memory slot size overflow");
  }
  const std::size_t slot_stride =
      sizeof(SlotHeader) + static_cast<std::size_t>(max_points) * point_size;
  if (slots >
      (std::numeric_limits<std::size_t>::max() - sizeof(RingHeader)) /
          slot_stride) {
    throw std::overflow_error("shared-memory mapping size overflow");
  }
  return sizeof(RingHeader) + static_cast<std::size_t>(slots) * slot_stride;
}

inline bool HasValidPrefix(const RingHeader* header) {
  return std::memcmp(header->magic, kShmCloudMagic, sizeof(kShmCloudMagic)) == 0 &&
         header->schema_version == kShmCloudSchemaVersion &&
         header->header_bytes == sizeof(RingHeader);
}

inline void CloseNoThrow(int fd) {
  if (fd >= 0) {
    ::close(fd);
  }
}

}  // namespace detail

inline int CreateEventFd() {
  const int fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (fd < 0) {
    throw detail::SystemError("eventfd");
  }
  return fd;
}

inline bool UnlinkShmCloudRing(const std::string& name) {
  detail::ValidateName(name);
  if (::shm_unlink(name.c_str()) == 0) {
    return true;
  }
  if (errno == ENOENT) {
    return false;
  }
  throw detail::SystemError("shm_unlink");
}

class ShmCloudRingWriter {
 public:
  ShmCloudRingWriter() = default;
  ShmCloudRingWriter(const ShmCloudRingWriter&) = delete;
  ShmCloudRingWriter& operator=(const ShmCloudRingWriter&) = delete;

  ShmCloudRingWriter(ShmCloudRingWriter&& other) noexcept { MoveFrom(other); }
  ShmCloudRingWriter& operator=(ShmCloudRingWriter&& other) noexcept {
    if (this != &other) {
      Reset();
      MoveFrom(other);
    }
    return *this;
  }

  ~ShmCloudRingWriter() { Reset(); }

  static ShmCloudRingWriter Create(const std::string& name,
                                   const WriterConfig& config = {}) {
    detail::ValidateName(name);
    const std::size_t mapping_size =
        detail::CheckedMappingSize(config.slot_count,
                                   config.max_points_per_slot);
    const int fd =
        ::shm_open(name.c_str(), O_RDWR | O_CREAT | O_CLOEXEC,
                   config.permissions);
    if (fd < 0) {
      throw detail::SystemError("shm_open writer");
    }
    if (::flock(fd, LOCK_EX) != 0) {
      const auto error = detail::SystemError("flock writer");
      detail::CloseNoThrow(fd);
      throw error;
    }

    std::uint64_t old_generation = 0;
    struct stat stat_buffer {};
    if (::fstat(fd, &stat_buffer) == 0 &&
        stat_buffer.st_size >= static_cast<off_t>(sizeof(RingHeader))) {
      void* old_memory =
          ::mmap(nullptr, sizeof(RingHeader), PROT_READ | PROT_WRITE,
                 MAP_SHARED, fd, 0);
      if (old_memory != MAP_FAILED) {
        auto* old_header = static_cast<RingHeader*>(old_memory);
        if (detail::HasValidPrefix(old_header)) {
          old_generation =
              old_header->generation.load(std::memory_order_acquire);
        }
        ::munmap(old_memory, sizeof(RingHeader));
      }
    }

    if (::ftruncate(fd, static_cast<off_t>(mapping_size)) != 0) {
      const auto error = detail::SystemError("ftruncate writer");
      ::flock(fd, LOCK_UN);
      detail::CloseNoThrow(fd);
      throw error;
    }
    void* memory = ::mmap(nullptr, mapping_size, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, 0);
    if (memory == MAP_FAILED) {
      const auto error = detail::SystemError("mmap writer");
      ::flock(fd, LOCK_UN);
      detail::CloseNoThrow(fd);
      throw error;
    }

    std::memset(memory, 0, mapping_size);
    auto* header = ::new (memory) RingHeader();
    std::memcpy(header->magic, kShmCloudMagic, sizeof(kShmCloudMagic));
    header->schema_version = kShmCloudSchemaVersion;
    header->header_bytes = sizeof(RingHeader);
    header->slot_count = config.slot_count;
    header->max_points_per_slot = config.max_points_per_slot;
    header->point_stride = sizeof(PointXYZI);
    header->slot_header_bytes = sizeof(SlotHeader);
    header->slot_stride =
        sizeof(SlotHeader) +
        static_cast<std::uint64_t>(config.max_points_per_slot) *
            sizeof(PointXYZI);
    const std::uint64_t generation =
        config.generation != 0 ? config.generation : old_generation + 1;
    header->generation.store(generation, std::memory_order_relaxed);
    header->published_sequence.store(0, std::memory_order_relaxed);
    header->session_id =
        config.session_id != 0 ? config.session_id : detail::DefaultSessionId();
    header->created_timestamp_ns = detail::NowNs();
    header->producer_pid = static_cast<std::uint32_t>(::getpid());
    header->flags = config.notification_fd >= 0 ? 1U : 0U;

    auto* bytes = static_cast<std::byte*>(memory);
    for (std::uint32_t index = 0; index < config.slot_count; ++index) {
      auto* slot = reinterpret_cast<SlotHeader*>(
          bytes + sizeof(RingHeader) +
          static_cast<std::size_t>(index) * header->slot_stride);
      ::new (slot) SlotHeader();
      slot->sequence_guard.store(0, std::memory_order_relaxed);
    }
    std::atomic_thread_fence(std::memory_order_release);
    ::flock(fd, LOCK_UN);

    ShmCloudRingWriter writer;
    writer.fd_ = fd;
    writer.memory_ = memory;
    writer.mapping_size_ = mapping_size;
    writer.header_ = header;
    writer.notification_fd_ = config.notification_fd;
    return writer;
  }

  std::uint64_t Publish(const PointXYZI* points, std::size_t point_count,
                        std::int64_t timestamp_ns, std::uint64_t tile_id = 0,
                        std::uint32_t flags = 0) {
    if (header_ == nullptr) {
      throw std::logic_error("shared-memory writer is not open");
    }
    if (point_count > header_->max_points_per_slot) {
      throw std::length_error("cloud exceeds max_points_per_slot");
    }
    if (point_count != 0 && points == nullptr) {
      throw std::invalid_argument("points is null for non-empty cloud");
    }

    const std::uint64_t sequence = next_sequence_++;
    SlotHeader* slot = Slot(sequence);
    const std::uint64_t writing_guard = sequence * 2 + 1;
    const std::uint64_t committed_guard = sequence * 2;
    slot->sequence_guard.store(writing_guard, std::memory_order_release);
    slot->generation = generation();
    slot->session_id = session_id();
    slot->tile_id = tile_id;
    slot->timestamp_ns = timestamp_ns;
    slot->point_count = static_cast<std::uint32_t>(point_count);
    slot->flags = flags;
    if (point_count != 0) {
      std::memcpy(slot + 1, points, point_count * sizeof(PointXYZI));
    }
    slot->sequence_guard.store(committed_guard, std::memory_order_release);
    header_->published_sequence.store(sequence, std::memory_order_release);
    Notify();
    return sequence;
  }

  std::uint64_t Publish(const std::vector<PointXYZI>& points,
                        std::int64_t timestamp_ns, std::uint64_t tile_id = 0,
                        std::uint32_t flags = 0) {
    return Publish(points.data(), points.size(), timestamp_ns, tile_id, flags);
  }

  std::uint64_t generation() const {
    return header_->generation.load(std::memory_order_acquire);
  }
  std::uint64_t session_id() const { return header_->session_id; }
  std::uint32_t max_points_per_slot() const {
    return header_->max_points_per_slot;
  }

 private:
  SlotHeader* Slot(std::uint64_t sequence) const {
    const std::uint64_t index = (sequence - 1) % header_->slot_count;
    auto* bytes = static_cast<std::byte*>(memory_);
    return reinterpret_cast<SlotHeader*>(
        bytes + sizeof(RingHeader) + index * header_->slot_stride);
  }

  void Notify() const noexcept {
    if (notification_fd_ < 0) {
      return;
    }
    constexpr std::uint64_t one = 1;
    const ssize_t result = ::write(notification_fd_, &one, sizeof(one));
    (void)result;  // EAGAIN only means notifications were coalesced.
  }

  void Reset() noexcept {
    if (memory_ != nullptr) {
      ::munmap(memory_, mapping_size_);
    }
    detail::CloseNoThrow(fd_);
    fd_ = -1;
    memory_ = nullptr;
    mapping_size_ = 0;
    header_ = nullptr;
  }

  void MoveFrom(ShmCloudRingWriter& other) noexcept {
    fd_ = std::exchange(other.fd_, -1);
    memory_ = std::exchange(other.memory_, nullptr);
    mapping_size_ = std::exchange(other.mapping_size_, 0);
    header_ = std::exchange(other.header_, nullptr);
    notification_fd_ = std::exchange(other.notification_fd_, -1);
    next_sequence_ = std::exchange(other.next_sequence_, 1);
  }

  int fd_{-1};
  void* memory_{nullptr};
  std::size_t mapping_size_{0};
  RingHeader* header_{nullptr};
  int notification_fd_{-1};
  std::uint64_t next_sequence_{1};
};

class ShmCloudRingReader {
 public:
  ShmCloudRingReader() = default;
  ShmCloudRingReader(const ShmCloudRingReader&) = delete;
  ShmCloudRingReader& operator=(const ShmCloudRingReader&) = delete;

  ShmCloudRingReader(ShmCloudRingReader&& other) noexcept { MoveFrom(other); }
  ShmCloudRingReader& operator=(ShmCloudRingReader&& other) noexcept {
    if (this != &other) {
      Reset();
      MoveFrom(other);
    }
    return *this;
  }

  ~ShmCloudRingReader() { Reset(); }

  static ShmCloudRingReader Open(const std::string& name,
                                 int notification_fd = -1) {
    detail::ValidateName(name);
    const int fd = ::shm_open(name.c_str(), O_RDONLY | O_CLOEXEC, 0);
    if (fd < 0) {
      throw detail::SystemError("shm_open reader");
    }
    struct stat stat_buffer {};
    if (::fstat(fd, &stat_buffer) != 0) {
      const auto error = detail::SystemError("fstat reader");
      detail::CloseNoThrow(fd);
      throw error;
    }
    if (stat_buffer.st_size < static_cast<off_t>(sizeof(RingHeader))) {
      detail::CloseNoThrow(fd);
      throw std::runtime_error("shared-memory header is truncated");
    }
    const std::size_t mapping_size =
        static_cast<std::size_t>(stat_buffer.st_size);
    void* memory =
        ::mmap(nullptr, mapping_size, PROT_READ, MAP_SHARED, fd, 0);
    if (memory == MAP_FAILED) {
      const auto error = detail::SystemError("mmap reader");
      detail::CloseNoThrow(fd);
      throw error;
    }

    const auto* header = static_cast<const RingHeader*>(memory);
    try {
      ValidateHeader(*header, mapping_size);
    } catch (...) {
      ::munmap(memory, mapping_size);
      detail::CloseNoThrow(fd);
      throw;
    }

    ShmCloudRingReader reader;
    reader.fd_ = fd;
    reader.memory_ = memory;
    reader.mapping_size_ = mapping_size;
    reader.header_ = header;
    reader.notification_fd_ = notification_fd;
    return reader;
  }

  ReadResult ReadLatest(CloudFrame& frame, unsigned max_attempts = 3) const {
    if (header_ == nullptr) {
      throw std::logic_error("shared-memory reader is not open");
    }
    for (unsigned attempt = 0; attempt < max_attempts; ++attempt) {
      const std::uint64_t sequence =
          header_->published_sequence.load(std::memory_order_acquire);
      if (sequence == 0) {
        return ReadResult::kNoData;
      }
      const SlotHeader* slot = Slot(sequence);
      const std::uint64_t expected_guard = sequence * 2;
      const std::uint64_t guard_before =
          slot->sequence_guard.load(std::memory_order_acquire);
      if (guard_before != expected_guard) {
        continue;
      }
      if (slot->point_count > header_->max_points_per_slot ||
          slot->generation != generation() ||
          slot->session_id != session_id()) {
        return ReadResult::kCorrupt;
      }

      frame.sequence = sequence;
      frame.generation = slot->generation;
      frame.session_id = slot->session_id;
      frame.tile_id = slot->tile_id;
      frame.timestamp_ns = slot->timestamp_ns;
      frame.flags = slot->flags;
      frame.points.resize(slot->point_count);
      if (!frame.points.empty()) {
        std::memcpy(frame.points.data(), slot + 1,
                    frame.points.size() * sizeof(PointXYZI));
      }
      std::atomic_thread_fence(std::memory_order_acquire);
      const std::uint64_t guard_after =
          slot->sequence_guard.load(std::memory_order_acquire);
      if (guard_before == guard_after &&
          header_->published_sequence.load(std::memory_order_acquire) >=
              sequence) {
        return ReadResult::kSuccess;
      }
    }
    return ReadResult::kOverwritten;
  }

  // Waits for an optional eventfd notification and drains its counter.
  // Returns false on timeout or when no notification fd was supplied.
  bool WaitForNotification(int timeout_ms) const {
    if (notification_fd_ < 0) {
      return false;
    }
    struct pollfd descriptor {
      notification_fd_, POLLIN, 0
    };
    int result;
    do {
      result = ::poll(&descriptor, 1, timeout_ms);
    } while (result < 0 && errno == EINTR);
    if (result <= 0 || (descriptor.revents & POLLIN) == 0) {
      return false;
    }
    std::uint64_t count;
    const ssize_t bytes = ::read(notification_fd_, &count, sizeof(count));
    return bytes == static_cast<ssize_t>(sizeof(count));
  }

  std::uint64_t generation() const {
    return header_->generation.load(std::memory_order_acquire);
  }
  std::uint64_t session_id() const { return header_->session_id; }
  std::uint64_t latest_sequence() const {
    return header_->published_sequence.load(std::memory_order_acquire);
  }
  std::uint32_t max_points_per_slot() const {
    return header_->max_points_per_slot;
  }

 private:
  static void ValidateHeader(const RingHeader& header,
                             std::size_t mapping_size) {
    if (!detail::HasValidPrefix(&header)) {
      throw std::runtime_error(
          "shared-memory magic, schema, or header size mismatch");
    }
    if (header.point_stride != sizeof(PointXYZI) ||
        header.slot_header_bytes != sizeof(SlotHeader)) {
      throw std::runtime_error("shared-memory point or slot ABI mismatch");
    }
    const std::size_t expected =
        detail::CheckedMappingSize(header.slot_count,
                                   header.max_points_per_slot);
    const std::uint64_t expected_stride =
        sizeof(SlotHeader) +
        static_cast<std::uint64_t>(header.max_points_per_slot) *
            sizeof(PointXYZI);
    if (header.slot_stride != expected_stride || mapping_size != expected) {
      throw std::runtime_error("shared-memory layout or mapping size mismatch");
    }
    if (!header.generation.is_lock_free() ||
        !header.published_sequence.is_lock_free()) {
      throw std::runtime_error(
          "platform does not provide lock-free 64-bit shared atomics");
    }
  }

  const SlotHeader* Slot(std::uint64_t sequence) const {
    const std::uint64_t index = (sequence - 1) % header_->slot_count;
    const auto* bytes = static_cast<const std::byte*>(memory_);
    return reinterpret_cast<const SlotHeader*>(
        bytes + sizeof(RingHeader) + index * header_->slot_stride);
  }

  void Reset() noexcept {
    if (memory_ != nullptr) {
      ::munmap(memory_, mapping_size_);
    }
    detail::CloseNoThrow(fd_);
    fd_ = -1;
    memory_ = nullptr;
    mapping_size_ = 0;
    header_ = nullptr;
  }

  void MoveFrom(ShmCloudRingReader& other) noexcept {
    fd_ = std::exchange(other.fd_, -1);
    memory_ = std::exchange(other.memory_, nullptr);
    mapping_size_ = std::exchange(other.mapping_size_, 0);
    header_ = std::exchange(other.header_, nullptr);
    notification_fd_ = std::exchange(other.notification_fd_, -1);
  }

  int fd_{-1};
  void* memory_{nullptr};
  std::size_t mapping_size_{0};
  const RingHeader* header_{nullptr};
  int notification_fd_{-1};
};

}  // namespace motionslam_ipc
