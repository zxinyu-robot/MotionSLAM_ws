/**
 * @file SharedMapBuffer.hpp
 * @brief /lio/global_map 投影栅格的 POSIX 共享内存缓冲区 (Nav2 零拷贝路径)
 *
 * 布局: MapShmHeader + int8_t[width*height] (0=free, 100=occupied)
 * 文件: /dev/shm/<name>
 */
#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

namespace motionslam {

constexpr uint32_t kLegacyMapShmMagic = 0x4D534D31u;  // 'MSM1'
constexpr uint32_t kMapShmMagic = 0x4D534D32u;        // 'MSM2'
constexpr uint16_t kMapShmSchema = 2;
constexpr size_t kMapFrameIdLen = 32;
constexpr size_t kMapIdentityLen = 64;

// seq 必须自然对齐，供跨进程 std::atomic_ref<uint64_t> seqlock 使用。
struct alignas(8) MapShmHeader {
  uint32_t magic{kMapShmMagic};
  uint16_t schema_version{kMapShmSchema};
  uint16_t header_size{0};
  uint64_t total_size{0};
  uint64_t seq{0};
  uint64_t context_generation{0};
  uint64_t monotonic_ns{0};
  uint32_t checksum{0};
  uint32_t width{0};
  uint32_t height{0};
  float resolution{0.05f};
  float origin_x{0.f};
  float origin_y{0.f};
  char map_id[kMapIdentityLen]{};
  char session_id[kMapIdentityLen]{};
  char tile_id[kMapIdentityLen]{};
  char frame_id[kMapFrameIdLen]{};
};
static_assert(std::is_trivially_copyable_v<MapShmHeader>);
static_assert(offsetof(MapShmHeader, seq) % alignof(uint64_t) == 0);

// 仅用于显式关闭 strict_protocol 时迁移旧生产者；生产默认不接受。
struct LegacyMapShmHeader {
  uint32_t magic{kLegacyMapShmMagic};
  uint32_t seq{0};
  uint32_t width{0};
  uint32_t height{0};
  float resolution{0.05f};
  float origin_x{0.f};
  float origin_y{0.f};
  char frame_id[kMapFrameIdLen]{};
};

struct MapSnapshot {
  MapShmHeader header{};
  std::vector<int8_t> data;
  bool legacy{false};
};

class SharedMapBuffer {
 public:
  static std::string ShmPath(const std::string& name) {
    return "/dev/shm/" + name;
  }

  static size_t TotalSize(uint32_t w, uint32_t h) {
    return sizeof(MapShmHeader) + static_cast<size_t>(w) * h;
  }

  static uint32_t Checksum(const int8_t* data, size_t size) {
    if (data == nullptr && size != 0) {
      return 0;
    }
    uint32_t crc = 0xffffffffU;
    for (size_t i = 0; i < size; ++i) {
      crc ^= static_cast<uint8_t>(data[i]);
      for (int bit = 0; bit < 8; ++bit) {
        crc = (crc >> 1U) ^ (0xedb88320U & (0U - (crc & 1U)));
      }
    }
    return ~crc;
  }

  static bool CopyText(char* target, size_t capacity, const std::string& source) {
    if (target == nullptr || capacity == 0 || source.empty() ||
        source.size() >= capacity) {
      return false;
    }
    std::memset(target, 0, capacity);
    std::memcpy(target, source.data(), source.size());
    return true;
  }

  static bool ReadText(const char* value, size_t capacity, std::string* output) {
    if (value == nullptr || output == nullptr) {
      return false;
    }
    const void* end = std::memchr(value, '\0', capacity);
    if (end == nullptr) {
      return false;
    }
    output->assign(value, static_cast<const char*>(end) - value);
    return !output->empty();
  }

  static uint64_t MonotonicNowNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  class Writer {
   public:
    explicit Writer(std::string name) : name_(std::move(name)) {}

    ~Writer() { Unmap(); }

    bool Write(const MapShmHeader& hdr, const int8_t* data) {
      if (hdr.width == 0 || hdr.height == 0 || data == nullptr) {
        return false;
      }
      std::string identity;
      if (hdr.context_generation == 0U ||
          !std::isfinite(hdr.resolution) || hdr.resolution <= 0.0F ||
          !std::isfinite(hdr.origin_x) || !std::isfinite(hdr.origin_y) ||
          !ReadText(hdr.map_id, kMapIdentityLen, &identity) ||
          !ReadText(hdr.session_id, kMapIdentityLen, &identity) ||
          !ReadText(hdr.tile_id, kMapIdentityLen, &identity) ||
          !ReadText(hdr.frame_id, kMapFrameIdLen, &identity)) {
        return false;
      }
      const size_t need = TotalSize(hdr.width, hdr.height);
      const size_t cells = static_cast<size_t>(hdr.width) * hdr.height;
      if (cells > std::numeric_limits<size_t>::max() - sizeof(MapShmHeader) ||
          need < sizeof(MapShmHeader) ||
          need > static_cast<size_t>(std::numeric_limits<off_t>::max())) {
        return false;
      }
      std::lock_guard<std::mutex> lock(mu_);
      if (!EnsureMapped(need)) {
        return false;
      }
      auto* out = static_cast<MapShmHeader*>(mem_);
      std::atomic_ref<uint64_t> sequence(out->seq);
      uint64_t previous = 0;
      if (out->magic == kMapShmMagic &&
          out->schema_version == kMapShmSchema) {
        previous = sequence.load(std::memory_order_relaxed);
      }
      const uint64_t odd = (previous & 1U) != 0U ? previous + 2U : previous + 1U;
      sequence.store(odd, std::memory_order_release);

      MapShmHeader published = hdr;
      published.magic = kMapShmMagic;
      published.schema_version = kMapShmSchema;
      published.header_size = sizeof(MapShmHeader);
      published.total_size = need;
      published.seq = odd;
      if (published.monotonic_ns == 0U) {
        published.monotonic_ns = MonotonicNowNs();
      }
      published.checksum = Checksum(data, cells);
      constexpr size_t seq_offset = offsetof(MapShmHeader, seq);
      constexpr size_t after_seq = seq_offset + sizeof(uint64_t);
      std::memcpy(static_cast<void*>(reinterpret_cast<char*>(out)),
                  static_cast<const void*>(
                      reinterpret_cast<const char*>(&published)),
                  seq_offset);
      std::memcpy(reinterpret_cast<char*>(out) + after_seq,
                  reinterpret_cast<const char*>(&published) + after_seq,
                  sizeof(published) - after_seq);
      auto* grid = reinterpret_cast<int8_t*>(mem_) + sizeof(MapShmHeader);
      std::memcpy(grid, data, cells);
      std::atomic_thread_fence(std::memory_order_release);
      sequence.store(odd + 1U, std::memory_order_release);
      msync(mem_, mapped_, MS_ASYNC);
      return true;
    }

   private:
    bool EnsureMapped(size_t size) {
      if (mem_ != nullptr && mapped_ >= size) {
        return true;
      }
      Unmap();
      const std::string path = ShmPath(name_);
      fd_ = ::open(path.c_str(), O_RDWR | O_CREAT, 0666);
      if (fd_ < 0) {
        return false;
      }
      if (::ftruncate(fd_, static_cast<off_t>(size)) != 0) {
        Unmap();
        return false;
      }
      mem_ = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
      if (mem_ == MAP_FAILED) {
        mem_ = nullptr;
        Unmap();
        return false;
      }
      mapped_ = size;
      return true;
    }

    void Unmap() {
      if (mem_ != nullptr && mem_ != MAP_FAILED) {
        ::munmap(mem_, mapped_);
      }
      mem_ = nullptr;
      mapped_ = 0;
      if (fd_ >= 0) {
        ::close(fd_);
      }
      fd_ = -1;
    }

    std::string name_;
    std::mutex mu_;
    int fd_{-1};
    void* mem_{nullptr};
    size_t mapped_{0};
  };

  class Reader {
   public:
    explicit Reader(std::string name, bool strict_protocol = true)
        : name_(std::move(name)), strict_protocol_(strict_protocol) {}

    ~Reader() { Close(); }

    bool Open() {
      Close();
      const std::string path = ShmPath(name_);
      fd_ = ::open(path.c_str(), O_RDONLY);
      if (fd_ < 0) {
        return false;
      }
      struct stat st {};
      const size_t minimum =
          strict_protocol_ ? sizeof(MapShmHeader) : sizeof(LegacyMapShmHeader);
      if (::fstat(fd_, &st) != 0 ||
          st.st_size < static_cast<off_t>(minimum)) {
        Close();
        return false;
      }
      mapped_ = static_cast<size_t>(st.st_size);
      mem_ = ::mmap(nullptr, mapped_, PROT_READ, MAP_SHARED, fd_, 0);
      if (mem_ == MAP_FAILED) {
        mem_ = nullptr;
        Close();
        return false;
      }
      MapSnapshot candidate;
      if (!ReadSnapshot(&candidate)) {
        Close();
        return false;
      }
      snapshot_ = std::move(candidate);
      return true;
    }

    bool Refresh() { return Open(); }

    bool Valid() const { return mem_ != nullptr && !snapshot_.data.empty(); }

    const MapShmHeader* Header() const {
      return Valid() ? &snapshot_.header : nullptr;
    }

    const int8_t* Data() const {
      return Valid() ? snapshot_.data.data() : nullptr;
    }

    const MapSnapshot& Snapshot() const { return snapshot_; }

    bool ReadSnapshot(MapSnapshot* output, std::string* error = nullptr) const {
      if (output == nullptr || mem_ == nullptr) {
        SetError(error, "map_not_mapped");
        return false;
      }
      const auto* header = static_cast<const MapShmHeader*>(mem_);
      if (header->magic == kLegacyMapShmMagic) {
        return ReadLegacySnapshot(output, error);
      }
      if (header->magic != kMapShmMagic) {
        SetError(error, "map_magic_mismatch");
        return false;
      }

      for (int attempt = 0; attempt < 8; ++attempt) {
        auto& mutable_seq = const_cast<uint64_t&>(header->seq);
        std::atomic_ref<uint64_t> sequence(mutable_seq);
        const uint64_t before = sequence.load(std::memory_order_acquire);
        if ((before & 1U) != 0U) {
          continue;
        }
        MapShmHeader copy;
        std::memcpy(&copy, header, sizeof(copy));
        if (!ValidateHeader(copy, mapped_, error)) {
          return false;
        }
        const size_t cells = static_cast<size_t>(copy.width) * copy.height;
        std::vector<int8_t> data(cells);
        std::memcpy(data.data(),
                    reinterpret_cast<const int8_t*>(mem_) + copy.header_size,
                    cells);
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint64_t after = sequence.load(std::memory_order_acquire);
        if (before != after || (after & 1U) != 0U) {
          continue;
        }
        if (Checksum(data.data(), data.size()) != copy.checksum) {
          SetError(error, "map_checksum_mismatch");
          return false;
        }
        output->header = copy;
        output->data = std::move(data);
        output->legacy = false;
        return true;
      }
      SetError(error, "map_snapshot_unstable");
      return false;
    }

   private:
    static void SetError(std::string* error, const char* value) {
      if (error != nullptr) {
        *error = value;
      }
    }

    static bool ValidateHeader(const MapShmHeader& header, size_t mapped,
                               std::string* error) {
      if (header.schema_version != kMapShmSchema ||
          header.header_size != sizeof(MapShmHeader)) {
        SetError(error, "map_schema_mismatch");
        return false;
      }
      if (header.width == 0 || header.height == 0 ||
          header.width > std::numeric_limits<size_t>::max() / header.height) {
        SetError(error, "map_geometry_invalid");
        return false;
      }
      const size_t cells = static_cast<size_t>(header.width) * header.height;
      if (cells > std::numeric_limits<size_t>::max() - sizeof(MapShmHeader)) {
        SetError(error, "map_size_overflow");
        return false;
      }
      if (header.total_size != sizeof(MapShmHeader) + cells ||
          header.total_size > mapped) {
        SetError(error, "map_size_mismatch");
        return false;
      }
      std::string text;
      if (header.context_generation == 0U ||
          header.monotonic_ns == 0U ||
          !std::isfinite(header.resolution) || header.resolution <= 0.0F ||
          !std::isfinite(header.origin_x) ||
          !std::isfinite(header.origin_y)) {
        SetError(error, "map_metadata_invalid");
        return false;
      }
      if (!ReadText(header.map_id, kMapIdentityLen, &text) ||
          !ReadText(header.session_id, kMapIdentityLen, &text) ||
          !ReadText(header.tile_id, kMapIdentityLen, &text) ||
          !ReadText(header.frame_id, kMapFrameIdLen, &text)) {
        SetError(error, "map_identity_invalid");
        return false;
      }
      return true;
    }

    bool ReadLegacySnapshot(MapSnapshot* output, std::string* error) const {
      if (strict_protocol_) {
        SetError(error, "legacy_map_rejected_by_strict_protocol");
        return false;
      }
      LegacyMapShmHeader legacy;
      std::memcpy(&legacy, mem_, sizeof(legacy));
      if (legacy.width == 0 || legacy.height == 0 ||
          legacy.width > std::numeric_limits<size_t>::max() / legacy.height) {
        SetError(error, "legacy_map_geometry_invalid");
        return false;
      }
      const size_t cells = static_cast<size_t>(legacy.width) * legacy.height;
      if (sizeof(legacy) + cells > mapped_) {
        SetError(error, "legacy_map_size_mismatch");
        return false;
      }
      MapShmHeader converted;
      converted.magic = kLegacyMapShmMagic;
      converted.schema_version = 1;
      converted.header_size = sizeof(legacy);
      converted.total_size = sizeof(legacy) + cells;
      converted.seq = legacy.seq;
      converted.width = legacy.width;
      converted.height = legacy.height;
      converted.resolution = legacy.resolution;
      converted.origin_x = legacy.origin_x;
      converted.origin_y = legacy.origin_y;
      std::memcpy(converted.frame_id, legacy.frame_id, kMapFrameIdLen);
      output->header = converted;
      output->data.resize(cells);
      std::memcpy(output->data.data(),
                  reinterpret_cast<const int8_t*>(mem_) + sizeof(legacy),
                  cells);
      output->legacy = true;
      return true;
    }

    void Close() {
      if (mem_ != nullptr && mem_ != MAP_FAILED) {
        ::munmap(mem_, mapped_);
      }
      mem_ = nullptr;
      mapped_ = 0;
      if (fd_ >= 0) {
        ::close(fd_);
      }
      fd_ = -1;
      snapshot_ = {};
    }

    std::string name_;
    bool strict_protocol_{true};
    int fd_{-1};
    void* mem_{nullptr};
    size_t mapped_{0};
    MapSnapshot snapshot_;
  };
};

}  // namespace motionslam
