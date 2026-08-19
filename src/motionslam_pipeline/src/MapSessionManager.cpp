#include "motionslam/MapSessionManager.hpp"

#include <fcntl.h>
#include <openssl/evp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace motionslam {
namespace {

namespace fs = std::filesystem;

void SetError(std::string* error, const std::string& value) {
  if (error != nullptr) {
    *error = value;
  }
}

template <size_t N>
void CopyText(std::array<char, N>* target, const std::string& source) {
  target->fill('\0');
  std::memcpy(target->data(), source.data(), std::min(N - 1, source.size()));
}

std::string ResolveMapPath(const YAML::Node& spec, const fs::path& base) {
  if (!spec || !spec["path"]) {
    return {};
  }
  fs::path path(spec["path"].as<std::string>());
  if (path.is_relative()) {
    path = base / path;
  }
  path = path.lexically_normal();
  if (fs::is_regular_file(path)) {
    return fs::absolute(path).string();
  }
  if (spec["fallback"]) {
    fs::path fallback(spec["fallback"].as<std::string>());
    if (fallback.is_relative()) {
      fallback = base / fallback;
    }
    if (fs::is_regular_file(fallback)) {
      return fs::absolute(fallback).lexically_normal().string();
    }
  }
  return path.string();
}

bool ReadMapFile(const YAML::Node& tile, const char* key, const fs::path& base,
                 MapFile* output, std::string* error) {
  const YAML::Node spec = tile[key];
  if (!spec || !spec["path"] || !spec["md5"]) {
    SetError(error, std::string("tile 缺少 ") + key + ".path/md5");
    return false;
  }
  output->path = ResolveMapPath(spec, base);
  output->md5 = spec["md5"].as<std::string>();
  if (!fs::is_regular_file(output->path)) {
    SetError(error, std::string(key) + " 不存在: " + output->path);
    return false;
  }
  std::string md5_error;
  const std::string actual = MapSessionManager::ComputeMd5(output->path, &md5_error);
  if (actual.empty() || actual != output->md5) {
    SetError(error, std::string(key) + " md5 不匹配: expected=" + output->md5 +
                        " actual=" + (actual.empty() ? md5_error : actual));
    return false;
  }
  return true;
}

bool ComputeBounds(const std::string& pcd_path, Bounds3d* bounds,
                   std::string* error) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(pcd_path, cloud) < 0 || cloud.empty()) {
    SetError(error, "无法读取 map_nav PCD 或点云为空: " + pcd_path);
    return false;
  }
  const double inf = std::numeric_limits<double>::infinity();
  Bounds3d result{inf, inf, inf, -inf, -inf, -inf};
  size_t finite_count = 0;
  for (const auto& point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    result.min_x = std::min(result.min_x, static_cast<double>(point.x));
    result.min_y = std::min(result.min_y, static_cast<double>(point.y));
    result.min_z = std::min(result.min_z, static_cast<double>(point.z));
    result.max_x = std::max(result.max_x, static_cast<double>(point.x));
    result.max_y = std::max(result.max_y, static_cast<double>(point.y));
    result.max_z = std::max(result.max_z, static_cast<double>(point.z));
    ++finite_count;
  }
  if (finite_count == 0 || !result.Valid()) {
    SetError(error, "map_nav PCD 没有有限 XYZ 点: " + pcd_path);
    return false;
  }
  *bounds = result;
  return true;
}

bool ParseBounds(const YAML::Node& node, Bounds3d* bounds) {
  if (!node || !node.IsSequence() || node.size() != 6) {
    return false;
  }
  *bounds = Bounds3d{node[0].as<double>(), node[1].as<double>(),
                     node[2].as<double>(), node[3].as<double>(),
                     node[4].as<double>(), node[5].as<double>()};
  return bounds->Valid();
}

bool SameContextRevision(const NavigationContextState& left,
                         const NavigationContextState& right) {
  return left.map_version == right.map_version &&
         left.bounds.min_x == right.bounds.min_x &&
         left.bounds.min_y == right.bounds.min_y &&
         left.bounds.min_z == right.bounds.min_z &&
         left.bounds.max_x == right.bounds.max_x &&
         left.bounds.max_y == right.bounds.max_y &&
         left.bounds.max_z == right.bounds.max_z &&
         left.building_id == right.building_id &&
         left.floor_id == right.floor_id &&
         left.map_id == right.map_id &&
         left.session_id == right.session_id &&
         left.tile_id == right.tile_id &&
         left.frame_id == right.frame_id;
}

}  // namespace

bool Bounds3d::Valid() const {
  return std::isfinite(min_x) && std::isfinite(min_y) && std::isfinite(min_z) &&
         std::isfinite(max_x) && std::isfinite(max_y) && std::isfinite(max_z) &&
         min_x <= max_x && min_y <= max_y && min_z <= max_z;
}

bool Bounds3d::Contains(double x, double y, double z, double margin) const {
  return Valid() && std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         margin >= 0.0 && x >= min_x + margin && x <= max_x - margin &&
         y >= min_y + margin && y <= max_y - margin &&
         z >= min_z + margin && z <= max_z - margin;
}

NavigationContextStatePage::NavigationContextStatePage(std::string path)
    : path_(std::move(path)) {}

NavigationContextStatePage::~NavigationContextStatePage() { Close(); }

bool NavigationContextStatePage::EnsureMapped(std::string* error) {
  if (memory_ != nullptr) {
    return true;
  }
  fd_ = ::open(path_.c_str(), O_RDWR | O_CREAT, 0660);
  if (fd_ < 0) {
    SetError(error, "无法打开状态页: " + path_ + ": " + std::strerror(errno));
    return false;
  }
  if (::ftruncate(fd_, sizeof(NavigationContextState)) != 0) {
    SetError(error, "无法调整状态页大小: " + std::string(std::strerror(errno)));
    Close();
    return false;
  }
  memory_ = ::mmap(nullptr, sizeof(NavigationContextState),
                   PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (memory_ == MAP_FAILED) {
    memory_ = nullptr;
    SetError(error, "无法映射状态页: " + std::string(std::strerror(errno)));
    Close();
    return false;
  }
  return true;
}

void NavigationContextStatePage::Close() {
  if (memory_ != nullptr) {
    ::munmap(memory_, sizeof(NavigationContextState));
    memory_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool NavigationContextStatePage::Write(const NavigationContext& context,
                                       std::string* error) {
  if (!EnsureMapped(error)) {
    return false;
  }
  auto* shared = static_cast<NavigationContextState*>(memory_);
  std::atomic_ref<uint64_t> shared_sequence(shared->sequence);
  uint64_t previous_sequence = 0;
  NavigationContextState previous_state;
  bool previous_valid = false;
  if (shared->magic == kNavigationContextMagic &&
      shared->schema_version == kNavigationContextSchema &&
      shared->header_size == sizeof(NavigationContextState) &&
      shared->total_size == sizeof(NavigationContextState)) {
    for (int attempt = 0; attempt < 8; ++attempt) {
      const uint64_t before =
          shared_sequence.load(std::memory_order_acquire);
      if ((before & 1U) != 0U) {
        continue;
      }
      std::memcpy(&previous_state, shared, sizeof(previous_state));
      std::atomic_thread_fence(std::memory_order_acquire);
      const uint64_t after =
          shared_sequence.load(std::memory_order_acquire);
      if (before == after && (after & 1U) == 0U) {
        previous_sequence = after;
        previous_valid = true;
        break;
      }
    }
  }
  const uint64_t odd_sequence =
      (previous_sequence & 1U) ? previous_sequence + 2 : previous_sequence + 1;

  NavigationContextState state;
  state.header_size = sizeof(NavigationContextState);
  state.total_size = sizeof(NavigationContextState);
  state.sequence = odd_sequence;
  state.monotonic_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
  state.map_version = context.floor.map_version;
  state.session_token = StableContextToken(context.session_id);
  state.tile_token = StableContextToken(context.floor.tile_id);
  state.registry_valid = context.registry_valid;
  state.relocation_converged = context.relocation_converged;
  state.ready = context.ready;
  state.relocation_fitness = context.relocation_fitness;
  state.max_fitness = context.max_fitness;
  state.bounds = context.floor.bounds;
  CopyText(&state.building_id, context.floor.building_id);
  CopyText(&state.floor_id, context.floor.floor_id);
  CopyText(&state.map_id, context.floor.map_id);
  CopyText(&state.session_id, context.session_id);
  CopyText(&state.tile_id, context.floor.tile_id);
  CopyText(&state.frame_id, context.floor.frame_id);
  CopyText(&state.reason, context.reason);
  if (!has_successful_write_) {
    // writer 实例重启本身就是发布 epoch 边界。即使地图身份未变，也必须
    // 让绑定旧进程 generation 的静态图立即失效并等待重新发布。
    state.generation = previous_valid ? previous_state.generation + 1U : 1U;
  } else {
    state.generation =
        previous_valid && SameContextRevision(previous_state, state)
            ? previous_state.generation
            : (previous_valid ? previous_state.generation + 1U : 1U);
  }

  shared_sequence.store(odd_sequence, std::memory_order_release);
  constexpr size_t sequence_offset =
      offsetof(NavigationContextState, sequence);
  constexpr size_t after_sequence =
      sequence_offset + sizeof(uint64_t);
  std::memcpy(memory_, &state, sequence_offset);
  std::memcpy(static_cast<char*>(memory_) + after_sequence,
              reinterpret_cast<const char*>(&state) + after_sequence,
              sizeof(state) - after_sequence);
  std::atomic_thread_fence(std::memory_order_release);
  shared_sequence.store(odd_sequence + 1, std::memory_order_release);
  has_successful_write_ = true;
  return true;
}

bool NavigationContextStatePage::Read(const std::string& path,
                                      NavigationContextState* state,
                                      std::string* error) {
  if (state == nullptr) {
    SetError(error, "state 输出为空");
    return false;
  }
  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    SetError(error, "状态页不存在: " + path);
    return false;
  }
  struct stat stat_buffer {};
  if (::fstat(fd, &stat_buffer) != 0 ||
      stat_buffer.st_size != static_cast<off_t>(sizeof(NavigationContextState))) {
    ::close(fd);
    SetError(error, "状态页大小或 schema 不兼容");
    return false;
  }
  void* memory = ::mmap(nullptr, sizeof(NavigationContextState), PROT_READ,
                        MAP_SHARED, fd, 0);
  if (memory == MAP_FAILED) {
    ::close(fd);
    SetError(error, "无法读取状态页");
    return false;
  }
  const auto* shared = static_cast<const NavigationContextState*>(memory);
  bool success = false;
  for (int attempt = 0; attempt < 5; ++attempt) {
    auto& mutable_sequence = const_cast<uint64_t&>(shared->sequence);
    std::atomic_ref<uint64_t> sequence(mutable_sequence);
    const uint64_t before = sequence.load(std::memory_order_acquire);
    if (before & 1U) {
      continue;
    }
    std::memcpy(state, shared, sizeof(*state));
    std::atomic_thread_fence(std::memory_order_acquire);
    const uint64_t after = sequence.load(std::memory_order_acquire);
    if (before == after && !(after & 1U)) {
      success = true;
      break;
    }
  }
  ::munmap(memory, sizeof(NavigationContextState));
  ::close(fd);
  if (!success || state->magic != kNavigationContextMagic ||
      state->schema_version != kNavigationContextSchema ||
      state->header_size != sizeof(NavigationContextState) ||
      state->total_size != sizeof(NavigationContextState)) {
    SetError(error, "状态页正在更新或 magic/schema 无效");
    return false;
  }
  return true;
}

std::string MapSessionManager::ComputeMd5(const std::string& path,
                                          std::string* error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    SetError(error, "无法打开文件: " + path);
    return {};
  }
  EVP_MD_CTX* context = EVP_MD_CTX_new();
  if (context == nullptr || EVP_DigestInit_ex(context, EVP_md5(), nullptr) != 1) {
    EVP_MD_CTX_free(context);
    SetError(error, "无法初始化 MD5");
    return {};
  }
  std::array<char, 64 * 1024> buffer{};
  while (input) {
    input.read(buffer.data(), buffer.size());
    if (input.gcount() > 0 &&
        EVP_DigestUpdate(context, buffer.data(),
                         static_cast<size_t>(input.gcount())) != 1) {
      EVP_MD_CTX_free(context);
      SetError(error, "MD5 更新失败");
      return {};
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context, digest.data(), &digest_size) != 1) {
    EVP_MD_CTX_free(context);
    SetError(error, "MD5 计算失败");
    return {};
  }
  EVP_MD_CTX_free(context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < digest_size; ++i) {
    output << std::setw(2) << static_cast<unsigned int>(digest[i]);
  }
  return output.str();
}

bool MapSessionManager::Load(const std::string& floor_registry_file,
                             const std::string& building_id,
                             const std::string& floor_id,
                             const std::string& map_id,
                             const std::string& session_id,
                             const std::string& tile_id, double max_fitness,
                             std::string* error) {
  context_ = NavigationContext{};
  context_.session_id = session_id;
  context_.max_fitness = max_fitness;
  if (building_id.empty() || floor_id.empty() || map_id.empty() ||
      session_id.empty() || tile_id.empty() || !std::isfinite(max_fitness) ||
      max_fitness < 0.0) {
    SetError(error, "building/floor/map/session/tile 和 max_fitness 参数无效");
    return false;
  }

  try {
    const fs::path registry_path = fs::absolute(floor_registry_file);
    const YAML::Node root = YAML::LoadFile(registry_path.string());
    if (!root["schema_version"] || root["schema_version"].as<uint32_t>() != 1 ||
        !root["floors"] || !root["floors"].IsSequence()) {
      SetError(error, "floor_registry schema_version/floors 无效");
      return false;
    }
    fs::path floor_registry_path;
    for (const auto& entry : root["floors"]) {
      if (entry["building_id"] && entry["floor_id"] &&
          entry["building_id"].as<std::string>() == building_id &&
          entry["floor_id"].as<std::string>() == floor_id &&
          entry["registry"]) {
        floor_registry_path = registry_path.parent_path() /
                              entry["registry"].as<std::string>();
        break;
      }
    }
    if (floor_registry_path.empty()) {
      SetError(error, "floor_registry 中没有请求的 building/floor");
      return false;
    }
    floor_registry_path = floor_registry_path.lexically_normal();
    const YAML::Node floor = YAML::LoadFile(floor_registry_path.string());
    if (!floor["schema_version"] || floor["schema_version"].as<uint32_t>() != 1 ||
        !floor["building_id"] || floor["building_id"].as<std::string>() != building_id ||
        !floor["floor_id"] || floor["floor_id"].as<std::string>() != floor_id ||
        !floor["map_id"] || floor["map_id"].as<std::string>() != map_id ||
        !floor["map_version"] || floor["map_version"].as<uint64_t>() == 0 ||
        !floor["tiles"] || !floor["tiles"][tile_id]) {
      SetError(error, "楼层 registry 的 schema/building/floor/map/tile 不匹配");
      return false;
    }

    context_.floor.schema_version = 1;
    context_.floor.building_id = building_id;
    context_.floor.floor_id = floor_id;
    context_.floor.map_id = map_id;
    context_.floor.map_version = floor["map_version"].as<uint64_t>();
    context_.floor.tile_id = tile_id;
    context_.floor.frame_id =
        floor["frame_id"] ? floor["frame_id"].as<std::string>() : "world";
    const YAML::Node tile = floor["tiles"][tile_id];
    const fs::path base = floor_registry_path.parent_path();
    if (!ReadMapFile(tile, "map_nav", base, &context_.floor.map_nav, error) ||
        !ReadMapFile(tile, "map_reloc", base, &context_.floor.map_reloc, error) ||
        !ReadMapFile(tile, "map_viz", base, &context_.floor.map_viz, error)) {
      return false;
    }
    if (!ParseBounds(floor["bounds"], &context_.floor.bounds) &&
        !ComputeBounds(context_.floor.map_nav.path, &context_.floor.bounds, error)) {
      return false;
    }
  } catch (const YAML::Exception& exception) {
    SetError(error, std::string("YAML 读取失败: ") + exception.what());
    return false;
  } catch (const std::exception& exception) {
    SetError(error, std::string("registry 加载失败: ") + exception.what());
    return false;
  }

  context_.registry_valid = true;
  RefreshReady();
  return true;
}

void MapSessionManager::UpdateRelocation(bool converged, double fitness) {
  context_.relocation_converged = converged;
  context_.relocation_fitness = fitness;
  RefreshReady();
}

void MapSessionManager::SetRequireRelocationConverged(bool required) {
  require_relocation_converged_ = required;
  if (!require_relocation_converged_) {
    context_.relocation_converged = true;
    context_.relocation_fitness = 0.0;
  }
  RefreshReady();
}

void MapSessionManager::RefreshReady() {
  if (!context_.registry_valid) {
    context_.ready = false;
    context_.reason = "registry_invalid";
  } else if (require_relocation_converged_ && !context_.relocation_converged) {
    context_.ready = false;
    context_.reason = "relocation_not_converged";
  } else if (require_relocation_converged_ &&
             !std::isfinite(context_.relocation_fitness)) {
    context_.ready = false;
    context_.reason = "fitness_invalid";
  } else if (require_relocation_converged_ &&
             context_.relocation_fitness > context_.max_fitness) {
    context_.ready = false;
    context_.reason = "fitness_above_threshold";
  } else {
    if (!require_relocation_converged_) {
      context_.relocation_converged = true;
      context_.relocation_fitness = 0.0;
    }
    context_.ready = true;
    context_.reason = "ready";
  }
}

GateResult MapSessionManager::CheckPose(const GateRequest& request,
                                        double bounds_margin) const {
  if (!context_.ready) {
    return {false, context_.reason};
  }
  if (request.map_id != context_.floor.map_id) {
    return {false, "map_id_mismatch"};
  }
  if (request.session_id != context_.session_id) {
    return {false, "session_id_mismatch"};
  }
  if (request.floor_id != context_.floor.floor_id) {
    return {false, "floor_id_mismatch"};
  }
  if (request.tile_id != context_.floor.tile_id) {
    return {false, "tile_id_mismatch"};
  }
  if (!std::isfinite(request.x) || !std::isfinite(request.y) ||
      !std::isfinite(bounds_margin) || bounds_margin < 0.0) {
    return {false, "pose_or_margin_invalid"};
  }
  const auto& bounds = context_.floor.bounds;
  if (request.x < bounds.min_x + bounds_margin ||
      request.x > bounds.max_x - bounds_margin ||
      request.y < bounds.min_y + bounds_margin ||
      request.y > bounds.max_y - bounds_margin) {
    return {false, "outside_map_bounds"};
  }
  return {true, "accepted"};
}

GateResult MapSessionManager::CheckStart(const GateRequest& request,
                                         double bounds_margin) const {
  return CheckPose(request, bounds_margin);
}

GateResult MapSessionManager::CheckGoal(const GateRequest& request,
                                        double bounds_margin) const {
  return CheckPose(request, bounds_margin);
}

}  // namespace motionslam
