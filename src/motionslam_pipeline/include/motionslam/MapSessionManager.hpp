#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

namespace motionslam {

constexpr uint32_t kNavigationContextMagic = 0x4d4e4331U;  // "MNC1"
constexpr uint16_t kNavigationContextSchema = 3;
constexpr size_t kContextIdLength = 64;

constexpr uint64_t StableContextToken(std::string_view value) {
  if (value.empty()) {
    return 0;
  }
  uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash == 0 ? 1 : hash;
}

struct Bounds3d {
  double min_x{0.0};
  double min_y{0.0};
  double min_z{0.0};
  double max_x{0.0};
  double max_y{0.0};
  double max_z{0.0};

  bool Valid() const;
  bool Contains(double x, double y, double z, double margin = 0.0) const;
};

struct MapFile {
  std::string path;
  std::string md5;
};

struct FloorMap {
  uint32_t schema_version{0};
  std::string building_id;
  std::string floor_id;
  std::string map_id;
  uint64_t map_version{0};
  std::string tile_id;
  std::string frame_id;
  MapFile map_nav;
  MapFile map_reloc;
  MapFile map_viz;
  Bounds3d bounds;
};

struct NavigationContext {
  FloorMap floor;
  std::string session_id;
  bool registry_valid{false};
  bool relocation_converged{false};
  double relocation_fitness{std::numeric_limits<double>::infinity()};
  double max_fitness{1.0};
  bool ready{false};
  std::string reason;
};

struct GateRequest {
  std::string map_id;
  std::string session_id;
  std::string floor_id;
  std::string tile_id;
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GateResult {
  bool accepted{false};
  std::string reason;
};

struct NavigationContextState {
  uint32_t magic{kNavigationContextMagic};
  uint16_t schema_version{kNavigationContextSchema};
  uint16_t header_size{0};
  uint32_t total_size{0};
  uint64_t sequence{0};
  uint64_t generation{0};
  uint64_t monotonic_ns{0};
  uint64_t map_version{0};
  uint64_t session_token{0};
  uint64_t tile_token{0};
  uint8_t registry_valid{0};
  uint8_t relocation_converged{0};
  uint8_t ready{0};
  uint8_t reserved{0};
  double relocation_fitness{0.0};
  double max_fitness{0.0};
  Bounds3d bounds;
  std::array<char, kContextIdLength> building_id{};
  std::array<char, kContextIdLength> floor_id{};
  std::array<char, kContextIdLength> map_id{};
  std::array<char, kContextIdLength> session_id{};
  std::array<char, kContextIdLength> tile_id{};
  std::array<char, kContextIdLength> frame_id{};
  std::array<char, 128> reason{};
};

class NavigationContextStatePage {
 public:
  explicit NavigationContextStatePage(
      std::string path = "/dev/shm/motionslam_navigation_context");
  ~NavigationContextStatePage();
  NavigationContextStatePage(const NavigationContextStatePage&) = delete;
  NavigationContextStatePage& operator=(const NavigationContextStatePage&) = delete;

  bool Write(const NavigationContext& context, std::string* error = nullptr);
  static bool Read(const std::string& path, NavigationContextState* state,
                   std::string* error = nullptr);

 private:
  bool EnsureMapped(std::string* error);
  void Close();

  std::string path_;
  int fd_{-1};
  void* memory_{nullptr};
  bool has_successful_write_{false};
};

class MapSessionManager {
 public:
  bool Load(const std::string& floor_registry_file,
            const std::string& building_id, const std::string& floor_id,
            const std::string& map_id, const std::string& session_id,
            const std::string& tile_id, double max_fitness,
            std::string* error = nullptr);

  void UpdateRelocation(bool converged, double fitness);
  void SetRequireRelocationConverged(bool required);
  const NavigationContext& Context() const { return context_; }
  GateResult CheckStart(const GateRequest& request,
                        double bounds_margin = 0.0) const;
  GateResult CheckGoal(const GateRequest& request,
                       double bounds_margin = 0.0) const;

  static std::string ComputeMd5(const std::string& path,
                                std::string* error = nullptr);

 private:
  GateResult CheckPose(const GateRequest& request, double bounds_margin) const;
  void RefreshReady();

  NavigationContext context_;
  bool require_relocation_converged_{true};
};

}  // namespace motionslam
