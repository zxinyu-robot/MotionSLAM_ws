#ifndef LEGGED_TERRAIN_STATE_STATE_CONTRACT_H
#define LEGGED_TERRAIN_STATE_STATE_CONTRACT_H

#include <array>
#include <cstdint>
#include <string>

namespace legged_terrain {

enum class FaultFlag : std::uint32_t {
    None = 0,
    Stale = 1U << 0U,
    SparseSupport = 1U << 1U,
    InvalidNormal = 1U << 2U,
    ExcessiveSlope = 1U << 3U,
    ExcessiveRoughness = 1U << 4U,
    ContactUncertain = 1U << 5U,
};

constexpr FaultFlag operator|(FaultFlag lhs, FaultFlag rhs) noexcept {
    return static_cast<FaultFlag>(
        static_cast<std::uint32_t>(lhs) |
        static_cast<std::uint32_t>(rhs));
}

constexpr bool has_fault(FaultFlag flags, FaultFlag flag) noexcept {
    return (static_cast<std::uint32_t>(flags) &
            static_cast<std::uint32_t>(flag)) != 0U;
}

struct StateHeader {
    std::int64_t timestamp_ns{};
    std::string frame_id;
    std::uint64_t sequence{};
    FaultFlag faults{FaultFlag::None};
};

struct BaseState {
    StateHeader header;
    std::array<double, 3> position_m{};
    std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> linear_velocity_mps{};
    std::array<double, 6> covariance_diagonal{};
    double localization_confidence{1.0};
};

struct TerrainState {
    StateHeader header;
    double support_height_m{};
    std::array<double, 3> surface_normal{0.0, 0.0, 1.0};
    double slope_deg{};
    double roughness_m{};
    double support_confidence{};
    double traversability{};
};

struct ContactState {
    StateHeader header;
    std::array<bool, 4> in_contact{};
    std::array<double, 4> confidence{};
};

}  // namespace legged_terrain

#endif
