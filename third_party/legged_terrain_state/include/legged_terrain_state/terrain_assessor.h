#ifndef LEGGED_TERRAIN_STATE_TERRAIN_ASSESSOR_H
#define LEGGED_TERRAIN_STATE_TERRAIN_ASSESSOR_H

#include "legged_terrain_state/state_contract.h"

#include <cstddef>
#include <string_view>

namespace legged_terrain {

enum class RiskLevel { Nominal, Caution, Critical };
enum class SafetyMode { Normal, ReduceSpeed, HoldPosition };

struct TerrainConfig {
    double caution_slope_deg{12.0};
    double critical_slope_deg{25.0};
    double caution_roughness_m{0.04};
    double critical_roughness_m{0.10};
    double minimum_support_confidence{0.55};
    std::size_t minimum_support_points{30};
};

struct TerrainObservation {
    StateHeader header;
    double support_height_m{};
    std::array<double, 3> estimated_normal{0.0, 0.0, 1.0};
    double roughness_m{};
    std::size_t support_points{};
    std::size_t expected_points{1};
};

struct TerrainDecision {
    TerrainState terrain;
    RiskLevel risk{RiskLevel::Nominal};
    SafetyMode safety_mode{SafetyMode::Normal};
};

class TerrainAssessor {
public:
    explicit TerrainAssessor(TerrainConfig config = {});
    TerrainDecision evaluate(const TerrainObservation& observation) const;

    static std::string_view name(RiskLevel risk) noexcept;
    static std::string_view name(SafetyMode mode) noexcept;

private:
    TerrainConfig config_;
};

}  // namespace legged_terrain

#endif
