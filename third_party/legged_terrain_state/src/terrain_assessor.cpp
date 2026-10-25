#include "legged_terrain_state/terrain_assessor.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace legged_terrain {

TerrainAssessor::TerrainAssessor(TerrainConfig config)
    : config_(std::move(config)) {
    if (config_.caution_slope_deg < 0.0 ||
        config_.critical_slope_deg <= config_.caution_slope_deg ||
        config_.caution_roughness_m < 0.0 ||
        config_.critical_roughness_m <= config_.caution_roughness_m ||
        config_.minimum_support_confidence < 0.0 ||
        config_.minimum_support_confidence > 1.0 ||
        config_.minimum_support_points == 0) {
        throw std::invalid_argument("invalid terrain assessment config");
    }
}

TerrainDecision TerrainAssessor::evaluate(
    const TerrainObservation& observation) const {
    if (!std::isfinite(observation.support_height_m) ||
        !std::isfinite(observation.roughness_m) ||
        observation.roughness_m < 0.0 || observation.expected_points == 0) {
        throw std::invalid_argument("invalid terrain observation");
    }

    TerrainDecision decision;
    decision.terrain.header = observation.header;
    decision.terrain.support_height_m = observation.support_height_m;
    decision.terrain.roughness_m = observation.roughness_m;

    const double nx = observation.estimated_normal[0];
    const double ny = observation.estimated_normal[1];
    const double nz = observation.estimated_normal[2];
    const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    const bool valid_normal = std::isfinite(norm) && norm > 1e-9;
    if (valid_normal) {
        decision.terrain.surface_normal = {nx / norm, ny / norm, nz / norm};
        const double cosine =
            std::clamp(std::abs(nz / norm), 0.0, 1.0);
        decision.terrain.slope_deg =
            std::acos(cosine) * 180.0 / std::numbers::pi;
    } else {
        decision.terrain.header.faults =
            decision.terrain.header.faults | FaultFlag::InvalidNormal;
    }

    decision.terrain.support_confidence = std::clamp(
        static_cast<double>(observation.support_points) /
            static_cast<double>(observation.expected_points),
        0.0,
        1.0);

    if (observation.support_points < config_.minimum_support_points ||
        decision.terrain.support_confidence <
            config_.minimum_support_confidence) {
        decision.terrain.header.faults =
            decision.terrain.header.faults | FaultFlag::SparseSupport;
    }
    if (decision.terrain.slope_deg >= config_.critical_slope_deg) {
        decision.terrain.header.faults =
            decision.terrain.header.faults | FaultFlag::ExcessiveSlope;
    }
    if (observation.roughness_m >= config_.critical_roughness_m) {
        decision.terrain.header.faults =
            decision.terrain.header.faults | FaultFlag::ExcessiveRoughness;
    }

    const double slope_score = std::clamp(
        1.0 - decision.terrain.slope_deg / config_.critical_slope_deg,
        0.0,
        1.0);
    const double roughness_score = std::clamp(
        1.0 - observation.roughness_m / config_.critical_roughness_m,
        0.0,
        1.0);
    decision.terrain.traversability =
        decision.terrain.support_confidence *
        std::min(slope_score, roughness_score);

    const bool critical =
        !valid_normal ||
        has_fault(decision.terrain.header.faults, FaultFlag::SparseSupport) ||
        has_fault(decision.terrain.header.faults, FaultFlag::ExcessiveSlope) ||
        has_fault(
            decision.terrain.header.faults, FaultFlag::ExcessiveRoughness);
    const bool caution =
        decision.terrain.slope_deg >= config_.caution_slope_deg ||
        observation.roughness_m >= config_.caution_roughness_m;

    if (critical) {
        decision.risk = RiskLevel::Critical;
        decision.safety_mode = SafetyMode::HoldPosition;
    } else if (caution) {
        decision.risk = RiskLevel::Caution;
        decision.safety_mode = SafetyMode::ReduceSpeed;
    }
    return decision;
}

std::string_view TerrainAssessor::name(RiskLevel risk) noexcept {
    switch (risk) {
        case RiskLevel::Nominal:
            return "nominal";
        case RiskLevel::Caution:
            return "caution";
        case RiskLevel::Critical:
            return "critical";
    }
    return "unknown";
}

std::string_view TerrainAssessor::name(SafetyMode mode) noexcept {
    switch (mode) {
        case SafetyMode::Normal:
            return "normal";
        case SafetyMode::ReduceSpeed:
            return "reduce_speed";
        case SafetyMode::HoldPosition:
            return "hold_position";
    }
    return "unknown";
}

}  // namespace legged_terrain
