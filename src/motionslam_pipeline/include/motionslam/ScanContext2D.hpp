#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace motionslam {

// Lightweight 2D Scan Context: ring x sector max-height bins (body frame, z slice).
class ScanContext2D {
 public:
  ScanContext2D(int num_rings = 20, int num_sectors = 60, double max_range = 40.0)
      : num_rings_(num_rings),
        num_sectors_(num_sectors),
        max_range_(max_range) {
    desc_.assign(static_cast<std::size_t>(num_rings_ * num_sectors_), 0.0);
  }

  void Build(const std::vector<float>& x, const std::vector<float>& y,
             const std::vector<float>& z, double z_min, double z_max) {
    std::fill(desc_.begin(), desc_.end(), 0.0);
    for (std::size_t i = 0; i < x.size(); ++i) {
      if (z[i] < z_min || z[i] > z_max) {
        continue;
      }
      const double r = std::hypot(x[i], y[i]);
      if (r < 0.5 || r > max_range_) {
        continue;
      }
      const double theta =
          std::atan2(y[i], x[i]) + M_PI;  // [0, 2pi)
      const int ring = std::min(
          num_rings_ - 1,
          static_cast<int>(r / max_range_ * num_rings_));
      const int sector = std::min(
          num_sectors_ - 1,
          static_cast<int>(theta / (2.0 * M_PI) * num_sectors_));
      const std::size_t idx =
          static_cast<std::size_t>(ring * num_sectors_ + sector);
      desc_[idx] = std::max(desc_[idx], static_cast<double>(z[i]));
    }
    // L2 normalize
    double norm = 0.0;
    for (double v : desc_) {
      norm += v * v;
    }
    norm = std::sqrt(std::max(norm, 1e-12));
    for (double& v : desc_) {
      v /= norm;
    }
  }

  double Distance(const ScanContext2D& other, int max_shift = 6) const {
    double best = 1e9;
    for (int shift = -max_shift; shift <= max_shift; ++shift) {
      double dist = 0.0;
      for (int s = 0; s < num_sectors_; ++s) {
        const int s2 = (s + shift + num_sectors_ * 10) % num_sectors_;
        for (int r = 0; r < num_rings_; ++r) {
          const double a = desc_[static_cast<std::size_t>(r * num_sectors_ + s)];
          const double b =
              other.desc_[static_cast<std::size_t>(r * num_sectors_ + s2)];
          dist += (a - b) * (a - b);
        }
      }
      best = std::min(best, dist);
    }
    return best;
  }

 private:
  int num_rings_;
  int num_sectors_;
  double max_range_;
  std::vector<double> desc_;
};

}  // namespace motionslam
