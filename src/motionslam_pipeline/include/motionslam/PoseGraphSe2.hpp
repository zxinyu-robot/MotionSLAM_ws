#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace motionslam {

struct Se2Pose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline Se2Pose Se2Inverse(const Se2Pose& a) {
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  Se2Pose out;
  out.x = -(c * a.x + s * a.y);
  out.y = -(-s * a.x + c * a.y);
  out.yaw = -a.yaw;
  return out;
}

inline Se2Pose Se2Compose(const Se2Pose& a, const Se2Pose& b) {
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  Se2Pose out;
  out.x = a.x + c * b.x - s * b.y;
  out.y = a.y + s * b.x + c * b.y;
  out.yaw = a.yaw + b.yaw;
  while (out.yaw > M_PI) {
    out.yaw -= 2.0 * M_PI;
  }
  while (out.yaw <= -M_PI) {
    out.yaw += 2.0 * M_PI;
  }
  return out;
}

inline Se2Pose Se2Between(const Se2Pose& from, const Se2Pose& to) {
  return Se2Compose(Se2Inverse(from), to);
}

struct Se2Edge {
  std::size_t i{0};
  std::size_t j{0};
  Se2Pose meas;  // T_j in i frame: p_i + R_i * meas = p_j approx
  double trans_weight{100.0};
  double rot_weight{200.0};
  bool is_loop{false};
};

// Fix node 0; Gauss-Newton on SE(2) for small graphs (mapping-scale).
inline void OptimizePoseGraph(std::vector<Se2Pose>& nodes,
                              const std::vector<Se2Edge>& edges,
                              int max_iterations = 20) {
  if (nodes.size() < 2 || edges.empty()) {
    return;
  }
  const std::size_t n = nodes.size();
  for (int iter = 0; iter < max_iterations; ++iter) {
    std::vector<double> dx(3 * n, 0.0);
    std::vector<double> diag(3 * n, 0.0);
    for (const auto& e : edges) {
      if (e.i >= n || e.j >= n) {
        continue;
      }
      const Se2Pose& pi = nodes[e.i];
      const Se2Pose& pj = nodes[e.j];
      const Se2Pose pred = Se2Compose(
          Se2Inverse(pi),
          pj);  // T_i_j predicted from current estimates
      const double ex = e.meas.x - pred.x;
      const double ey = e.meas.y - pred.y;
      double eyaw = e.meas.yaw - pred.yaw;
      while (eyaw > M_PI) {
        eyaw -= 2.0 * M_PI;
      }
      while (eyaw <= -M_PI) {
        eyaw += 2.0 * M_PI;
      }
      const double w_t = e.trans_weight;
      const double w_r = e.rot_weight;
      const int ii = static_cast<int>(e.i);
      const int jj = static_cast<int>(e.j);
      dx[3 * ii + 0] -= w_t * ex;
      dx[3 * ii + 1] -= w_t * ey;
      dx[3 * ii + 2] -= w_r * eyaw;
      dx[3 * jj + 0] += w_t * ex;
      dx[3 * jj + 1] += w_t * ey;
      dx[3 * jj + 2] += w_r * eyaw;
      diag[3 * ii + 0] += w_t;
      diag[3 * ii + 1] += w_t;
      diag[3 * ii + 2] += w_r;
      diag[3 * jj + 0] += w_t;
      diag[3 * jj + 1] += w_t;
      diag[3 * jj + 2] += w_r;
    }
    // Fix first node
    dx[0] = dx[1] = dx[2] = 0.0;
    diag[0] = diag[1] = diag[2] = 1.0;
    double max_step = 0.0;
    for (std::size_t k = 1; k < n; ++k) {
      for (int c = 0; c < 3; ++c) {
        const double d = diag[3 * k + c];
        if (d < 1e-6) {
          continue;
        }
        const double step = dx[3 * k + c] / d;
        max_step = std::max(max_step, std::abs(step));
        if (c == 0) {
          nodes[k].x += step;
        } else if (c == 1) {
          nodes[k].y += step;
        } else {
          nodes[k].yaw += step;
          while (nodes[k].yaw > M_PI) {
            nodes[k].yaw -= 2.0 * M_PI;
          }
          while (nodes[k].yaw <= -M_PI) {
            nodes[k].yaw += 2.0 * M_PI;
          }
        }
      }
    }
    if (max_step < 1e-4) {
      break;
    }
  }
}

}  // namespace motionslam
