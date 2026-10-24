// FSR MissionOrchestrator 的 Go2 适配节点。
// 把 demo_scan_bt_orchestrator.py 的核心编排语义迁到 C++ 编排层：
//   odom/cloud/bspline/waypoints → MissionOrchestrator::tick → MissionCommand
//   DispatchSubgoal → /goal_pose(PCT) → /global_path → 前视截取 → /initial_path(SCAN)
// Python 编排器继续作为默认兜底；本节点是 C++ 对齐验证路径。
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>

#include "mission/mission_orchestrator.h"

using namespace std::chrono_literals;
namespace fm = fork_spatial::mission;

class FsrMissionAdapter : public rclcpp::Node {
public:
    FsrMissionAdapter() : Node("fsr_mission_adapter") {
        ocfg_.subgoal_reach_m = declare_parameter("subgoal_reach_m", 0.45);
        ocfg_.reach_stable_speed_mps = declare_parameter("reach_stable_speed_mps", 0.08);
        ocfg_.min_execute_before_reach_s = declare_parameter("min_execute_before_reach_s", 1.5);
        ocfg_.progress_dist_delta_m = declare_parameter("progress_dist_delta_m", 0.12);
        ocfg_.stuck_timeout_s = declare_parameter("stuck_timeout_s", 60.0);
        ocfg_.stuck_redispatch_max = declare_parameter("stuck_redispatch_max", 3);
        ocfg_.replan_fail_threshold = declare_parameter("replan_fail_threshold", 8);
        ocfg_.plan_ready_timeout_s = declare_parameter("plan_ready_timeout_s", 8.0);
        ocfg_.ready_min_messages = declare_parameter("cloud_ready_min_messages", 10.0);
        ocfg_.ready_window_s = declare_parameter("cloud_ready_window_s", 4.0);
        ocfg_.ready_settle_s = declare_parameter("cloud_settle_s", 3.0);
        ocfg_.recovery_wait_s = declare_parameter("recovery_wait_s", 2.5);
        ocfg_.max_plan_fail_recovery = declare_parameter("max_plan_fail_recovery", 4);
        ocfg_.tick_hz = declare_parameter("tick_hz", 10.0);
        tick_s_ = 1.0 / ocfg_.tick_hz;
        max_subgoal_dist_m_ = declare_parameter("max_subgoal_dist_m", 8.0);
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        waypoints_file_ = declare_parameter<std::string>("waypoints_file", "");
        autostart_ = declare_parameter("autostart_lifecycle", true);

        orchestrator_ = std::make_unique<fm::MissionOrchestrator>(ocfg_);

        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        init_path_pub_ = create_publisher<nav_msgs::msg::Path>("/initial_path", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/demo/mission/status", 10);
        event_pub_ = create_publisher<std_msgs::msg::String>("/demo/mission/event", 10);
        lifecycle_pub_ = create_publisher<std_msgs::msg::String>("/demo/lifecycle/transition", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/lio/robo/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) { on_odom(*m); });
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lio/cloud_world", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr) { last_cloud_ = now(); });
        global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10, [this](nav_msgs::msg::Path::SharedPtr m) { on_global_path(*m); });
        bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
            "/planning/bspline", 10,
            [this](scan_planner_msgs::msg::Bspline::SharedPtr m) { on_bspline(*m); });
        start_sub_ = create_subscription<std_msgs::msg::String>(
            "/demo/mission/start", 10, [this](std_msgs::msg::String::SharedPtr) { start_mission(); });
        cancel_sub_ = create_subscription<std_msgs::msg::String>(
            "/demo/mission/cancel", 10,
            [this](std_msgs::msg::String::SharedPtr) { orchestrator_->cancel("external"); });

        timer_ = create_wall_timer(
            std::chrono::duration<double>(tick_s_), [this] { tick(); });
        lifecycle_state_ = "unconfigured";
        lifecycle_sub_ = create_subscription<std_msgs::msg::String>(
            "/demo/lifecycle/state", rclcpp::QoS(10).transient_local(),
            [this](std_msgs::msg::String::SharedPtr m) { lifecycle_state_ = m->data; });

        if (autostart_) {
            start_pub_ = create_publisher<std_msgs::msg::String>("/demo/mission/start", 10);
            boot_timer_ = create_wall_timer(1.0s, [this] {
                publish_str(lifecycle_pub_, "configure");
                publish_str(lifecycle_pub_, "activate");
                boot_timer_->cancel();
            });
            start_timer_ = create_wall_timer(2.0s, [this] {
                publish_str(start_pub_, "start");
                start_timer_->cancel();
            });
        }
        RCLCPP_INFO(get_logger(), "fsr_mission_adapter ready (tick=%.1fHz)", ocfg_.tick_hz);
    }

private:
    static void publish_str(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub,
                            const std::string& s) {
        if (!pub) return;
        std_msgs::msg::String m;
        m.data = s;
        pub->publish(m);
    }

    void start_mission() {
        if (mission_started_) return;
        fm::MissionRequest req;
        if (!load_waypoints(req)) {
            RCLCPP_ERROR(get_logger(), "waypoints load failed: %s", waypoints_file_.c_str());
            emit_event("mission_rejected", "no_waypoints");
            return;
        }
        mission_started_ = orchestrator_->submit(req);
        RCLCPP_INFO(get_logger(), "mission submitted: %zu subgoals", req.subgoals.size());
    }

    bool load_waypoints(fm::MissionRequest& req) {
        try {
            YAML::Node doc = YAML::LoadFile(waypoints_file_);
            const YAML::Node params =
                doc["demo_scan_waypoints"] ? doc["demo_scan_waypoints"]["ros__parameters"]
                                           : doc["ros__parameters"];
            for (const auto& wp : params["waypoints"]) {
                fm::Subgoal sg;
                sg.x = wp["x"].as<double>();
                sg.y = wp["y"].as<double>();
                sg.z = wp["z"].as<double>();
                sg.frame_id = frame_id_;
                sg.reach_tolerance_m = ocfg_.subgoal_reach_m;
                req.subgoals.push_back(sg);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "yaml parse: %s", e.what());
            return false;
        }
        return !req.subgoals.empty();
    }

    void on_odom(const nav_msgs::msg::Odometry& m) {
        have_odom_ = true;
        last_odom_ = now();
        const auto& p = m.pose.pose.position;
        const auto& v = m.twist.twist.linear;
        state_.x = p.x;
        state_.y = p.y;
        state_.z = p.z;
        state_.speed_mps = std::hypot(v.x, v.y, v.z);
    }

    void on_global_path(const nav_msgs::msg::Path& m) {
        if (!awaiting_path_) return;
        if (request_stamp_.nanoseconds() > 0 && m.header.stamp.nanosec > 0 &&
            rclcpp::Time(m.header.stamp) < request_stamp_) {
            RCLCPP_WARN(get_logger(), "stale /global_path ignored");
            return;
        }
        if (m.poses.size() < 2) {
            emit_event("pct_path_failed", "empty_or_short_path");
            orchestrator_->on_plan_failed();
            awaiting_path_ = false;
            return;
        }
        awaiting_path_ = false;
        emit_event("pct_path_received");
        auto scan = build_scan_path(m);
        if (scan.poses.size() >= 2) {
            init_path_pub_->publish(scan);
            RCLCPP_INFO(get_logger(), "SCAN initial_path poses=%zu (fsr)", scan.poses.size());
        }
    }

    void on_bspline(const scan_planner_msgs::msg::Bspline& m) {
        const bool emergency = m.pos_pts.size() < 2 ||
            std::all_of(m.pos_pts.begin(), m.pos_pts.end(), [&](const auto& p) {
                const auto& f = m.pos_pts.front();
                return std::abs(p.x - f.x) < 1e-3 && std::abs(p.y - f.y) < 1e-3 &&
                       std::abs(p.z - f.z) < 1e-3;
            });
        if (emergency) {
            emit_event("emergency_bspline");
            return;
        }
        if (static_cast<int64_t>(m.traj_id) <= traj_id_at_dispatch_) return;
        if (dispatch_stamp_.nanoseconds() > 0 &&
            rclcpp::Time(m.start_time) < dispatch_stamp_)
            return;
        planner_ready_ = true;
        orchestrator_->on_planner_ready();
        emit_event("scan_planner_ready");
    }

    nav_msgs::msg::Path build_scan_path(const nav_msgs::msg::Path& src) {
        // 对齐 _copy_path_for_scan：近端前缀 + densify(0.2m) + 去共线 + 丢贴脸首点
        std::vector<std::pair<double, double>> raw;
        if (have_odom_ && !src.poses.empty()) {
            const auto& h = src.poses.front().pose.position;
            if (std::hypot(state_.x - h.x, state_.y - h.y) > 0.6) {
                raw.emplace_back(state_.x, state_.y);
            }
        }
        for (const auto& ps : src.poses) {
            raw.emplace_back(ps.pose.position.x, ps.pose.position.y);
        }
        auto dense = decimate_collinear(densify(raw, 0.20));
        while (dense.size() > 1 &&
               std::hypot(dense[0].first - state_.x, dense[0].second - state_.y) < 0.04) {
            dense.erase(dense.begin());
        }
        nav_msgs::msg::Path out;
        out.header.stamp = now();
        out.header.frame_id = frame_id_;
        for (const auto& [x, y] : dense) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = out.header;
            ps.pose.position.x = x;
            ps.pose.position.y = y;
            ps.pose.orientation.w = 1.0;
            out.poses.push_back(ps);
        }
        return out;
    }

    static std::vector<std::pair<double, double>> densify(
        const std::vector<std::pair<double, double>>& pts, double step) {
        std::vector<std::pair<double, double>> out;
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
            const auto [x0, y0] = pts[i];
            const auto [x1, y1] = pts[i + 1];
            const double d = std::hypot(x1 - x0, y1 - y0);
            const int n = std::max(1, static_cast<int>(std::ceil(d / step)));
            for (int k = 0; k < n; ++k) {
                const double t = static_cast<double>(k) / n;
                out.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
            }
        }
        if (!pts.empty()) out.push_back(pts.back());
        return out;
    }

    static std::vector<std::pair<double, double>> decimate_collinear(
        const std::vector<std::pair<double, double>>& pts) {
        if (pts.size() <= 2) return pts;
        std::vector<std::pair<double, double>> out{pts.front()};
        for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
            const auto& a = out.back();
            const auto& b = pts[i];
            const auto& c = pts[i + 1];
            const double ux = b.first - a.first, uy = b.second - a.second;
            const double vx = c.first - b.first, vy = c.second - b.second;
            const double cross = std::abs(ux * vy - uy * vx);
            const double lenu = std::hypot(ux, uy), lenv = std::hypot(vx, vy);
            if (cross > 1e-4 * lenu * lenv || lenu > 1e-9 || lenv > 1e-9) {
                out.push_back(b);
            }
        }
        out.push_back(pts.back());
        return out;
    }

    void emit_event(const std::string& ev, const std::string& detail = "") {
        std_msgs::msg::String m;
        m.data = "{\"event\":\"" + ev + "\",\"adapter\":\"fsr\",\"subgoal_index\":" +
                 std::to_string(orchestrator_->current_subgoal()) +
                 (detail.empty() ? "" : ",\"detail\":\"" + detail + "\"") + "}";
        event_pub_->publish(m);
        std_msgs::msg::String s;
        s.data = "{\"event\":\"" + ev + "\",\"adapter\":\"fsr\",\"status\":\"" +
                 status_name() + "\",\"phase\":\"" +
                 (orchestrator_->status() == fm::MissionStatus::Running ? "execute" : "idle") +
                 "\",\"subgoal_index\":" +
                 std::to_string(orchestrator_->current_subgoal()) + "}";
        status_pub_->publish(s);
    }

    const char* status_name() const {
        switch (orchestrator_->status()) {
            case fm::MissionStatus::Idle: return "idle";
            case fm::MissionStatus::WaitReady: return "wait_ready";
            case fm::MissionStatus::Running: return "running";
            case fm::MissionStatus::Recovering: return "recovering";
            case fm::MissionStatus::Succeeded: return "succeeded";
            case fm::MissionStatus::Failed: return "failed";
            case fm::MissionStatus::Canceled: return "canceled";
        }
        return "unknown";
    }

    void tick() {
        if (!mission_started_) return;
        const auto t_now = now();
        const bool state_fresh = have_odom_ && (t_now - last_odom_).seconds() < 0.5;
        const bool cloud_fresh = last_cloud_.nanoseconds() > 0 &&
                                 (t_now - last_cloud_).seconds() < 5.0;
        state_.stamp_ns = t_now.nanoseconds();
        state_.localization_valid = state_fresh && cloud_fresh;
        const auto cmd = orchestrator_->tick(state_, planner_ready_);
        switch (cmd.type) {
            case fm::CommandType::DispatchSubgoal: {
                geometry_msgs::msg::PoseStamped goal;
                goal.header.stamp = now();
                goal.header.frame_id = frame_id_;
                goal.pose.position.x = cmd.target.x;
                goal.pose.position.y = cmd.target.y;
                goal.pose.position.z = cmd.target.z;
                goal.pose.orientation.w = 1.0;
                request_stamp_ = goal.header.stamp;
                dispatch_stamp_ = goal.header.stamp;
                traj_id_at_dispatch_ = last_traj_id_;
                awaiting_path_ = true;
                planner_ready_ = false;
                goal_pub_->publish(goal);
                emit_event("subgoal_dispatched");
                break;
            }
            case fm::CommandType::StopTrajectory:
                emit_event("stop_trajectory", cmd.reason);
                break;
            case fm::CommandType::RecoveryWait:
                emit_event("recovery_wait", cmd.reason);
                break;
            case fm::CommandType::MissionTerminal:
                emit_event(cmd.reason.rfind("succeeded", 0) == 0 ? "mission_succeeded"
                                                                 : "mission_terminal",
                           cmd.reason);
                break;
            case fm::CommandType::SlowModeOn:
            case fm::CommandType::SlowModeOff:
            case fm::CommandType::None:
                break;
        }
    }

    fm::OrchestratorConfig ocfg_;
    std::unique_ptr<fm::MissionOrchestrator> orchestrator_;
    fm::RobotState state_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr init_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, event_pub_,
        lifecycle_pub_, start_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_, cancel_sub_, lifecycle_sub_;
    rclcpp::TimerBase::SharedPtr timer_, boot_timer_, start_timer_;

    bool mission_started_{false};
    bool have_odom_{false};
    bool planner_ready_{false};
    bool awaiting_path_{false};
    bool autostart_{false};
    int64_t traj_id_at_dispatch_{-1};
    int64_t last_traj_id_{0};
    rclcpp::Time request_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time dispatch_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_cloud_{0, 0, RCL_ROS_TIME};
    std::string waypoints_file_, frame_id_, lifecycle_state_;
    double tick_s_{0.1};
    double max_subgoal_dist_m_{8.0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FsrMissionAdapter>());
    rclcpp::shutdown();
    return 0;
}
