// PCT (Point Cloud Tomography) planner — LCM native module main loop.
//
// Subscribes to: odometry (Odometry), explored_areas (PointCloud2),
//                goal (PointStamped).
// Publishes to:  way_point (PointStamped), goal_path (Path),
//                tomogram (PointCloud2) for visualization.
//
// Behavior:
//   - On explored_areas: rebuild the tomogram from the accumulated cloud and
//     re-init the ele_planner. Flags the current plan stale.
//   - On goal: record the new goal, clear the current plan.
//   - At update_rate: if a goal is pending and a tomogram is available,
//     replan from current odom. Publish the next lookahead waypoint every
//     cycle based on the most recent plan.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "nav_msgs/Odometry.hpp"
#include "nav_msgs/Path.hpp"
#include "geometry_msgs/PointStamped.hpp"
#include "geometry_msgs/PoseStamped.hpp"

#include "planner/path_utils.h"
#include "planner/pct_planner.h"
#include "tomography/tomogram.h"

namespace {

std::atomic<bool> g_shutdown{false};

void signal_handler(int) { g_shutdown.store(true); }

struct LcmState {
  std::mutex mu;
  bool odom_init = false;
  Eigen::Vector3d robot_pos = Eigen::Vector3d::Zero();

  bool goal_pending = false;
  Eigen::Vector3d goal = Eigen::Vector3d::Zero();

  bool cloud_pending = false;
  std::vector<float> cloud_xyz;  // flattened [x0,y0,z0,x1,...]
};

struct Handlers {
  LcmState* state;
  std::string world_frame;

  void on_odom(const lcm::ReceiveBuffer*, const std::string&,
               const nav_msgs::Odometry* msg) {
    std::lock_guard<std::mutex> lk(state->mu);
    state->robot_pos = Eigen::Vector3d(msg->pose.pose.position.x,
                                       msg->pose.pose.position.y,
                                       msg->pose.pose.position.z);
    state->odom_init = true;
  }

  void on_explored_areas(const lcm::ReceiveBuffer*, const std::string&,
                         const sensor_msgs::PointCloud2* msg) {
    auto points = smartnav::parse_pointcloud2(*msg);
    if (points.empty()) return;
    std::vector<float> xyz;
    xyz.reserve(points.size() * 3);
    for (const auto& p : points) {
      xyz.push_back(p.x);
      xyz.push_back(p.y);
      xyz.push_back(p.z);
    }
    std::lock_guard<std::mutex> lk(state->mu);
    state->cloud_xyz = std::move(xyz);
    state->cloud_pending = true;
  }

  void on_goal(const lcm::ReceiveBuffer*, const std::string&,
               const geometry_msgs::PointStamped* msg) {
    std::lock_guard<std::mutex> lk(state->mu);
    state->goal = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    state->goal_pending = true;
  }
};

void print_help(const char* argv0) {
  std::printf(
      "Usage: %s [--KEY VALUE]...\n"
      "\n"
      "LCM topics (required):\n"
      "  --odometry CHANNEL            Odometry input (nav_msgs::Odometry)\n"
      "  --explored_areas CHANNEL      Explored-area point cloud input\n"
      "  --goal CHANNEL                Goal input (geometry_msgs::PointStamped)\n"
      "  --way_point CHANNEL           Waypoint output\n"
      "  --goal_path CHANNEL           Full planned path output\n"
      "  --tomogram CHANNEL            Tomogram visualization output\n"
      "\n"
      "Tomogram parameters:\n"
      "  --resolution FLOAT            Grid resolution (m)\n"
      "  --slice_dh FLOAT              Slice spacing (m)\n"
      "  --slope_max FLOAT             Max stand-on slope (rad)\n"
      "  --step_max FLOAT              Max step-over height (m)\n"
      "  --cost_barrier FLOAT          Traversability cost for blocked cells\n"
      "  --kernel_size INT             Traversability kernel size (odd)\n"
      "  --safe_margin FLOAT           Inflation safe margin (m)\n"
      "  --inflation FLOAT             Inflation radius (m)\n"
      "  --interval_min FLOAT          Minimum vertical clearance (m)\n"
      "  --interval_free FLOAT         Free-space interval threshold (m)\n"
      "  --standable_ratio FLOAT       Standable neighbor fraction [0,1]\n"
      "\n"
      "Planner parameters:\n"
      "  --lookahead_distance FLOAT    Waypoint lookahead distance (m)\n"
      "  --update_rate FLOAT           Plan/publish loop rate (Hz)\n"
      "  --tomogram_rebuild_period_sec FLOAT  Minimum seconds between tomogram\n"
      "                                rebuilds (0 = rebuild every iteration).\n"
      "                                Rebuilds are expensive so the default of\n"
      "                                1.0 s caps the cost while still tracking\n"
      "                                exploration. Required because our live\n"
      "                                `explored_areas` publish rate (~10 Hz)\n"
      "                                is much faster than the ~1 s rebuild.\n"
      "  --min_plan_half_extent_m FLOAT Superset: expand the tomogram grid to\n"
      "                                include at least a square of this half-\n"
      "                                extent around the robot, even if the\n"
      "                                observed cloud is smaller. 0 = strict\n"
      "                                upstream behavior (grid sized to cloud\n"
      "                                bbox + 4 cells). Required for live maps\n"
      "                                that start small and grow.\n"
      "  --frame_id STRING             World frame id for published headers\n"
      "  --help                        Print this help and exit\n",
      argv0);
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGTERM, signal_handler);
  std::signal(SIGINT, signal_handler);
  std::setvbuf(stdout, nullptr, _IOLBF, 0);

  for (int i = 1; i < argc; ++i) {
    const std::string a(argv[i]);
    if (a == "--help" || a == "-h") {
      print_help(argv[0]);
      return 0;
    }
  }

  dimos::NativeModule mod(argc, argv);

  // Defaults mirror the upstream `pct_planner_params.yaml`. The Python
  // `PCTPlannerConfig` always emits every field via `to_cli_args()`, so
  // these fallbacks only matter for standalone invocations.
  pct::TomogramConfig tomo_cfg;
  double lookahead_dist = 1.25;
  float update_rate = 5.0f;
  float tomogram_rebuild_period_sec = 1.0f;
  float min_plan_half_extent_m = 15.0f;
  std::string frame_id = "map";
  pct::PlannerConfig planner_cfg;
  try {
    tomo_cfg.resolution = mod.arg_float("resolution", 0.075f);
    tomo_cfg.slice_dh = mod.arg_float("slice_dh", 0.4f);
    tomo_cfg.slope_max = mod.arg_float("slope_max", 0.45f);
    tomo_cfg.step_max = mod.arg_float("step_max", 0.5f);
    tomo_cfg.cost_barrier = mod.arg_float("cost_barrier", 100.0f);
    tomo_cfg.kernel_size = mod.arg_int("kernel_size", 11);
    tomo_cfg.safe_margin = mod.arg_float("safe_margin", 0.025f);
    tomo_cfg.inflation = mod.arg_float("inflation", 0.05f);
    tomo_cfg.interval_min = mod.arg_float("interval_min", 0.3f);
    tomo_cfg.interval_free = mod.arg_float("interval_free", 0.5f);
    tomo_cfg.standable_ratio = mod.arg_float("standable_ratio", 0.02f);

    planner_cfg.astar_cost_threshold = mod.arg_float(
        "astar_cost_threshold", static_cast<float>(planner_cfg.astar_cost_threshold));
    planner_cfg.safe_cost_margin = mod.arg_float(
        "safe_cost_margin", static_cast<float>(planner_cfg.safe_cost_margin));
    planner_cfg.max_heading_rate = mod.arg_float(
        "max_heading_rate", static_cast<float>(planner_cfg.max_heading_rate));
    planner_cfg.use_quintic = mod.arg_bool("use_quintic", planner_cfg.use_quintic);

    lookahead_dist = mod.arg_float("lookahead_distance", 1.25f);
    update_rate = mod.arg_float("update_rate", 5.0f);
    tomogram_rebuild_period_sec =
        mod.arg_float("tomogram_rebuild_period_sec", 1.0f);
    min_plan_half_extent_m = mod.arg_float("min_plan_half_extent_m", 15.0f);
    frame_id = mod.arg("frame_id", "map");
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[PCT] ERROR: failed to parse CLI args: %s\n", e.what());
    return 1;
  }

  pct::TomogramPlanner planner(planner_cfg, tomo_cfg);

  lcm::LCM lcm;
  if (!lcm.good()) {
    std::fprintf(stderr, "[PCT] ERROR: LCM init failed\n");
    return 1;
  }

  LcmState state;
  Handlers handlers{&state, frame_id};
  const std::string topic_odom = mod.topic("odometry");
  const std::string topic_cloud = mod.topic("explored_areas");
  const std::string topic_goal = mod.topic("goal");
  const std::string topic_wp = mod.topic("way_point");
  const std::string topic_path = mod.topic("goal_path");
  const std::string topic_tomo = mod.topic("tomogram");

  lcm.subscribe(topic_odom, &Handlers::on_odom, &handlers);
  lcm.subscribe(topic_cloud, &Handlers::on_explored_areas, &handlers);
  lcm.subscribe(topic_goal, &Handlers::on_goal, &handlers);

  std::printf("[PCT] Subscribed odom=%s cloud=%s goal=%s\n",
              topic_odom.c_str(), topic_cloud.c_str(), topic_goal.c_str());
  std::printf("[PCT] Publishing wp=%s path=%s tomo=%s\n", topic_wp.c_str(),
              topic_path.c_str(), topic_tomo.c_str());
  std::printf("[PCT] Config: resolution=%.3f slice_dh=%.2f lookahead=%.2f rate=%.1fHz\n",
              tomo_cfg.resolution, tomo_cfg.slice_dh, lookahead_dist, update_rate);

  Eigen::MatrixXd current_path;
  bool has_plan = false;
  bool have_goal = false;
  Eigen::Vector3d active_goal = Eigen::Vector3d::Zero();
  // Throttle tomogram rebuilds — PreloadedMapTracker publishes
  // explored_areas at ~10 Hz but rebuilding the grid is ~1 s, so we cap
  // the rebuild rate from the CLI. Set `--tomogram_rebuild_period_sec 0`
  // to rebuild on every iteration (matches upstream behavior when the
  // tomogram is loaded from a static pickle file).
  const auto tomogram_rebuild_period =
      std::chrono::milliseconds(static_cast<int>(
          std::max(0.0f, tomogram_rebuild_period_sec) * 1000.0f));
  auto last_tomogram_rebuild =
      std::chrono::steady_clock::now() - tomogram_rebuild_period;

  const auto plan_period =
      std::chrono::milliseconds(static_cast<int>(1000.0f / update_rate));
  auto last_plan_time = std::chrono::steady_clock::now() - plan_period;
  const int drain_timeout_ms = 10;

  while (!g_shutdown.load()) {
    lcm.handleTimeout(drain_timeout_ms);

    const auto now = std::chrono::steady_clock::now();
    if (now - last_plan_time < plan_period) continue;
    last_plan_time = now;

    // Decide up front whether this iteration will consume a pending cloud.
    // Rebuilds are ~1 s each, so we throttle them; if the throttle says
    // "not yet", leave the cloud in state for the next iteration instead
    // of silently dropping it.
    const bool may_rebuild_tomogram =
        !planner.has_tomogram() ||
        (std::chrono::steady_clock::now() - last_tomogram_rebuild) >=
            tomogram_rebuild_period;

    bool odom_ok, consumed_cloud, consumed_goal;
    Eigen::Vector3d robot_pos, goal;
    std::vector<float> cloud_xyz;
    {
      std::lock_guard<std::mutex> lk(state.mu);
      odom_ok = state.odom_init;
      robot_pos = state.robot_pos;
      goal = state.goal;
      // Consume the goal atomically — clearing goal_pending here prevents
      // a new goal that lands during Plan() from being lost by a later
      // unconditional clear.
      consumed_goal = state.goal_pending;
      if (consumed_goal) state.goal_pending = false;
      // Only move the cloud out of state if we intend to rebuild this
      // iteration; otherwise the next iteration will see it again.
      consumed_cloud = state.cloud_pending && may_rebuild_tomogram;
      if (consumed_cloud) {
        cloud_xyz = std::move(state.cloud_xyz);
        state.cloud_pending = false;
      }
    }

    if (consumed_cloud) {
      std::printf("[PCT] Rebuilding tomogram from %zu points\n",
                  cloud_xyz.size() / 3);
      planner.BuildTomogramFromCloud(cloud_xyz.data(),
                                     cloud_xyz.size() / 3, robot_pos,
                                     min_plan_half_extent_m);
      last_tomogram_rebuild = std::chrono::steady_clock::now();
      has_plan = false;
    }

    if (consumed_goal) {
      active_goal = goal;
      have_goal = true;
      has_plan = false;
    }

    const bool need_replan =
        have_goal && !has_plan && odom_ok && planner.has_tomogram();
    if (need_replan) {
      std::printf("[PCT] Planning to goal (%.2f,%.2f,%.2f)\n",
                  active_goal.x(), active_goal.y(), active_goal.z());
      Eigen::MatrixXd path = planner.Plan(robot_pos, active_goal);
      if (path.rows() > 0) {
        current_path = std::move(path);
        has_plan = true;

        nav_msgs::Path path_msg;
        const double ts =
            std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        path_msg.header = dimos::make_header(frame_id, ts);
        path_msg.poses_length = static_cast<int32_t>(current_path.rows());
        path_msg.poses.resize(current_path.rows());
        for (int i = 0; i < current_path.rows(); ++i) {
          auto& ps = path_msg.poses[i];
          ps.header = path_msg.header;
          ps.pose.position.x = current_path(i, 0);
          ps.pose.position.y = current_path(i, 1);
          // Publish the real planned z so multi-floor visualizers see the
          // correct 3D path. Only the waypoint (below) is flattened to
          // robot z for the local planner's terrain-band filter.
          ps.pose.position.z = current_path(i, 2);
          ps.pose.orientation.w = 1.0;
        }
        lcm.publish(topic_path, &path_msg);
        std::printf(
            "[PCT] Published path with %lld poses: start=(%.2f,%.2f) "
            "end=(%.2f,%.2f) goal=(%.2f,%.2f) robot=(%.2f,%.2f)\n",
            static_cast<long long>(current_path.rows()),
            current_path(0, 0), current_path(0, 1),
            current_path(current_path.rows() - 1, 0),
            current_path(current_path.rows() - 1, 1),
            active_goal.x(), active_goal.y(), robot_pos.x(), robot_pos.y());
      } else {
        std::printf("[PCT] Plan failed\n");
      }
    }

    if (has_plan && odom_ok) {
      double wp[3];
      if (pct::GetWaypointFromTraj(current_path, robot_pos, lookahead_dist,
                                   wp)) {
        geometry_msgs::PointStamped wp_msg;
        const double ts =
            std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        wp_msg.header = dimos::make_header(frame_id, ts);
        wp_msg.point.x = wp[0];
        wp_msg.point.y = wp[1];
        // Publish the planned z. For multi-floor scenes this carries
        // the target floor's height; the local planner's terrain-band
        // filter is compared against the robot's own pose, and
        // reasonable stairs/ramps (gradient ≤ max_relative_z per
        // `lookahead_distance`) stay inside the band. If a waypoint at
        // the target z lies outside the local planner's `max_relative_z`
        // window, that's a genuine "cannot reach this floor directly"
        // signal rather than something to hide by flattening z here.
        wp_msg.point.z = wp[2];
        lcm.publish(topic_wp, &wp_msg);
      }
    }
  }

  std::printf("[PCT] Shutdown complete.\n");
  return 0;
}
