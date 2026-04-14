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
      "\n"
      "Planner parameters:\n"
      "  --lookahead_distance FLOAT    Waypoint lookahead distance (m)\n"
      "  --update_rate FLOAT           Plan/publish loop rate (Hz)\n"
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

  pct::TomogramConfig tomo_cfg;
  tomo_cfg.resolution = mod.arg_float("resolution", 0.3f);
  tomo_cfg.slice_dh = mod.arg_float("slice_dh", 0.5f);
  tomo_cfg.slope_max = mod.arg_float("slope_max", 0.40f);
  tomo_cfg.step_max = mod.arg_float("step_max", 0.3f);
  tomo_cfg.cost_barrier = mod.arg_float("cost_barrier", 50.0f);
  tomo_cfg.kernel_size = mod.arg_int("kernel_size", 5);
  tomo_cfg.safe_margin = mod.arg_float("safe_margin", 0.3f);
  tomo_cfg.inflation = mod.arg_float("inflation", 0.2f);
  tomo_cfg.interval_min = mod.arg_float("interval_min", 0.5f);
  tomo_cfg.interval_free = mod.arg_float("interval_free", 0.65f);
  tomo_cfg.standable_ratio = mod.arg_float("standable_ratio", 0.5f);

  pct::PlannerConfig planner_cfg;

  const double lookahead_dist = mod.arg_float("lookahead_distance", 2.0f);
  const float update_rate = mod.arg_float("update_rate", 5.0f);
  const std::string frame_id = mod.arg("frame_id", "map");

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

  const auto plan_period =
      std::chrono::milliseconds(static_cast<int>(1000.0f / update_rate));
  auto last_plan_time = std::chrono::steady_clock::now() - plan_period;
  const int drain_timeout_ms = 10;

  while (!g_shutdown.load()) {
    lcm.handleTimeout(drain_timeout_ms);

    const auto now = std::chrono::steady_clock::now();
    if (now - last_plan_time < plan_period) continue;
    last_plan_time = now;

    bool odom_ok, cloud_pending, goal_pending;
    Eigen::Vector3d robot_pos, goal;
    std::vector<float> cloud_xyz;
    {
      std::lock_guard<std::mutex> lk(state.mu);
      odom_ok = state.odom_init;
      cloud_pending = state.cloud_pending;
      goal_pending = state.goal_pending;
      robot_pos = state.robot_pos;
      goal = state.goal;
      if (cloud_pending) {
        cloud_xyz = std::move(state.cloud_xyz);
        state.cloud_pending = false;
      }
    }

    if (cloud_pending) {
      std::printf("[PCT] Rebuilding tomogram from %zu points\n",
                  cloud_xyz.size() / 3);
      planner.BuildTomogramFromCloud(cloud_xyz.data(), cloud_xyz.size() / 3);
      has_plan = false;
    }

    if (goal_pending && odom_ok && planner.has_tomogram()) {
      std::printf("[PCT] Planning to goal (%.2f,%.2f,%.2f)\n", goal.x(),
                  goal.y(), goal.z());
      Eigen::MatrixXd path = planner.Plan(robot_pos, goal);
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
          ps.pose.position.z = current_path(i, 2);
          ps.pose.orientation.w = 1.0;
        }
        lcm.publish(topic_path, &path_msg);
        std::printf("[PCT] Published path with %lld poses\n",
                    static_cast<long long>(current_path.rows()));
      } else {
        std::printf("[PCT] Plan failed\n");
      }
      {
        std::lock_guard<std::mutex> lk(state.mu);
        state.goal_pending = false;
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
        wp_msg.point.z = wp[2];
        lcm.publish(topic_wp, &wp_msg);
      }
    }
  }

  std::printf("[PCT] Shutdown complete.\n");
  return 0;
}
