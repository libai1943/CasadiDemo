#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <fstream>

#include "common/math/vec2d.h"
#include "common/math/circle_2d.h"
#include "common/util/time.h"
#include "hybrid_astar/vehicle_parameter.h"

#include "visualization_plot.h"

#include "interface.h"

ParkingPlanner::ParkingPlanner(): nh_("~/parking_planner") {
  env_ = std::make_shared<Environment>();
  ReadConfig();
}

void ParkingPlanner::ReadConfig() {
  // vehicle parameters
  nh_.param("back_edge_to_center", env_->vehicle.back_edge_to_center, 1.9);
  nh_.param("wheel_base", env_->vehicle.wheel_base, 5.73);
  nh_.param("length", env_->vehicle.length, 9.34);
  nh_.param("width", env_->vehicle.width, 3.5);
  nh_.param("max_acceleration", env_->vehicle.max_acceleration, 0.2);
  nh_.param("max_deceleration", env_->vehicle.max_deceleration, -0.2);
  nh_.param("max_velocity", env_->vehicle.max_velocity, 2.0);
  nh_.param("max_reverse_velocity", env_->vehicle.max_reverse_velocity, -1.0);
  nh_.param("max_steer_angle", env_->vehicle.max_steer_angle, 0.49);
  nh_.param("max_steer_angle_rate", env_->vehicle.max_steer_angle_rate, 0.14);
  nh_.param("steer_ratio", env_->vehicle.steer_ratio, 1.0);
  env_->vehicle.GenerateDiscs();

  // hybrid A* parameters
  nh_.param("xy_grid_resolution", parking_config_.xy_grid_resolution, 0.6);
  nh_.param("phi_grid_resolution", parking_config_.phi_grid_resolution, 0.1);
  parking_config_.next_node_num = nh_.param("next_node_num", 6);
  nh_.param("step_size", parking_config_.step_size, 0.5);
  nh_.param("traj_forward_penalty", parking_config_.traj_forward_penalty, 0.5);
  nh_.param("traj_back_penalty", parking_config_.traj_back_penalty, 1.1);
  nh_.param("traj_gear_switch_penalty", parking_config_.traj_gear_switch_penalty, 8.0);
  nh_.param("traj_steer_penalty", parking_config_.traj_steer_penalty, 1.0);
  nh_.param("traj_steer_change_penalty", parking_config_.traj_steer_change_penalty, 2.0);
  nh_.param("grid_a_star_xy_resolution", parking_config_.grid_a_star_xy_resolution, 0.5);
  nh_.param("analytic_expansion_cost", parking_config_.analytic_expansion_cost, 20.0);
  nh_.param("dubins_phi_gamma", parking_config_.dubins_phi_gamma, 1.1);
  parking_config_.explore_node_num = nh_.param("explore_node_num", 100000);

  // optimization parameters
  nh_.param("nfe", config_.nfe, 100);
  nh_.param("tf_max", config_.tf_max, 500.0);
  nh_.param("corridor_max_iter", config_.corridor_max_iter, 1000);
  nh_.param("corridor_incremental_limit", config_.corridor_incremental_limit, 20.0);

  nh_.param("opti_omega", config_.opti_omega, 5.0);
  nh_.param("opti_a", config_.opti_a, 1.0);
  nh_.param("opti_phi", config_.opti_phi, 1.0);

  nh_.param("opti_iter_max", config_.opti_iter_max, 100);
  nh_.param("opti_w_penalty0", config_.opti_w_penalty0, 1e3);
  nh_.param("opti_alpha", config_.opti_alpha, 3.0);
  nh_.param("opti_w_penalty_max", config_.opti_w_penalty_max, 1e7);
  nh_.param("opti_varepsilon_tol", config_.opti_varepsilon_tol, 1e-6);
  nh_.param("opti_solution_norm_tol", config_.opti_solution_norm_tol, 1e-1);
}

bool ParkingPlanner::Plan(Trajectory *result, int parking_type) {
  MyEnvironment my_env;
  my_env.points = env_->points.points();  // Get boundary points from PointCloud
  my_env.obstacles = env_->obstacles;
  my_env.start = start_pose_;
  my_env.goal = goal_pose_;

  // Log boundary and obstacle information
  ROS_INFO("Planning with %lu boundary points and %lu obstacles",
           my_env.points.size(), my_env.obstacles.size());
  // Convert VehicleParam to VehicleParameter
  VehicleParameter vehicle_param;
  vehicle_param.wheel_base = env_->vehicle.wheel_base;
  vehicle_param.front_hang = env_->vehicle.front_edge_to_center - env_->vehicle.wheel_base;
  vehicle_param.rear_hang = env_->vehicle.back_edge_to_center;
  vehicle_param.width = env_->vehicle.width;
  vehicle_param.v_max = std::abs(env_->vehicle.max_velocity);
  vehicle_param.a_max = env_->vehicle.max_acceleration;
  vehicle_param.delta_max = env_->vehicle.max_steer_angle;
  vehicle_param.omega_max = env_->vehicle.max_steer_angle_rate;
  vehicle_param.steer_ratio = env_->vehicle.steer_ratio;
  vehicle_param.GenerateDisc();

  // Set xy_bounds BEFORE creating planner
  auto bounds = env_->xy_bound();
  parking_config_.xy_bounds = bounds;
  ROS_INFO("Planning bounds: x=[%.2f, %.2f], y=[%.2f, %.2f]", bounds[0], bounds[1], bounds[2], bounds[3]);

  planner_ = std::make_shared<planning::HybridAStar>(parking_config_, vehicle_param, env_->obstacles);
  optimizer_ = std::make_shared<trajectory_nlp::TrajectoryOptimizer>(config_, env_);
  my_env.parking_type = parking_type;
  return PlanTrajectory(my_env, parking_type, TrajectoryPoint(start_pose_), TrajectoryPoint(goal_pose_), *result);
}

bool ParkingPlanner::PlanTrajectory(const MyEnvironment &my_env, int parking_type, TrajectoryPoint start, TrajectoryPoint goal, Trajectory &result) {
  double run_time = 0.0;

  // Explicitly set all dynamic states to zero to ensure proper terminal constraints
  goal.v = 0.0;      // Zero velocity (stopped)
  goal.a = 0.0;      // Zero acceleration
  goal.phi = 0.0;    // Straight wheels
  goal.omega = 0.0;  // Zero steering rate

  // Initialize start state dynamic values
  start.v = 0.0;     // Start from rest
  start.a = 0.0;     // Zero initial acceleration
  start.phi = 0.0;   // Straight wheels at start
  start.omega = 0.0; // Zero initial steering rate

  ROS_INFO("Terminal constraints set: goal position=(%.2f, %.2f, %.2f°), v=%.2f, a=%.2f, phi=%.2f",
           goal.x, goal.y, goal.theta * 180.0/M_PI, goal.v, goal.a, goal.phi);

  // xy_bounds already set in Plan() before creating planner_
  // reverse searching direction
  bool is_reversed = parking_type == 1;

  double start_time = common::util::GetCurrentTimestamp();
  planning::HybridAStartResult ha_result;

  // Use Plan method instead of PlanViaFTHA
  double sx = is_reversed ? goal.x : start.x;
  double sy = is_reversed ? goal.y : start.y;
  double sphi = is_reversed ? goal.theta : start.theta;
  double ex = is_reversed ? start.x : goal.x;
  double ey = is_reversed ? start.y : goal.y;
  double ephi = is_reversed ? start.theta : goal.theta;

  bool is_success = planner_->Plan(sx, sy, sphi, ex, ey, ephi, &ha_result);
  run_time = common::util::GetCurrentTimestamp() - start_time;

  if(!is_success) {
    ROS_ERROR("HA failed");
    return false;
  }

  ROS_INFO("Coarse Time: %f", run_time);

  // Hybrid A* path visualization
  VisualizationPlot::Plot(ha_result.x, ha_result.y, 0.3, Color::Blue, 1, "Hybrid A* Path");
  VisualizationPlot::Trigger();

  // Must reverse it back to Start->Goal before passing to optimizer
  if (is_reversed) {
    std::reverse(ha_result.x.begin(), ha_result.x.end());
    std::reverse(ha_result.y.begin(), ha_result.y.end());
    std::reverse(ha_result.phi.begin(), ha_result.phi.end());
    ROS_INFO("Reversed Hybrid A* path (Goal->Start) back to (Start->Goal) for optimization");
  }

  // ============ DIAGNOSTIC: Check Hybrid A* terminal accuracy AFTER reversal ============
  if (!ha_result.x.empty()) {
    size_t ha_last = ha_result.x.size() - 1;
    // Always check against the actual GOAL position (not ex, ey which might be start)
    double ha_terminal_dist = hypot(ha_result.x[ha_last] - goal.x, ha_result.y[ha_last] - goal.y);
    double ha_terminal_angle_diff = std::abs(common::math::NormalizeAngle(ha_result.phi[ha_last] - goal.theta));
    ROS_INFO("========== Hybrid A* Terminal Accuracy (After Reversal) ==========");
    ROS_INFO("  Goal: (%.3f, %.3f, %.3f°)", goal.x, goal.y, goal.theta * 180.0/M_PI);
    ROS_INFO("  HA Result: (%.3f, %.3f, %.3f°)", ha_result.x[ha_last], ha_result.y[ha_last],
             ha_result.phi[ha_last] * 180.0/M_PI);
    ROS_INFO("  Position error: %.6f m", ha_terminal_dist);
    ROS_INFO("  Heading error: %.3f° (%.6f rad)", ha_terminal_angle_diff * 180.0/M_PI, ha_terminal_angle_diff);
    if (ha_terminal_dist > 0.5) {
      ROS_WARN("  WARNING: Hybrid A* path does not reach goal! Distance: %.3fm", ha_terminal_dist);
    }
    ROS_INFO("==================================================================");
  }
  // =====================================================================================

  trajectory_nlp::CoarseDecision decision;
  decision.x = ha_result.x;
  decision.y = ha_result.y;
  decision.theta = ToContinuousAngle(ha_result.phi);

  if (!decision.x.empty()) {
    size_t last_idx = decision.x.size() - 1;
    double term_dist_before = hypot(decision.x[last_idx] - goal.x, decision.y[last_idx] - goal.y);
    double angle_diff_before = std::abs(common::math::NormalizeAngle(decision.theta[last_idx] - goal.theta));

    // Always force exact match to prevent initial infeasibility
    if (term_dist_before > 1e-9 || angle_diff_before > 1e-9) {
      ROS_WARN("CORRECTING coarse path terminal: pos error=%.6f m, angle error=%.6f°",
               term_dist_before, angle_diff_before * 180.0/M_PI);
      decision.x[last_idx] = goal.x;
      decision.y[last_idx] = goal.y;
      decision.theta[last_idx] = goal.theta;

      ROS_INFO("  Before: (%.6f, %.6f, %.6f°)",
               decision.x[last_idx] - (decision.x[last_idx] - goal.x),
               decision.y[last_idx] - (decision.y[last_idx] - goal.y),
               (decision.theta[last_idx] - (decision.theta[last_idx] - goal.theta)) * 180.0/M_PI);
      ROS_INFO("  After:  (%.6f, %.6f, %.6f°)", goal.x, goal.y, goal.theta * 180.0/M_PI);
    }
  }

  start_time = common::util::GetCurrentTimestamp();
  trajectory_nlp::States states;
  is_success = optimizer_->Optimize(decision, start, goal, states);
  double opti_run_time = common::util::GetCurrentTimestamp() - start_time;
  run_time += opti_run_time;

  ROS_INFO("Real Optimize Time: %f", opti_run_time);
  ROS_INFO("Real Run Time: %f", run_time);

  if (!is_success) {
    ROS_ERROR("Optimization failed!");
    config_.nfe *= 2;
    optimizer2_ = std::make_shared<trajectory_nlp::TrajectoryOptimizer>(config_, env_);
    is_success = optimizer2_->Optimize(decision, start, goal, states);
    double opti_run_time = common::util::GetCurrentTimestamp() - start_time;
    run_time += opti_run_time;
    ROS_INFO("Real Run Time: %f", run_time);
  }

  std::lock_guard<std::mutex> lock(trajectory_mutex);
  result.clear();

  // Check if states are valid
  if (states.x.empty()) {
    ROS_ERROR("Optimization returned empty states!");
    return false;
  }

  for(size_t i = 0; i < states.x.size(); i++) {
    TrajectoryPoint tp{};
    tp.x = states.x[i]; tp.y = states.y[i]; tp.theta = states.theta[i];
    tp.v = states.v[i]; tp.phi = states.phi[i]; tp.a = states.a[i]; tp.omega = states.omega[i];
    tp.kappa = tan(tp.phi) / env_->vehicle.wheel_base;
    result.push_back(tp);
  }

  ComputeTimeAndDistance(result, states.tf, config_.nfe);

  // Add goal points at the end (only if result is not empty)
  if (!result.empty()) {
    for(size_t i = 0; i < 20; i++) {
      TrajectoryPoint pt{};
      pt.x = goal.x;
      pt.y = goal.y;
      pt.theta = goal.theta;
      pt.t = result[result.size() - 1].t + 0.001 * (i+1);
      pt.s = result[result.size() - 1].s;
      result.push_back(pt);
    }
  }

  return true;
}
