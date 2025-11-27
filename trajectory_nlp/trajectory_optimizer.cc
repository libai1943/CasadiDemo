#include <bitset>
#include <chrono>

#include "trajectory_optimizer.h"
#include "common/math/math_utils.h"
#include "common/math/trajectory1d.h"
#include "common/util/vector.h"
#include "common/util/csv_logger.h"
#include "visualization_plot.h"

namespace trajectory_nlp {

using common::math::Trajectory1d;
using common::util::NLPOptimizationLogger;

TrajectoryOptimizer::TrajectoryOptimizer(const TrajectoryNLPConfig &config, Env env): config_(config), env_(std::move(env)), nlp_(config, env_) {
  vehicle_ = env_->vehicle;
}
// LIOM
bool TrajectoryOptimizer::Optimize(const CoarseDecision &decision, const TrajectoryPoint &start, const TrajectoryPoint &goal, States &result) {
  // 初始化CSV记录器
  NLPOptimizationLogger::Initialize();

  // 1. 初始解生成 (Resample)
  auto coarse = decision;
  auto guess = ResampleCoarsePath(coarse);
  CalculateInitialGuess(guess);

  // ============ Verify uniform time intervals ============
  ROS_INFO("========== Initial Guess Time Verification ==========");
  ROS_INFO("Total points (nfe): %zu", guess.t.size());
  ROS_INFO("Total time (tf): %.6f", guess.tf);
  double expected_dt = guess.tf / (guess.t.size() - 1);
  ROS_INFO("Expected uniform dt: %.6f", expected_dt);

  // Check first few intervals
  ROS_INFO("First 5 time intervals:");
  for (size_t i = 1; i < std::min(size_t(6), guess.t.size()); i++) {
    double dt = guess.t[i] - guess.t[i-1];
    ROS_INFO("  t[%zu] - t[%zu] = %.6f - %.6f = %.9f (error: %.2e)",
             i, i-1, guess.t[i], guess.t[i-1], dt, std::abs(dt - expected_dt));
  }

  // Check last few intervals
  ROS_INFO("Last 5 time intervals:");
  size_t start_idx = guess.t.size() >= 6 ? guess.t.size() - 5 : 1;
  for (size_t i = start_idx; i < guess.t.size(); i++) {
    double dt = guess.t[i] - guess.t[i-1];
    ROS_INFO("  t[%zu] - t[%zu] = %.6f - %.6f = %.9f (error: %.2e)",
             i, i-1, guess.t[i], guess.t[i-1], dt, std::abs(dt - expected_dt));
  }

  // Check terminal states
  ROS_INFO("Terminal state verification:");
  ROS_INFO("  goal: x=%.3f, y=%.3f, theta=%.3f, v=%.3f, a=%.3f, phi=%.3f",
           goal.x, goal.y, goal.theta, goal.v, goal.a, goal.phi);
  size_t last = guess.t.size() - 1;
  ROS_INFO("  guess[%zu]: x=%.3f, y=%.3f, theta=%.3f, v=%.3f, a=%.3f, phi=%.3f",
           last, guess.x[last], guess.y[last], guess.theta[last],
           guess.v[last], guess.a[last], guess.phi[last]);
  double terminal_dist = hypot(guess.x[last] - goal.x, guess.y[last] - goal.y);
  ROS_INFO("  Distance from guess to goal: %.6f m", terminal_dist);
  ROS_INFO("======================================================");
  // ====================================================================

  Constraints constraints;
  constraints.start = start;
  constraints.goal = goal;

  // 保存reference trajectory (coarse path重采样后的结果)
  States reference = guess;

  // 初始化迭代变量
  States current_states = guess;
  States previous_states;  // Store previous iteration's solution for convergence check

  // 标准LIOM权重策略：从较小值开始，每次迭代乘以alpha
  double w_penalty = config_.opti_w_penalty0;

  int iter = 0;

  while (iter < config_.opti_iter_max) {

      // Step 1: 基于当前轨迹生成走廊 (GenerateCorridors)
      constraints.gears = common::math::GetPathGears(current_states.x, current_states.y, current_states.theta);
      if (!FormulateCorridorConstraints(current_states, constraints)) {
          std::cout << "[LIOM] Failed to generate corridors at iter " << iter << std::endl;
          // 如果生成走廊失败，可能需要回退或终止
          NLPOptimizationLogger::LogFinalResult(current_states.tf, current_states.x, current_states.y,
                                                current_states.theta, current_states.v, current_states.phi,
                                                current_states.a, current_states.omega, false);
          NLPOptimizationLogger::Finalize();
          return false;
      }

      // Step 2: 求解轻量级 OCP (SolveOCP) with reference trajectory tracking
      States next_states;
      double cur_infeasibility = 0.0;

      bool solver_success = nlp_.SolveIteratively(iter, w_penalty, constraints, current_states, reference, next_states, cur_infeasibility);

      VisualizationPlot::Plot(next_states.x, next_states.y, 0.05, Color::Grey, iter, "Intermediate Iteration");
      VisualizationPlot::Trigger();

      if (!solver_success) {
          std::cout << "[LIOM] Solver failed at iter " << iter << std::endl;
          NLPOptimizationLogger::LogFinalResult(current_states.tf, current_states.x, current_states.y,
                                                current_states.theta, current_states.v, current_states.phi,
                                                current_states.a, current_states.omega, false);
          NLPOptimizationLogger::Finalize();
          return false;
      }

      std::cout << "[LIOM] iter = " << iter << ", cur_infeasibility = " << cur_infeasibility << ", w_penalty = " << w_penalty << std::endl;

      // Step 3: 检查收敛性
      // 不仅要求运动学可行（infeasibility小），还要求前后两轮解向量一致
      bool kinematic_feasible = (cur_infeasibility < config_.opti_varepsilon_tol);
      bool solution_converged = false;
      double solution_norm_diff = 0.0;

      if (iter > 0) {
          // Compute norm difference between current and previous solution
          solution_norm_diff = ComputeSolutionNormDifference(next_states, previous_states);
          solution_converged = (solution_norm_diff < config_.opti_solution_norm_tol);
          std::cout << "[LIOM] Solution norm difference = " << solution_norm_diff << std::endl;
      }

      // Convergence criterion: kinematic feasibility AND solution consistency
      if (kinematic_feasible && (iter == 0 || solution_converged)) {
          std::cout << "[LIOM] Converged at iter " << iter
                    << " with infeasibility " << cur_infeasibility
                    << " and solution norm diff " << solution_norm_diff << std::endl;
          result = next_states;

          // Log final corridor constraints (only for converged solution)
          Constraints final_constraints;
          final_constraints.start = start;
          final_constraints.goal = goal;
          final_constraints.gears = common::math::GetPathGears(result.x, result.y, result.theta);
          FormulateCorridorConstraints(result, final_constraints, true);  // log_to_csv = true

          // 记录最终结果
          NLPOptimizationLogger::LogFinalResult(result.tf, result.x, result.y, result.theta,
                                                result.v, result.phi, result.a, result.omega, true);
          NLPOptimizationLogger::Finalize();

          // Final trajectory visualization: normal line width with vehicle footprints
          VisualizationPlot::PlotTrajectory(result.x, result.y, result.v, vehicle_.max_velocity, 0.1, 999, "Final Trajectory");
          VisualizationPlot::PlotVehicleFootprints(result.x, result.y, result.theta,
                                                    vehicle_.length, vehicle_.width, vehicle_.back_edge_to_center,
                                                    Color::Cyan, 0.05, 1000, "Vehicle Footprints");
          VisualizationPlot::Trigger();
          return true;
      }

      // Step 4: 更新状态和权重进入下一次迭代
      previous_states = current_states;  // Save current state before updating
      current_states = next_states;

      // Update penalty weight with cap to prevent ill-conditioning
      if (w_penalty < config_.opti_w_penalty_max) {
        w_penalty *= config_.opti_alpha;
        // Ensure it doesn't exceed the maximum
        w_penalty = std::min(w_penalty, config_.opti_w_penalty_max);
      }

      iter++;
  }

  std::cout << "[LIOM] Reached max iterations without full convergence." << std::endl;

  // Log final corridor constraints (even for non-converged solution, for debugging)
  Constraints final_constraints;
  final_constraints.start = start;
  final_constraints.goal = goal;
  final_constraints.gears = common::math::GetPathGears(current_states.x, current_states.y, current_states.theta);
  FormulateCorridorConstraints(current_states, final_constraints, true);  // log_to_csv = true

  // 记录最终结果（未收敛）
  NLPOptimizationLogger::LogFinalResult(current_states.tf, current_states.x, current_states.y,
                                        current_states.theta, current_states.v, current_states.phi,
                                        current_states.a, current_states.omega, false);
  NLPOptimizationLogger::Finalize();

  return false;
}

// Function to resample the coarse decision into a finer path
States TrajectoryOptimizer::ResampleCoarsePath(CoarseDecision &coarse) {

  // Generate an optimal time profile for the coarse decision
  auto time_profile = GenerateOptimalTimeProfile(coarse);

  // Create linearly spaced time samples
  auto time_sampled = common::util::LinSpaced(0, time_profile.back(), config_.nfe);

  States result; // Declare an object to store the resulting resampled states
  result.tf = time_profile.back(); // Store the final time
  result.t = time_sampled; // Store the time samples

  // Resample x, y, and theta using 1D interpolation
  result.x = Trajectory1d(time_profile, coarse.x).Interpolate1d(time_sampled).GetY();
  result.y = Trajectory1d(time_profile, coarse.y).Interpolate1d(time_sampled).GetY();
  result.theta = common::math::ToContinuousAngle(Trajectory1d(time_profile, coarse.theta).Interpolate1d(time_sampled).GetY());

  return result; // Return the resampled states
}

// Function to calculate an initial guess for the states (参考CartesianPlanner)
void TrajectoryOptimizer::CalculateInitialGuess(States &states) const {

  // Initialize vectors for velocity and steering angle
  states.v.resize(config_.nfe, 0.0);
  states.phi.resize(config_.nfe, 0.0);

  // Loop to populate the initial guesses for velocity and steering angle
  for(size_t i = 1; i < config_.nfe; i++) {
    // Use actual time difference instead of assuming uniform time steps
    double dt = states.t[i] - states.t[i-1];

    // Skip if time step is too small
    if (dt < 1e-6) {
      states.v[i] = (i > 1) ? states.v[i-1] : 0.0;
      states.phi[i] = (i > 1) ? states.phi[i-1] : 0.0;
      continue;
    }

    // Calculate tracking angle between consecutive points
    double tracking_angle = atan2(states.y[i] - states.y[i-1], states.x[i] - states.x[i-1]);

    // Determine if the vehicle should move forward or backward
    bool gear = std::abs(common::math::NormalizeAngle(tracking_angle - states.theta[i])) < M_PI_2;

    // Calculate velocity based on distance between points and ACTUAL time step
    double velocity = hypot(states.y[i] - states.y[i-1], states.x[i] - states.x[i-1]) / dt;
    states.v[i] = std::min(vehicle_.max_velocity, std::max(vehicle_.max_reverse_velocity, gear ? velocity : -velocity));

    // Calculate the steering angle based on change in orientation and velocity
    // Prevent division by zero when velocity is near zero (e.g., at gear switch or start/stop)
    if (std::abs(states.v[i]) < 1e-3) {
      // Velocity too small, cannot reliably compute steering from kinematics
      // Keep previous steering angle or set to zero
      states.phi[i] = (i > 1) ? states.phi[i-1] : 0.0;
    } else {
      double raw_phi = atan((states.theta[i] - states.theta[i-1]) * vehicle_.wheel_base / (states.v[i] * dt));
      states.phi[i] = std::min(vehicle_.max_steer_angle, std::max(-vehicle_.max_steer_angle, raw_phi));
    }
  }

  // Initialize acceleration
  states.a.resize(config_.nfe, 0.0);
  for(size_t i = 1; i < config_.nfe; i++) {
    double dt = states.t[i] - states.t[i-1];
    if (dt < 1e-6) {
      states.a[i] = (i > 1) ? states.a[i-1] : 0.0;
      continue;
    }
    states.a[i] = std::min(vehicle_.max_acceleration,
                           std::max(vehicle_.max_deceleration, (states.v[i] - states.v[i-1]) / dt));
  }

  // Initialize steering rate
  states.omega.resize(config_.nfe, 0.0);
  for(size_t i = 1; i < config_.nfe; i++) {
    double dt = states.t[i] - states.t[i-1];
    if (dt < 1e-6) {
      states.omega[i] = (i > 1) ? states.omega[i-1] : 0.0;
      continue;
    }
    states.omega[i] = std::min(vehicle_.max_steer_angle_rate,
                               std::max(-vehicle_.max_steer_angle_rate, (states.phi[i] - states.phi[i-1]) / dt));
  }
}

// Generate optimal time profile for a given coarse trajectory decision
std::vector<double> TrajectoryOptimizer::GenerateOptimalTimeProfile(const CoarseDecision &coarse) {
  // Declare a vector 'gears' to store gear values for each point (either 1 for forward or -1 for reverse)
  std::vector<int> gears(coarse.x.size());
  // Declare a vector 'stations' initialized to 0s to store accumulated distance at each point on the coarse trajectory
  std::vector<double> stations(coarse.x.size(), 0);
  // Loop through the gears vector to determine the gear and the accumulated distance for each point
  for(size_t i = 0; i < gears.size(); i++) {
    // Calculate the gear (forward or reverse) for each point except the last one
    if(i < gears.size()-1) {
      // Calculate the angle formed by the current point and the next point
      double tracking_angle = atan2(coarse.y[i+1] - coarse.y[i], coarse.x[i+1] - coarse.x[i]);
      // Determine if the vehicle should be in forward or reverse gear based on the tracking angle and theta value
      bool gear = std::abs(common::math::NormalizeAngle(tracking_angle - coarse.theta[i])) < M_PI_2;
      // Assign gear value based on the calculated boolean: 1 for forward and -1 for reverse
      gears[i] = gear ? 1 : -1;
    }
    // Calculate accumulated distance for points after the first one
    if(i > 0) {
      // Update stations vector with accumulated distance from previous point to current point
      stations[i] = stations[i-1] + hypot(coarse.x[i] - coarse.x[i-1], coarse.y[i] - coarse.y[i-1]);
    }
  }
  // Set the last gear value same as the second to last
  gears[gears.size()-1] = gears[gears.size()-2];
  // Initialize a time profile vector to store time values for each point on the trajectory
  std::vector<double> time_profile(gears.size());
  size_t last_idx = 0;  // Index to track the start of the current segment
  double start_time = 0;
  // Loop through the gears vector to generate the optimal time profile for each segment
  for(size_t i = 0; i < gears.size(); i++) {
    // Split the trajectory at points where gear changes or at the last point
    if(i == gears.size() - 1 || gears[i+1] != gears[i]) {
      // Extract the current segment of stations (from last_idx to current i)
      std::vector<double> station_segment;
      std::copy_n(stations.begin() + last_idx, i - last_idx + 1, std::back_inserter(station_segment));
      // Generate the time profile for the current segment
      auto profile = GenerateOptimalTimeProfileSegment(gears[i], station_segment, start_time);
      // Update the main time profile with the calculated values for the current segment
      std::copy(profile.begin(), profile.end(), std::next(time_profile.begin(), last_idx));
      // Update the start time for the next segment
      start_time = profile.back();
      // Update the index to track the start of the next segment
      last_idx = i + 1;
    }
  }
  // Return the final time profile
  return time_profile;
}
// Generate an optimal time profile for a given segment based on gear, stations, and starting time
std::vector<double>
TrajectoryOptimizer::GenerateOptimalTimeProfileSegment(int gear, const std::vector<double> &stations, double start_time) const {
  // Initialize vehicle parameters
  double max_accel = vehicle_.max_acceleration; double max_decel = vehicle_.max_deceleration;
  double max_velocity = vehicle_.max_velocity; double min_velocity = vehicle_.max_reverse_velocity;
  // If the gear is reverse, invert the vehicle parameters
  if(gear < 0) {
    max_accel = -vehicle_.max_deceleration; max_decel = -vehicle_.max_acceleration;
    max_velocity = -vehicle_.max_reverse_velocity; min_velocity = -vehicle_.max_velocity;
  }
  // Indices to keep track of where the vehicle starts and stops accelerating
  int accel_idx = 0, decel_idx = stations.size()-1;
  double vi = 0.0;  // Initial velocity
  // Initialize a profile to store velocity values for each point
  std::vector<double> profile(stations.size());
  // Calculate velocity profile considering acceleration
  for (int i = 0; i < stations.size()-1; i++) {
    // Calculate the distance between the current and next point
    double ds = stations[i+1] - stations[i];
    // Store the current velocity for the point
    profile[i] = vi;
    // Update the velocity considering the acceleration and the distance to the next point
    vi = sqrt(vi * vi + 2 * max_accel * ds);
    vi = std::min(max_velocity, std::max(min_velocity, vi));
    // If the vehicle reaches max velocity, update the acceleration index and break
    if(vi >= max_velocity) {
      accel_idx = i+1;
      break;
    }
  }

  vi = 0.0;  // Reset initial velocity
  // Calculate velocity profile considering deceleration
  for (int i = stations.size()-1; i > accel_idx; i--) {
    // Calculate the distance between the current and previous point
    double ds = stations[i] - stations[i-1];
    // Store the current velocity for the point
    profile[i] = vi;
    // Update the velocity considering the deceleration and the distance from the previous point
    vi = sqrt(vi * vi - 2 * max_decel * ds);
    vi = std::min(max_velocity, std::max(min_velocity, vi));
    // If the vehicle reaches max velocity, update the deceleration index and break
    if(vi >= max_velocity) {
      decel_idx = i;
      break;
    }
  }
  // Assign max velocity values for the points between acceleration and deceleration indices
  std::fill(std::next(profile.begin(), accel_idx), std::next(profile.begin(), decel_idx), max_velocity);
  // Initialize a time profile to store time values for each point, starting with the given start time
  std::vector<double> time_profile(stations.size(), start_time);
  // Calculate the time profile based on the calculated velocities and distances between points
  for(size_t i = 1; i < stations.size(); i++) {
    // If the velocity for the current point is very close to 0, assign the previous time value
    if(profile[i] < 1e-6) {
      time_profile[i] = time_profile[i-1];
      continue;
    }
    // Calculate the time for the current point based on the distance and velocity
    time_profile[i] = time_profile[i-1] + (stations[i] - stations[i-1]) / profile[i];
  }
  // Return the calculated time profile for the segment
  return time_profile;
}

bool TrajectoryOptimizer::FormulateCorridorConstraints(const States &states, Constraints &constraints, bool log_to_csv) {
  constraints.front_bound.resize(config_.nfe);
  constraints.rear_bound.resize(config_.nfe);

  for(size_t i = 0; i < config_.nfe; i++) {
    double xf, yf, xr, yr;
    std::tie(xf, yf, xr, yr) = vehicle_.GetDiscPositions(states.x[i], states.y[i], states.theta[i]);

    AABox2d front, rear;
    if(!GenerateAABox(xf, yf, vehicle_.radius, front)) {
      return false;
    }
    constraints.front_bound[i] = { front.min_x(), front.max_x(), front.min_y(), front.max_y() };

    // Only log corridor constraints if explicitly requested (e.g., at final convergence)
    if (log_to_csv) {
      NLPOptimizationLogger::LogCorridorConstraints(i,
                                                    front.min_x(), front.max_x(),
                                                    front.min_y(), front.max_y(),
                                                    true);
    }

    if(!GenerateAABox(xr, yr, vehicle_.radius, rear)) {
      return false;
    }
    constraints.rear_bound[i] = { rear.min_x(), rear.max_x(), rear.min_y(), rear.max_y() };

    // Only log corridor constraints if explicitly requested (e.g., at final convergence)
    if (log_to_csv) {
      NLPOptimizationLogger::LogCorridorConstraints(i,
                                                    rear.min_x(), rear.max_x(),
                                                    rear.min_y(), rear.max_y(),
                                                    false);
    }
  }

  // Plot Corridor - light color and thin line to avoid visual clutter
  for(size_t i = 0; i < constraints.rear_bound.size(); i++) {
    Polygon2d box = Polygon2d(Box2d(
        AABox2d({constraints.rear_bound[i][0], constraints.rear_bound[i][2]},
                {constraints.rear_bound[i][1], constraints.rear_bound[i][3]})));
    Color corridor_color(0.7, 0.7, 0.7);
    corridor_color.set_a(0.5);
    VisualizationPlot::PlotPolygon(box, 0.02, corridor_color, i, "Rear Corridor");
  }
  VisualizationPlot::Trigger();

  return true;
}
// This function attempts to generate an axis-aligned bounding box (AABox) for a given
// (x, y) point with the specified radius. The bounding box is adjusted to avoid collisions.
bool TrajectoryOptimizer::GenerateAABox(double &x, double &y, double radius, AABox2d &box) const {
  using namespace std::chrono;
  static int call_count = 0;
  static duration<double> total_duration(0);

  auto start = high_resolution_clock::now(); // 开始计时
  // Create an initial bounding box using the given point and radius.
  double ri = radius;
  AABox2d bound({x-ri, y-ri}, {x+ri, y+ri});
  // Check if the initial bounding box collides with any obstacle in the environment.
  if(env_->CheckCollision(Box2d(bound))) {
    // initial condition not satisfied, involute to find feasible box
    // The initial box has a collision. Begin the process to adjust (involute) the box.
    int inc = 4;
    double real_x, real_y;
    // Repeat the adjustment process until a collision-free box is found or iteration limits are reached.
    do {
      // Determine which edge of the bounding box to adjust.
      int iter = inc / 4;
      uint8_t edge = inc % 4;
      // Initialize the adjusted position to the current (x, y).
      real_x = x;
      real_y = y;
      // Adjust the box's position based on which edge is being modified.
      if(edge == 0) {
        real_x = x - iter * 0.01;
      } else if(edge == 1) {
        real_x = x + iter * 0.01;
      } else if(edge == 2) {
        real_y = y - iter * 0.01;
      } else if(edge == 3) {
        real_y = y + iter * 0.01;
      }
      // Update the bounding box with the adjusted position.
      inc++;
      bound = AABox2d({real_x-ri, real_y-ri}, {real_x+ri, real_y+ri});
    } while(env_->CheckCollision(Box2d(bound)) && inc < config_.corridor_max_iter);
    // If the adjustment iterations exceed the allowed limit, return false.
    if(inc > config_.corridor_max_iter) {
      return false;
    }
    // Update the original x and y with the adjusted values.
    x = real_x;
    y = real_y;
  }
  // Begin the process to expand the bounding box edges as much as possible, while avoiding collisions.
  int inc = 4;
  std::bitset<4> blocked;  // Tracks which edges can no longer be expanded.
  double incremental[4] = {0.0};
  double step = radius * 0.01;//0.1;//radius * 0.005;
  // Repeat the expansion process until all edges are blocked or iteration limits are reached.
  do {
    int iter = inc / 4;
    uint8_t edge = inc % 4;
    inc++;
    // If the current edge is already blocked from further expansion, skip to the next iteration.
    if(blocked[edge]) continue;
    // Attempt to expand the current edge.
    incremental[edge] = iter * step;
    // Generate a test bounding box with the expanded edge.
    AABox2d test({x - ri - incremental[0], y - ri - incremental[2]},
                 { x + ri + incremental[1], y + ri + incremental[3]});
    // If the test bounding box collides or the expansion reaches a limit, block further expansion of the current edge.
    if(env_->CheckCollision(Box2d(test)) || incremental[edge] >= config_.corridor_incremental_limit) {
      incremental[edge] -= step;
      blocked[edge] = true;
    }
  } while(!blocked.all() && inc < config_.corridor_max_iter);
  // If the expansion iterations exceed the allowed limit, return false.
  if(inc > config_.corridor_max_iter) {
    return false;
  }
  // Finalize the bounding box using the expanded edges and assign it to the 'box' argument.
  box = {{x - incremental[0], y - incremental[2]},
          { x + incremental[1], y + incremental[3]}};

  auto end = high_resolution_clock::now(); // 结束计时
  duration<double> elapsed = end - start; // 计算经过的时间
  total_duration += elapsed; // 累加到总时间
  call_count++; // 调用次数增加

  // Return true to indicate a successful generation of the bounding box.
  return true;
}

// Compute L2 norm difference between two solution vectors
// This includes all state variables: tf, x, y, theta, v, phi, a, omega
double TrajectoryOptimizer::ComputeSolutionNormDifference(const States &states1, const States &states2) const {
  double norm_diff = 0.0;

  // Add difference in tf
  double tf_diff = states1.tf - states2.tf;
  norm_diff += tf_diff * tf_diff;

  // Add differences in all trajectory points
  for(size_t i = 0; i < config_.nfe; i++) {
    double x_diff = states1.x[i] - states2.x[i];
    double y_diff = states1.y[i] - states2.y[i];
    double theta_diff = states1.theta[i] - states2.theta[i];
    double v_diff = states1.v[i] - states2.v[i];
    double phi_diff = states1.phi[i] - states2.phi[i];
    double a_diff = states1.a[i] - states2.a[i];
    double omega_diff = states1.omega[i] - states2.omega[i];

    norm_diff += x_diff * x_diff + y_diff * y_diff + theta_diff * theta_diff
               + v_diff * v_diff + phi_diff * phi_diff + a_diff * a_diff
               + omega_diff * omega_diff;
  }

  return std::sqrt(norm_diff);
}

}
