#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "common/math/pose.h"
#include "hybrid_astar/hybrid_a_star.h"
#include "trajectory_nlp/trajectory_optimizer.h"

#include "common/trajectory_point.h"
#include "my_env.h"

using namespace common::math;

inline std::vector<Vec2d> PointsToVec(const std::vector<geometry_msgs::Point> &points) {
  std::vector<Vec2d> tmp(points.size());
  for(size_t i = 0; i < points.size(); i++) {
    tmp[i].set_x(points[i].x);
    tmp[i].set_y(points[i].y);
  }
  return tmp;
}

class ParkingPlanner {
public:
  std::mutex trajectory_mutex;
  ParkingPlanner();

  void ReadConfig();

  void set_current_state(double x, double y, double theta, double v, double phi, double a, double omega) {
    current_state_.x = x;
    current_state_.y = y;
    current_state_.theta = theta;
    current_state_.v = v;
    current_state_.phi = phi;
    current_state_.a = a;
    current_state_.omega = omega;
  }

  void set_current_state(double x, double y, double theta, double v){
    current_state_.x = x;
    current_state_.y = y;
    current_state_.theta = theta;
    current_state_.v = v;
  }

  void set_start_pose(double x, double y, double theta) {
    start_pose_ = Pose(x, y, theta);
  }

  const Pose &start_pose() const {
    return start_pose_;
  }

  const TrajectoryPoint &current_state() const {
    return current_state_;
  }

  void set_goal_pose(double x, double y, double theta) {
    goal_pose_ = Pose(x, y, theta);
  }

  const Pose &goal_pose() const {
    return goal_pose_;
  }

  void Clear() {
    env_->points.Clear();
  }

  void SetParkingSpace(std::vector<geometry_msgs::Point> &points) {
    if(!points.empty()) {
      env_->points.AddParkingSpace(PointsToVec(points));
      env_->UpdateBounds();  // Update boundary values from PointCloud
    }
  }

  void SetBorderPoints(const std::vector<std::vector<geometry_msgs::Point>> &points) {
    for(auto &pc: points) {
      env_->points.AddPoints(PointsToVec(pc));
    }
    env_->UpdateBounds();  // Update boundary values from PointCloud
  }

  void SetObstacles(const std::vector<std::vector<geometry_msgs::Point>> &obstacles) {
    env_->obstacles.resize(obstacles.size());
    for(size_t i = 0; i < obstacles.size(); i++) {
      env_->obstacles[i] = Polygon2d(PointsToVec(obstacles[i]));
    }
  }

  /**
   *
   * @param result
   * @param parking_type 1: goal parking or no parking, 2: start parking
   * @return
   */
  bool Plan(Trajectory *result, int parking_type);

  Env env() {
    return env_;
  }

private:
  Env env_;
  ParkingPlannerConfig parking_config_;
  TrajectoryNLPConfig config_;
  std::shared_ptr<planning::HybridAStar> planner_;
  std::shared_ptr<trajectory_nlp::TrajectoryOptimizer> optimizer_, optimizer2_;

  ros::NodeHandle nh_;
  Pose start_pose_, goal_pose_;
  TrajectoryPoint current_state_;
  std::string env_file_;

  bool PlanTrajectory(const MyEnvironment &my_env, int parking_type, TrajectoryPoint start, TrajectoryPoint goal, Trajectory &result);
};
