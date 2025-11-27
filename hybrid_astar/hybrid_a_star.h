/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#pragma once

#include <algorithm>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <array>

#include "common/math/math_utils.h"
#include "hybrid_astar/grid_search.h"
#include "hybrid_astar/node3d.h"
#include "hybrid_astar/reeds_shepp_path.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "hybrid_astar/parking_planner_config.h"

namespace planning {

struct HybridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
};

class HybridAStar {
public:
  HybridAStar(const ParkingPlannerConfig &config, 
              const VehicleParameter &vehicle, 
              const std::vector<common::math::Polygon2d> &obstacles);
  virtual ~HybridAStar() = default;
  bool Plan(double sx, double sy, double sphi, double ex, double ey, double ephi,
            HybridAStartResult* result);
  bool TrajectoryPartition(const HybridAStartResult& result,
                           std::vector<HybridAStartResult>* partitioned_result);

private:
  bool AnalyticExpansion(const std::shared_ptr<Node3d>& current_node);
  // check collision and validity
  bool ValidityCheck(const std::shared_ptr<Node3d>& node);
  // check Reeds Shepp path collision and validity
  bool RSPCheck(const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end);
  // load the whole RSP as nodes and add to the close set
  std::shared_ptr<Node3d> LoadRSPinCS(
      const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
      std::shared_ptr<Node3d> current_node);
  std::shared_ptr<Node3d> Next_node_generator(
      const std::shared_ptr<Node3d>& current_node, size_t next_node_index);
  void CalculateNodeCost(const std::shared_ptr<Node3d>& current_node,
                         const std::shared_ptr<Node3d>& next_node);
  double TrajCost(const std::shared_ptr<Node3d>& current_node,
                  const std::shared_ptr<Node3d>& next_node);
  double HoloObstacleHeuristic(const std::shared_ptr<Node3d>& next_node);
  bool GetResult(HybridAStartResult* result);
  bool GetTemporalProfile(HybridAStartResult* result);
  bool GenerateSpeedAcceleration(HybridAStartResult* result);

private:
  ParkingPlannerConfig planner_open_space_config_;
  VehicleParameter vehicle_param_;
  size_t next_node_num_ = 0;
  double max_steer_angle_ = 0.0;
  double step_size_ = 0.0;
  double xy_grid_resolution_ = 0.0;
  double delta_t_ = 0.0;
  double traj_forward_penalty_ = 0.0;
  double traj_back_penalty_ = 0.0;
  double traj_gear_switch_penalty_ = 0.0;
  double traj_steer_penalty_ = 0.0;
  double traj_steer_change_penalty_ = 0.0;
  double heu_rs_forward_penalty_ = 0.0;
  double heu_rs_back_penalty_ = 0.0;
  double heu_rs_gear_switch_penalty_ = 0.0;
  double heu_rs_steer_penalty_ = 0.0;
  double heu_rs_steer_change_penalty_ = 0.0;
  std::array<double, 4> XYbounds_;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> final_node_;
  std::vector<common::math::Polygon2d> obstacles_;

  struct cmp {
    bool operator()(const std::pair<int64_t, double>& left,
                    const std::pair<int64_t, double>& right) const {
      return left.second >= right.second;
    }
  };
  std::priority_queue<std::pair<int64_t, double>,
      std::vector<std::pair<int64_t, double>>, cmp>
      open_pq_;
  std::unordered_map<int64_t, std::shared_ptr<Node3d>> open_set_;
  std::unordered_map<int64_t, std::shared_ptr<Node3d>> close_set_;
  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;
};

}  // namespace planning
