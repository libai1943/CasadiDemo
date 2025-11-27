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

#include "hybrid_astar/node3d.h"
#include <array>
#include <cassert>

namespace planning {

using common::math::Box2d;

Node3d::Node3d(double x, double y, double phi) {
  x_ = x;
  y_ = y;
  phi_ = phi;
}

Node3d::Node3d(double x, double y, double phi,
               const std::array<double, 4> &XYbounds,
               const ParkingPlannerConfig &open_space_conf) {
  x_ = x;
  y_ = y;
  phi_ = phi;

  x_grid_ = static_cast<int>(
      (x_ - XYbounds[0]) /
      open_space_conf.xy_grid_resolution);
  y_grid_ = static_cast<int>(
      (y_ - XYbounds[2]) /
      open_space_conf.xy_grid_resolution);
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      open_space_conf.phi_grid_resolution);

  traversed_x_.push_back(x);
  traversed_y_.push_back(y);
  traversed_phi_.push_back(phi);

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
}

Node3d::Node3d(const std::vector<double> &traversed_x,
               const std::vector<double> &traversed_y,
               const std::vector<double> &traversed_phi,
               const std::array<double, 4> &XYbounds,
               const ParkingPlannerConfig &open_space_conf) {
  assert(traversed_x.size() == traversed_y.size());
  assert(traversed_x.size() == traversed_phi.size());

  x_ = traversed_x.back();
  y_ = traversed_y.back();
  phi_ = traversed_phi.back();

  // XYbounds in xmin, xmax, ymin, ymax
  x_grid_ = static_cast<int>(
      (x_ - XYbounds[0]) /
      open_space_conf.xy_grid_resolution);
  y_grid_ = static_cast<int>(
      (y_ - XYbounds[2]) /
      open_space_conf.xy_grid_resolution);
  phi_grid_ = static_cast<int>(
      (phi_ - (-M_PI)) /
      open_space_conf.phi_grid_resolution);

  traversed_x_ = traversed_x;
  traversed_y_ = traversed_y;
  traversed_phi_ = traversed_phi;

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
  step_size_ = traversed_x.size();
}

Box2d Node3d::GetBoundingBox(const VehicleParameter &vehicle_param_,
                             const double x, const double y, const double phi) {
  double ego_length = vehicle_param_.length;
  double ego_width = vehicle_param_.width;
  double shift_distance =
      ego_length / 2.0 - vehicle_param_.rear_hang;
  Box2d ego_box(
      { x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi) },
      phi, ego_length, ego_width);
  return ego_box;
}

bool Node3d::operator==(const Node3d &right) const {
  return right.GetIndex() == index_;
}

int64_t Node3d::ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
  // WARNING: may lost high 12 bits, it's fine in little-endian world when less than 2^20(1048576)
  return ((int64_t) x_grid << 40) | ((int64_t) y_grid << 20) | phi_grid;
}

}  // namespace planning
