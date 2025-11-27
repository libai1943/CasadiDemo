#pragma once
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/util/point_cloud.h"
#include "common/vehicle_param.h"
#include "algorithm"

struct Environment {
  double x_min = -100, y_min = -100;
  double x_max = 100, y_max = 100;
  std::vector<common::math::Polygon2d> obstacles;
  common::util::PointCloud points;
  VehicleParam vehicle;

  std::array<double, 4> xy_bound() {
    return { x_min, x_max, y_min, y_max };
  }

  // Update boundary values from the PointCloud's computed bounds
  void UpdateBounds() {
    auto bounds = points.bounds();
    x_min = bounds[0];  // min_x
    x_max = bounds[1];  // max_x
    y_min = bounds[2];  // min_y
    y_max = bounds[3];  // max_y
  }

  bool CheckPoseCollision(const common::math::Pose &pose, double radius_buffer = 0.0) {
    common::math::AABox2d initial_box({ -vehicle.radius - radius_buffer, -vehicle.radius - radius_buffer },
                                      { vehicle.radius + radius_buffer, vehicle.radius + radius_buffer });

    // Only use 2 discs: front and rear
    double xf, yf, xr, yr;
    std::tie(xf, yf, xr, yr) = vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());

    auto f_box = initial_box, r_box = initial_box;
    f_box.Shift({ xf, yf });
    r_box.Shift({ xr, yr });
    if(CheckCollision(common::math::Box2d(f_box)) || CheckCollision(common::math::Box2d(r_box))) {
      return true;
    }
    return false;
  }

  bool CheckCollision(const common::math::Box2d &box) {
    if (points.CheckInBox(box)) {
      return true;
    }

    return std::any_of(obstacles.begin(), obstacles.end(), [&](common::math::Polygon2d &ob) {
      return ob.HasOverlap(box);
    });
  }
};

using Env = std::shared_ptr<Environment>;
