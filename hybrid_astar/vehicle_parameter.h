#pragma once
#define  _USE_MATH_DEFINES
#include <math.h>
#include <tuple>
#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/math/pose.h"
#include "common/math/circle_2d.h"

struct VehicleParameter {
  double wheel_base = 2.8;
  double front_hang = 0.96;
  double rear_hang = 0.929; // rear collision distance to rear axle center
  double width = 1.942;
  double v_max = 2.5;
  double a_max = 0.5;
  double delta_max = 0.7;
  double omega_max = 0.5;

  double steer_ratio = 1.0;

  double length = 0;
  double radius = 0;
  double r2x = 0, f2x = 0;

  void GenerateDisc() {
    length = rear_hang + wheel_base + front_hang;
    radius = hypot(0.25 * length, 0.5 * width);
    r2x = 0.25 * length - rear_hang;
    f2x = 0.75 * length - rear_hang;
  }

  template<class T>
  std::tuple<T, T, T, T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
    auto xf = x + f2x * cos(theta);
    auto xr = x + r2x * cos(theta);
    auto yf = y + f2x * sin(theta);
    auto yr = y + r2x * sin(theta);
    return std::make_tuple(xf, yf, xr, yr);
  }

  std::pair<common::math::Circle2d, common::math::Circle2d> GetDiscs(double x, double y, double theta) const {
    double xf, yf, xr, yr;
    std::tie(xf, yf, xr, yr) = GetDiscPositions(x, y, theta);
    return std::make_pair(common::math::Circle2d(xf, yf, radius), common::math::Circle2d(xr, yr, radius));
  }

  common::math::Box2d GenerateBox(const common::math::Pose &pose) const {
    double distance = length / 2 - rear_hang;
    return common::math::Box2d(pose.extend(distance), pose.theta(), length, width);
  }
};
