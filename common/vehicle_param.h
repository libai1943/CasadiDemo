#pragma once
#include <tuple>

#include "common/math/pose.h"
#include "common/math/box2d.h"
#include "common/math/circle_2d.h"

class VehicleParam {
public:
  // Car center point is car reference point, i.e., center of rear axle.
  double back_edge_to_center = 0.8;
  // the distance between the front and back wheels
  double wheel_base = 4.0;
  double length = 5.8;
  double width = 2.2;
  double front_edge_to_center = length - back_edge_to_center;
  double center_to_rear = front_edge_to_center - length * 0.5;

  double max_acceleration = 1.5;
  double max_deceleration = -1.5;

  double max_velocity = 15.0 / 3.6;
  // set to zero is reverse is disabled
  double max_reverse_velocity = -15.0 /3.6;

  // vehicle max steer angle
  double max_steer_angle = 0.5;
  // vehicle max steer rate; how fast can the steering wheel turn.
  double max_steer_angle_rate = 0.50;

  // ratio between the turn of steering wheel and the turn of wheels
  double steer_ratio = 1; 

  double radius;
  double f2x, r2x;

  void GenerateDiscs() {
    radius = hypot(0.25 * length, 0.5 * width);
    r2x = 0.25 * length - back_edge_to_center;
    f2x = 0.75 * length - back_edge_to_center;
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
    double distance = length / 2 - back_edge_to_center;
    return { pose.extend(distance), pose.theta(), length, width };
  }

  common::math::Box2d GenerateBoxBuffer(const common::math::Pose &pose) const {
    double distance = length / 2 - back_edge_to_center;
    return { pose.extend(distance), pose.theta(), length + 0.5, width + 0.5 };
  }

};