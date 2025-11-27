#pragma once
#include <memory>

#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/math/circle_2d.h"

namespace common {
namespace util {

using common::math::Box2d;
using common::math::Vec2d;
using common::math::Circle2d;


class PointCloud {
public:
  inline void Clear() {
    points_.clear();
    max_x_ = 73;
    min_x_ = -13;
    max_y_ = 18;
    min_y_ = -18;
  }

  void AddParkingSpace(const std::vector<Vec2d> &points) {
    parking_space_.reset();
    AddPoints(points);
    parking_space_ = std::make_shared<common::math::Polygon2d>(points);
  }

  void AddPoints(const std::vector<Vec2d> &points, double sample_step = 0.5);

  bool CheckInBox(const Box2d &box) const;
  bool CheckInCircle(const Circle2d &circle) const;

  const std::vector<Vec2d> &points() {
    return points_;
  }

  void SetPoints(const std::vector<Vec2d> &points);

  inline std::array<double, 4> bounds() {
    return { min_x_, max_x_, min_y_, max_y_ };
  }

private:
  double  max_x_ = 70;
  double  min_x_ = -10;
  double  max_y_ = 12;
  double  min_y_ = -15;
  std::vector<Vec2d> points_;
  std::shared_ptr<common::math::Polygon2d> parking_space_;
};

}
}