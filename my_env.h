#pragma once
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

#include "common/math/vec2d.h"
#include "common/math/pose.h"
#include "common/math/polygon2d.h"
#include "common/trajectory_point.h"

#include "visualization_plot.h"
#include "trajectory_nlp/trajectory_nlp.h"

using json = nlohmann::json;

namespace common {
namespace math {

inline void to_json(json& j, const Vec2d& p) { j = json{p.x(), p.y()}; }
inline void from_json(const json& j, Vec2d& p) { p.set_x(j[0]); p.set_y(j[1]); }

inline void to_json(json& j, const Pose& p) { j = json{p.x(), p.y(), p.theta()}; }
inline void from_json(const json& j, Pose& p) { p.set_x(j[0]); p.set_y(j[1]); p.set_theta(j[2]); }

inline void to_json(json& j, const Polygon2d& p) { j = p.points(); }
inline void from_json(const json& j, Polygon2d& p) { p = Polygon2d(j.get<std::vector<Vec2d>>()); }

}
}

inline void to_json(json& j, const TrajectoryPoint& p) { j["x"] = p.x; j["y"] = p.y; j["theta"] = p.theta; j["v"] = p.v; j["phi"] = p.phi; j["a"] = p.a; j["omega"] = p.omega; }
inline void from_json(const json& j, TrajectoryPoint& p) { p.x = j["x"]; p.y = j["y"]; p.theta = j["theta"]; p.v = j["v"]; p.phi = j["phi"]; p.a = j["a"]; p.omega = j["omega"]; }

struct MyEnvironment {
  std::vector<common::math::Vec2d> points;
  std::vector<common::math::Polygon2d> obstacles;
  common::math::Pose start, goal;
  TrajectoryPoint current;
  Trajectory last_trajectory;
  int parking_type = 1;

  MyEnvironment() = default;

  MyEnvironment(const std::vector<common::math::Vec2d> &points, const std::vector<common::math::Polygon2d> &obstacles)
  : points(points), obstacles(obstacles) {}

  bool Read(const std::string &env_file);

  void Save(const std::string &env_file) const;

  void CollectResult(const std::string &env_file, const std::string &tag, double run_time) const;

  void SaveCSV() {
    std::ofstream os("/home/yssun/openspace_ws/border.csv");
    if(os.is_open()) {
      for(auto &pt: points) {
        os << pt.x() << "," << pt.y() << std::endl;
      }
    }
  }

  void Visualize() const {
    for(int i = 0; i < obstacles.size(); i++) {
      VisualizationPlot::PlotFilledPolygon(obstacles[i], Color::Grey, i, "Obstacles");
    }
    std::vector<double> xs, ys;
    for(auto & i : points) {
      xs.push_back(i.x());
      ys.push_back(i.y());
    }
    VisualizationPlot::PlotPoints(xs, ys, Color::Black, 0.2, 1, "Border");

    VisualizationPlot::Trigger();
  }
};

inline void to_json(json& j, const MyEnvironment& p) {
  j["border"] = p.points;
  j["obstacles"] = p.obstacles;
  j["start"] = p.start;
  j["goal"] = p.goal;
  j["last_trajectory"] = p.last_trajectory;
  j["current"] = p.current;
  j["parking_type"] = p.parking_type;
}

inline void from_json(const json& j, MyEnvironment& p) {
  // Support both "border" and "boundary_points" field names for backward compatibility
  if(j.contains("border")) {
    j["border"].get_to(p.points);
  } else if(j.contains("boundary_points")) {
    j["boundary_points"].get_to(p.points);
  }

  j["obstacles"].get_to(p.obstacles);

  // Make "start" optional - only parse if exists
  if(j.contains("start")) {
    j["start"].get_to(p.start);
  }

  j["goal"].get_to(p.goal);

  if(j.contains("last_trajectory")) {
    j["last_trajectory"].get_to(p.last_trajectory);
  }
  if(j.contains("current")) {
    j["current"].get_to(p.current);
  }

  // Make "parking_type" optional with default value
  if(j.contains("parking_type")) {
    j["parking_type"].get_to(p.parking_type);
  } else {
    p.parking_type = 1;  // Default value
  }
}