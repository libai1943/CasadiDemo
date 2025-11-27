#pragma once

#include "common/vehicle_param.h"

#include "common/util/color.h"
#include "common/math/vec2d.h"
#include "common/math/polygon2d.h"

#include <mutex>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

using common::math::Vec2d;
using common::math::Polygon2d;
using common::util::Color;

namespace VisualizationPlot {
using Vector = std::vector<double>;

void Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic);

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color(1, 1, 1),
          int id = -1, const std::string &ns = "");

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, const std::vector<Color> &color = {},
          int id = -1, const std::string &ns = "");

void PlotPolygon(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotPolygon(const Polygon2d &polygon, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotFilledPolygon(const Vector &xs, const Vector &ys, Color color = Color::Grey,
                       int id = -1, const std::string &ns = "");

void PlotFilledPolygon(const Polygon2d &polygon, Color color = Color::Grey,
                       int id = -1, const std::string &ns = "");

void PlotPath(const Vector &xs, const Vector &ys, const Vector &thetas, double bias = 120.0,
                    double width = 0.1, int id = -1, const std::string &ns = "");

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity = 10.0,
                    double width = 0.1, int id = -1, const std::string &ns = "");

void PlotPoints(const Vector &xs, const Vector &ys, const Color &color = Color::White, double width = 0.1,
                int id = -1, const std::string &ns = "");

void PlotVehicleFootprints(const Vector &xs, const Vector &ys, const Vector &thetas,
                           double length, double width, double rear_to_center,
                           Color color = Color::White, double line_width = 0.05,
                           int id_start = -1, const std::string &ns = "");

void Trigger();

void Clear();
}
