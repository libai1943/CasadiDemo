#include <common/math/math_utils.h>
#include "visualization_plot.h"

namespace VisualizationPlot {
namespace {
std::string frame_ = "map";
std::mutex mutex_;

ros::Publisher publisher_;
visualization_msgs::MarkerArray arr_;
}
}

void VisualizationPlot::Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic) {
  frame_ = frame;
  publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 1000, true);
}

void
VisualizationPlot::Plot(const Vector &xs, const Vector &ys, double width, Color color, int id, const std::string &ns) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    pt.z = 0.1 * id;
    msg.points.push_back(pt);
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}

void VisualizationPlot::Plot(const VisualizationPlot::Vector &xs, const VisualizationPlot::Vector &ys, double width,
                             const std::vector <Color> &color, int id, const std::string &ns) {
  assert(xs.size() == color.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(color[i].toColorRGBA());
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}


void VisualizationPlot::PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color, int id,
                                    const std::string &ns) {
  auto xxs = xs;
  auto yys = ys;
  xxs.push_back(xxs[0]);
  yys.push_back(yys[0]);
  Plot(xxs, yys, width, color, id, ns);
}

void VisualizationPlot::PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                                    const std::string &ns) {
  std::vector<double> xs, ys;
  for (auto &pt: polygon.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  PlotPolygon(xs, ys, width, color, id, ns);
}

void VisualizationPlot::PlotFilledPolygon(const Vector &xs, const Vector &ys, Color color, int id,
                                          const std::string &ns) {
  if (xs.size() < 3) return;  // Need at least 3 points for a polygon

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = msg.scale.z = 1.0;
  msg.color = color.toColorRGBA();

  // Fan triangulation from first vertex
  for (size_t i = 1; i < xs.size() - 1; i++) {
    // Triangle: (0, i, i+1)
    geometry_msgs::Point pt0, pt1, pt2;
    pt0.x = xs[0];
    pt0.y = ys[0];
    pt0.z = 0.1 * id;

    pt1.x = xs[i];
    pt1.y = ys[i];
    pt1.z = 0.1 * id;

    pt2.x = xs[i + 1];
    pt2.y = ys[i + 1];
    pt2.z = 0.1 * id;

    msg.points.push_back(pt0);
    msg.points.push_back(pt1);
    msg.points.push_back(pt2);
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}

void VisualizationPlot::PlotFilledPolygon(const Polygon2d &polygon, Color color, int id,
                                          const std::string &ns) {
  std::vector<double> xs, ys;
  for (auto &pt: polygon.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  PlotFilledPolygon(xs, ys, color, id, ns);
}

void VisualizationPlot::PlotPath(const Vector &xs, const Vector &ys, const Vector &thetas, double bias, double width, int id,
                                 const std::string &ns) {
  std::vector<Color> colors(xs.size());
  auto gears = common::math::GetPathGears(xs, ys, thetas);

  for (size_t i = 0; i < xs.size()-1; i++) {
    colors[i] = Color::fromHSV(2 * bias - (gears[i] ? 1.0 : -1.0) * bias, 1.0, 1.0);
    colors[i].set_a(std::min(1.0, width / 0.02));
  }
  colors[colors.size()-1] = colors.back();

  Plot(xs, ys, width, colors, id, ns);
}

void VisualizationPlot::PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity,
                                       double width, int id, const std::string &ns) {
  std::vector<Color> colors(xs.size());

  for (size_t i = 0; i < xs.size(); i++) {
    double percent = (vs[i] / max_velocity);
    colors[i] = Color::fromHSV(120.0f - percent * 120, 1.0, 1.0);
  }

  Plot(xs, ys, width, colors, id, ns);
}

void VisualizationPlot::PlotPoints(const Vector &xs, const Vector &ys, const Color &color, double width, int id,
                                   const std::string &ns) {
  assert(xs.size() == ys.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = !xs.empty() ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
  msg.type = visualization_msgs::Marker::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}

void VisualizationPlot::PlotVehicleFootprints(const Vector &xs, const Vector &ys, const Vector &thetas,
                                               double length, double width, double rear_to_center,
                                               Color color, double line_width, int id_start,
                                               const std::string &ns) {
  assert(xs.size() == ys.size() && xs.size() == thetas.size());

  // Draw vehicle rectangle at EACH configuration point
  // User requirement: 每个配置点都把对应的车身的矩形画出来
  for (size_t i = 0; i < xs.size(); i++) {
    double x = xs[i];
    double y = ys[i];
    double theta = thetas[i];

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    // Vehicle coordinate system:
    // - (x, y) = reference point (typically rear axle center)
    // - theta = vehicle heading
    // - rear_to_center = distance from rear bumper to reference point
    // - length = total vehicle length (rear bumper to front bumper)
    // - width = total vehicle width

    double half_width = width / 2.0;

    // Distance from reference point to rear bumper (negative, behind the vehicle)
    double rear_offset = -rear_to_center;
    // Distance from reference point to front bumper
    double front_offset = length - rear_to_center;

    // Four corners of the vehicle rectangle
    std::vector<double> corner_xs(4), corner_ys(4);

    // Rear-left corner
    corner_xs[0] = x + rear_offset * cos_theta - half_width * sin_theta;
    corner_ys[0] = y + rear_offset * sin_theta + half_width * cos_theta;

    // Rear-right corner
    corner_xs[1] = x + rear_offset * cos_theta + half_width * sin_theta;
    corner_ys[1] = y + rear_offset * sin_theta - half_width * cos_theta;

    // Front-right corner
    corner_xs[2] = x + front_offset * cos_theta + half_width * sin_theta;
    corner_ys[2] = y + front_offset * sin_theta - half_width * cos_theta;

    // Front-left corner
    corner_xs[3] = x + front_offset * cos_theta - half_width * sin_theta;
    corner_ys[3] = y + front_offset * sin_theta + half_width * cos_theta;

    int marker_id = (id_start >= 0) ? (id_start + static_cast<int>(i)) : -1;
    PlotPolygon(corner_xs, corner_ys, line_width, color, marker_id, ns);
  }
}

void VisualizationPlot::Trigger() {
  mutex_.lock();
  publisher_.publish(arr_);
  arr_.markers.clear();
  ros::spinOnce();
  mutex_.unlock();
}

void VisualizationPlot::Clear() {
  mutex_.lock();
  arr_.markers.clear();

  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.ns = "Markers";

  msg.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(msg);
  publisher_.publish(arr);
  mutex_.unlock();
}
