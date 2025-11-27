#pragma once
#include "common/math/pose.h"
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <iomanip>

struct TrajectoryPoint {
  double x = 0.0, y = 0.0, theta = 0.0, v = 0.0, phi = 0.0, a = 0.0, omega = 0.0, kappa = 0.0, jerk = 0.0;
  double t = 0.0, s = 0.0;  // 时间和里程
  TrajectoryPoint() = default;
  explicit TrajectoryPoint(const common::math::Pose &pose): x(pose.x()), y(pose.y()), theta(pose.theta()) {}

  explicit operator common::math::Pose() const {
    return {x, y, theta};
  }
};

using Trajectory = std::vector<TrajectoryPoint>;

// Define the TrajectorySegment structure
struct TrajectorySegment {
    Trajectory points;
    uint8_t gear = 1;  // gear values: 1 - forward, 2 - reverse, 3 - stationary
    double segement_tf = 0;
    double segement_t0 = 0;
    int index_init;
    int index_final;
};

static inline void ComputeTimeAndDistance(Trajectory& trajectory, double tf, int nfe) {
    if (trajectory.empty()) {
        return;
    }

    double delta_t = tf / (nfe-1);

    // Use relative time starting from 0, not Unix timestamp
    trajectory[0].t = 0.0;
    trajectory[0].s = 0.0;

    // trajectory[1].t = 0.0;
    // trajectory[1].s = 0.0;

    // int sharpIndexOffset = 1;

    for (int i = 1; i < trajectory.size(); ++i) {
        // bool isSharpPoint = ((trajectory[i].v * trajectory[i-1].v < 0) && std::fabs(trajectory[i].v) > 1e-4 && std::fabs(trajectory[i-1].v) > 1e-4);
        
        // if (isSharpPoint) {
        //     trajectory[i].t = delta_t;
        //     trajectory[i].s = hypot(trajectory[i].x-trajectory[i-1].x, trajectory[i].y-trajectory[i-1].y);
        //     sharpIndexOffset = i-1;
        // } else {
        trajectory[i].t = delta_t * i + trajectory[0].t;
        trajectory[i].s = trajectory[i-1].s + hypot(trajectory[i].x-trajectory[i-1].x, trajectory[i].y-trajectory[i-1].y);
        // }

        // 调试信息
        // std::cout << "Index: " << i << " | v: " << trajectory[i].v << " | v_prev: " << trajectory[i-1].v << " | a: " << trajectory[i].a << std::endl;
        // std::cout << "tt: "  << std::fixed << std::setprecision(11) << trajectory[i].t << std::endl;
        // std::cout << "ss: " << trajectory[i].s << std::endl;
    }
    // trajectory.back().t = tf;
    // trajectory.back().s = (trajectory.rbegin() + 1)->s;
}


static inline std::vector<TrajectorySegment> SplitTrajectoryByGear(const Trajectory& trajectory) {
    std::vector<TrajectorySegment> segments;
    std::vector<TrajectorySegment> segments_expand;
    if (trajectory.empty()) {
        return segments;
    }

    TrajectorySegment currentSegment;
    int break_index = 0;
    bool first_segment = true;
    for (int i = 0; i < trajectory.size(); ++i) {
        if (trajectory[i].v >= -1e-6 && trajectory[i].v <= 1e-6) {
            break_index = i;
        }
        currentSegment.points.push_back(trajectory[i]);
    }
    currentSegment.gear = trajectory[break_index].v >= 1e-6 ? 1 : 2;

    currentSegment.index_init = 0;
    
    for (size_t i = break_index; i < trajectory.size(); ++i) {
        uint8_t currentGear = trajectory[i].v > 1e-6 ? 1 : (trajectory[i].v < 1e-6 ? 2 : 3);
        if (currentGear == 3 && first_segment) {
            currentGear = currentSegment.gear;
        }
        double currentTime = trajectory[i].t;
        if (currentGear != currentSegment.gear) {
            currentSegment.index_final = i-1;
            segments.push_back(currentSegment);
            first_segment = false;
            currentSegment.points.clear();
            currentSegment.gear = currentGear;
            currentSegment.segement_tf = currentTime;
            
            currentSegment.index_init = i;
        }
        currentSegment.points.push_back(trajectory[i]);
    }

    if (!currentSegment.points.empty()) {
        segments.push_back(currentSegment);
    }
    
    return segments;
}

// double distance(const TrajectoryPoint& p1, const TrajectoryPoint& p2) {
//     return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
// }

// TrajectoryPoint interpolate(const TrajectoryPoint& p1, const TrajectoryPoint& p2, double ratio) {
//     TrajectoryPoint result;
//     result.x = p1.x + (p2.x - p1.x) * ratio;
//     result.y = p1.y + (p2.y - p1.y) * ratio;
//     // 线性插值其他属性
//     result.v = p1.v + (p2.v - p1.v) * ratio;
//     result.phi = p1.phi + (p2.phi - p1.phi) * ratio;
//     result.a = p1.a + (p2.a - p1.a) * ratio;
//     result.omega = p1.omega + (p2.omega - p1.omega) * ratio;
//     result.kappa = p1.kappa + (p2.kappa - p1.kappa) * ratio;
//     result.jerk = p1.jerk + (p2.jerk - p1.jerk) * ratio;
//     return result;
// }

// std::vector<TrajectoryPoint> discretize(const std::vector<TrajectoryPoint>& points, double desired_spacing) {
//     std::vector<TrajectoryPoint> new_points;
//     for (size_t i = 0; i < points.size() - 1; ++i) {
        
//         const TrajectoryPoint& p1 = points[i];
//         const TrajectoryPoint& p2 = points[i + 1];
//         new_points.push_back(p1);

//         double dist = distance(p1, p2);
//         int num_inserts = static_cast<int>(dist / desired_spacing);

//         for (int j = 1; j <= num_inserts; ++j) {
//             double ratio = static_cast<double>(j) / (num_inserts + 1);
//             new_points.push_back(interpolate(p1, p2, ratio));
//         }
//     }
//     new_points.push_back(points.back()); // 添加最后一个点
//     return new_points;
// }

