#pragma once

#include "trajectory_nlp.h"
#include "trajectory_nlp_config.h"

#include "common/math/aabox2d.h"
#include "common/math/polygon2d.h"

namespace trajectory_nlp {

using common::math::Polygon2d;
using common::math::AABox2d;
using common::math::Box2d;

struct CoarseDecision {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> theta;
};


class TrajectoryOptimizer {
public:
  TrajectoryOptimizer(const TrajectoryNLPConfig &config, Env env);

  bool Optimize(const CoarseDecision &decision, const TrajectoryPoint &start, const TrajectoryPoint &goal, States &result);

private:
  TrajectoryNLPConfig config_;
  Env env_;
  VehicleParam vehicle_;
  TrajectoryNLP nlp_;

  // 1.
  States ResampleCoarsePath(CoarseDecision &coarse);

  // for 1.
  std::vector<double> GenerateOptimalTimeProfile(const CoarseDecision &coarse);

  std::vector<double> GenerateOptimalTimeProfileSegment(int gear, const std::vector<double> &stations, double start_time) const;

  // 2.
  void CalculateInitialGuess(States &states) const;

  // 3.
  bool FormulateCorridorConstraints(const States &states, Constraints &constraints, bool log_to_csv = false);

  bool GenerateAABox(double &x, double &y, double radius, AABox2d &box) const;

  // Helper function to compute L2 norm difference between two solution vectors
  double ComputeSolutionNormDifference(const States &states1, const States &states2) const;

};

}