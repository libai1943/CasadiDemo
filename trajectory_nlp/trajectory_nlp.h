#pragma once
#include <array>
#include <unordered_map>
#include <casadi/casadi.hpp>
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/trajectory_point.h"
#include "common/environment.h"
#include "trajectory_nlp_config.h"


namespace trajectory_nlp {

using namespace casadi;
using common::math::Polygon2d;
using common::math::Pose;

struct States {
  double tf;
  std::vector<double> t;  // Time stamps for each point
  // Control inputs: a (acceleration) and omega (steering rate)
  std::vector<double> x, y, theta, v, phi, a, omega;
};

struct Constraints {
  TrajectoryPoint start, goal;
  std::vector<std::array<double, 4>> front_bound;  // Only 2 discs: front and rear
  std::vector<std::array<double, 4>> rear_bound;
  std::vector<bool> gears;
};

class TrajectoryNLP {
public:
  explicit TrajectoryNLP(const TrajectoryNLPConfig &, Env);

  bool Solve(const Constraints &profile, const States &guess, States &result);

  bool SolveIteratively(
    int iter, double w_inf, const Constraints &profile, const States &guess, const States &reference, States &result, double &infeasibility);

private:
  TrajectoryNLPConfig config_;
  Env env_;
  Dict nlp_config_;
  SX objective_, iterative_objective_;
  SX var_tf_, var_x_, var_y_, var_theta_, var_v_, var_phi_, var_a_, var_omega_;
  SX opti_var_, iterative_var_;
  SX var_xf_, var_xr_, var_yf_, var_yr_;  // Only 2 discs: front and rear
  SX g_kin_, g_iterative_kin_;
  SX g_corridor_, g_vw_;
  SX p_inf_w_;

  Function infeasibility_evaluator_;

  void buildCommon();

  Function buildNLP();
  Function buildIterativeNLP(int iter);

  std::pair<DM, DM> GetVariableBounds(const Constraints &constraints) const;
  std::pair<DM, DM> GetCorridorBounds(const Constraints &profile) const;

  States GetStatesFromSolution(const DM &solution) const;
};

}

