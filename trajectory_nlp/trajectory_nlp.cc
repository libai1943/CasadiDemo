#include "trajectory_nlp.h"
#include "common/util/time.h"
#include "common/util/csv_logger.h"

namespace trajectory_nlp {

using common::util::NLPOptimizationLogger;

TrajectoryNLP::TrajectoryNLP(const TrajectoryNLPConfig &config, Env env) : config_(config), env_(std::move(env)) {
  nlp_config_ = {{"ipopt", Dict({
    {"linear_solver", "ma27"},
    {"print_level", 3}
  })}};

  buildCommon();
}

void TrajectoryNLP::buildCommon() {
  var_tf_ = SX::sym("tf");
  var_x_ = SX::sym("x", config_.nfe);
  var_y_ = SX::sym("y", config_.nfe);
  var_theta_ = SX::sym("theta", config_.nfe);
  var_v_ = SX::sym("v", config_.nfe);
  var_phi_ = SX::sym("phi", config_.nfe);
  var_a_ = SX::sym("a", config_.nfe);
  var_omega_ = SX::sym("omega", config_.nfe);
  opti_var_ = SX::vertcat({ var_tf_, var_x_, var_y_, var_theta_, var_v_, var_phi_, var_a_, var_omega_ });

  var_xf_ = SX::sym("xf", config_.nfe);
  var_yf_ = SX::sym("yf", config_.nfe);
  var_xr_ = SX::sym("xr", config_.nfe);
  var_yr_ = SX::sym("yr", config_.nfe);
  iterative_var_ = SX::vertcat({ opti_var_, var_xf_, var_yf_, var_xr_, var_yr_ });

  objective_ = var_tf_
             + config_.opti_phi * sumsqr(var_phi_)
             + config_.opti_a * sumsqr(var_a_)
             + config_.opti_omega * sumsqr(var_omega_);

  auto hi = var_tf_ / (config_.nfe - 1);
  auto prev = Slice(0, config_.nfe - 1);
  auto next = Slice(1, config_.nfe);
  auto g_x_kin = var_x_(next) - (var_x_(prev) + hi * var_v_(prev) * cos(var_theta_(prev)));
  auto g_y_kin = var_y_(next) - (var_y_(prev) + hi * var_v_(prev) * sin(var_theta_(prev)));
  auto g_theta_kin = var_theta_(next) - (var_theta_(prev) + hi * var_v_(prev) * tan(var_phi_(prev)) / env_->vehicle.wheel_base);
  auto g_v_kin = var_v_(next) - (var_v_(prev) + hi * var_a_(prev));
  auto g_phi_kin = var_phi_(next) - (var_phi_(prev) + hi * var_omega_(prev));

  SX xf = var_x_ + env_->vehicle.f2x * cos(var_theta_);
  SX yf = var_y_ + env_->vehicle.f2x * sin(var_theta_);
  SX xr = var_x_ + env_->vehicle.r2x * cos(var_theta_);
  SX yr = var_y_ + env_->vehicle.r2x * sin(var_theta_);

  g_kin_ = SX::vertcat({ g_x_kin, g_y_kin, g_theta_kin, g_v_kin, g_phi_kin });

  g_iterative_kin_ = SX::vertcat({ g_kin_, var_xf_ - xf, var_xr_ - xr, var_yf_ - yf, var_yr_ - yr });

  p_inf_w_ = SX::sym("inf_w");
  auto infeasibility = sumsqr(g_iterative_kin_);
  iterative_objective_ = objective_ + p_inf_w_ * infeasibility;
  infeasibility_evaluator_ = Function("inf", { iterative_var_ }, { infeasibility }, {});

  g_corridor_ = SX::vertcat({ xf, yf, xr, yr });
}

Function TrajectoryNLP::buildNLP() {
  SX g = SX::vertcat({ g_kin_, g_corridor_ });
  SXDict nlp = {{"x", opti_var_}, {"f", objective_}, {"g", g}};

  auto config = nlp_config_;
  if(config_.use_iterative) {
    config["ipopt"] = combine(nlp_config_["ipopt"], {{"max_iter", config_.iterative_direct_iterations * config_.nfe / 100}});
  }
  return nlpsol("solver", "ipopt", nlp, config);
}

Function TrajectoryNLP::buildIterativeNLP(int iter) {
  SX p = SX::vertcat({ p_inf_w_ });
  SXDict nlp = {{ "x", iterative_var_ }, { "p", p }, { "f", iterative_objective_}};

  auto first_config = nlp_config_, rest_config = nlp_config_;
  first_config["ipopt"] = combine(first_config["ipopt"], {{"max_iter", config_.iterative_first_iterations * config_.nfe / 100}});
  rest_config["ipopt"] = combine(rest_config["ipopt"], {{"max_iter", config_.iterative_rest_iterations * config_.nfe / 100}});

  return nlpsol("iterative_first_solver", "ipopt", nlp, iter == 0 ? first_config : rest_config);
}

std::pair<DM, DM> TrajectoryNLP::GetVariableBounds(const Constraints &constraints) const {
  auto identity = DM::ones(config_.nfe, 1);

  DM lb_x, lb_y, lb_theta, lb_v, lb_phi, lb_a, lb_omega;
  DM ub_x, ub_y, ub_theta, ub_v, ub_phi, ub_a, ub_omega;

  lb_x = env_->x_min * identity; ub_x = env_->x_max * identity;
  lb_y = env_->y_min * identity; ub_y = env_->y_max * identity;
  lb_theta = -inf * identity; ub_theta = inf * identity;
  lb_v = env_->vehicle.max_reverse_velocity * identity; ub_v = env_->vehicle.max_velocity * identity;
  lb_phi = -env_->vehicle.max_steer_angle * identity; ub_phi = env_->vehicle.max_steer_angle * identity;
  lb_a = env_->vehicle.max_deceleration * identity; ub_a = env_->vehicle.max_acceleration * identity;
  lb_omega = -env_->vehicle.max_steer_angle_rate * identity; ub_omega = env_->vehicle.max_steer_angle_rate * identity;

  int end = config_.nfe - 1;
  lb_x(0) = ub_x(0) = constraints.start.x;
  lb_y(0) = ub_y(0) = constraints.start.y;
  lb_theta(0) = ub_theta(0) = constraints.start.theta;
  lb_v(0) = ub_v(0) = constraints.start.v;
  lb_phi(0) = ub_phi(0) = constraints.start.phi;
  lb_a(0) = ub_a(0) = constraints.start.a;
  lb_omega(0) = ub_omega(0) = constraints.start.omega;

  lb_x(end) = ub_x(end) = constraints.goal.x;
  lb_y(end) = ub_y(end) = constraints.goal.y;
  lb_theta(end) = ub_theta(end) = constraints.goal.theta;
  lb_v(end) = ub_v(end) = constraints.goal.v;
  lb_phi(end) = ub_phi(end) = constraints.goal.phi;
  lb_a(end) = ub_a(end) = constraints.goal.a;
  lb_omega(end) = ub_omega(end) = constraints.goal.omega;

  DM lbx = DM::vertcat({ 0.1, lb_x, lb_y, lb_theta, lb_v, lb_phi, lb_a, lb_omega });
  DM ubx = DM::vertcat({ config_.tf_max, ub_x, ub_y, ub_theta, ub_v, ub_phi, ub_a, ub_omega });
  return std::make_pair(lbx, ubx);
}

std::pair<DM, DM> TrajectoryNLP::GetCorridorBounds(const Constraints &profile) const {
  DM lb_xf, lb_yf, lb_xr, lb_yr, ub_xf, ub_yf, ub_xr, ub_yr;
  lb_xf = lb_yf = lb_xr = lb_yr = -DM::inf(config_.nfe, 1);
  ub_xf = ub_yf = ub_xr = ub_yr = DM::inf(config_.nfe, 1);

  for(int i = 1; i < config_.nfe; i++) {
    lb_xf(i) = profile.front_bound[i][0];
    ub_xf(i) = profile.front_bound[i][1];
    lb_yf(i) = profile.front_bound[i][2];
    ub_yf(i) = profile.front_bound[i][3];

    lb_xr(i) = profile.rear_bound[i][0];
    ub_xr(i) = profile.rear_bound[i][1];
    lb_yr(i) = profile.rear_bound[i][2];
    ub_yr(i) = profile.rear_bound[i][3];
  }

  return std::make_pair(DM::vertcat({ lb_xf, lb_yf, lb_xr, lb_yr }), DM::vertcat({ ub_xf, ub_yf, ub_xr, ub_yr }));
}

States TrajectoryNLP::GetStatesFromSolution(const DM &solution) const {
  States result;
  result.tf = double(solution(0, 0));
  result.x.resize(config_.nfe); result.y.resize(config_.nfe); result.theta.resize(config_.nfe);
  result.v.resize(config_.nfe); result.phi.resize(config_.nfe); result.a.resize(config_.nfe);
  result.omega.resize(config_.nfe);
  for(size_t i = 0; i < config_.nfe; i++) {
    result.x[i] = double(solution(1 + i, 0));
    result.y[i] = double(solution(1 + config_.nfe + i, 0));
    result.theta[i] = double(solution(1 + 2 * config_.nfe + i, 0));
    result.v[i] = double(solution(1 + 3 * config_.nfe + i, 0));
    result.phi[i] = double(solution(1 + 4 * config_.nfe + i, 0));
    result.a[i] = double(solution(1 + 5 * config_.nfe + i, 0));
    result.omega[i] = double(solution(1 + 6 * config_.nfe + i, 0));
  }
  return result;
}

bool TrajectoryNLP::Solve(const Constraints &profile, const States &guess, States &result) {
  auto eq_bound = DM::zeros((config_.nfe - 1) * 5);

  auto var_bounds = GetVariableBounds(profile);
  auto corridor_bounds = GetCorridorBounds(profile);

  DMDict arg, res;
  arg["lbx"] = var_bounds.first;
  arg["ubx"] = var_bounds.second;
  arg["lbg"] = DM::vertcat({ eq_bound, corridor_bounds.first });
  arg["ubg"] = DM::vertcat({ eq_bound, corridor_bounds.second });
  arg["x0"] = DM::vertcat({ guess.tf, guess.x, guess.y, guess.theta, guess.v, guess.phi, guess.a, guess.omega });

  auto solver = buildNLP();
  res = solver(arg);

  bool is_success = solver.stats()["success"];

  DM opt = res.at("x");
  result = GetStatesFromSolution(opt);

  result.t = guess.t;

  return is_success;
}

bool TrajectoryNLP::SolveIteratively(
  int iter, double w_inf, const Constraints &profile, const States &guess, const States &reference, States &result, double &infeasibility) {

  if (iter == 0) {
    NLPOptimizationLogger::LogInitialGuess(guess.tf, guess.x, guess.y, guess.theta,
                                           guess.v, guess.phi, guess.a, guess.omega);

#if ENABLE_INITIAL_GUESS_INFEAS_CALC
    std::vector<double> xf_init(config_.nfe), yf_init(config_.nfe),
                        xr_init(config_.nfe), yr_init(config_.nfe);
    for(int i = 0; i < config_.nfe; i++) {
      std::tie(xf_init[i], yf_init[i], xr_init[i], yr_init[i]) =
        env_->vehicle.GetDiscPositions(guess.x[i], guess.y[i], guess.theta[i]);
    }

    DM guess_solution = DM::vertcat({ guess.tf, guess.x, guess.y, guess.theta,
                                      guess.v, guess.phi, guess.a, guess.omega,
                                      xf_init, yf_init, xr_init, yr_init });

    std::vector<DM> infeas_arg = { guess_solution };
    auto infeas_result = infeasibility_evaluator_(infeas_arg);
    double initial_infeasibility = infeas_result.front()->at(0);

    NLPOptimizationLogger::LogInitialGuessInfeasibility(initial_infeasibility);
#endif
  }

  auto var_bounds = GetVariableBounds(profile);
  auto corridor_bounds = GetCorridorBounds(profile);

  std::vector<double> xf(config_.nfe), yf(config_.nfe), xr(config_.nfe), yr(config_.nfe);
  for(int i = 0; i < config_.nfe; i++) {
    std::tie(xf[i], yf[i], xr[i], yr[i]) = env_->vehicle.GetDiscPositions(guess.x[i], guess.y[i], guess.theta[i]);
  }

  DMDict arg, res;
  arg["lbx"] = DM::vertcat({ var_bounds.first, corridor_bounds.first });
  arg["ubx"] = DM::vertcat({ var_bounds.second, corridor_bounds.second });
  arg["lbg"] = DM::vertcat({  });
  arg["ubg"] = DM::vertcat({  });
  arg["x0"] = DM::vertcat({ guess.tf, guess.x, guess.y, guess.theta, guess.v, guess.phi, guess.a, guess.omega, xf, yf, xr, yr });
  arg["p"] = DM::vertcat({ w_inf });

  auto solver = buildIterativeNLP(iter);
  res = solver(arg);

  bool is_success = solver.stats()["success"];

  DM opt = res.at("x");
  result = GetStatesFromSolution(opt);

  result.t = guess.t;

  std::vector<DM> arg_in = { opt };
  auto arg_out = infeasibility_evaluator_(arg_in);
  infeasibility = arg_out.front()->at(0);

  NLPOptimizationLogger::LogIterationResult(iter, infeasibility, result.tf,
                                            result.x, result.y, result.theta,
                                            result.v, result.phi, result.a, result.omega);

#if ENABLE_DETAILED_OPT_LOGGING
  double cost_value = 0.0;
  if (res.count("f")) {
    DM cost_dm = res.at("f");
    cost_value = static_cast<double>(cost_dm.get_elements()[0]);
  }

  std::vector<double> constraint_violations;
  std::vector<double> constraint_bounds;

  if (res.count("g")) {
    DM g = res.at("g");
    std::vector<double> g_values = g.get_elements();

    DM ubg = arg.at("ubg");
    std::vector<double> ubg_values = ubg.get_elements();

    DM lbg = arg.at("lbg");
    std::vector<double> lbg_values = lbg.get_elements();

    for (size_t i = 0; i < g_values.size(); ++i) {
      double violation = 0.0;
      if (g_values[i] > ubg_values[i]) {
        violation = g_values[i] - ubg_values[i];
      } else if (g_values[i] < lbg_values[i]) {
        violation = lbg_values[i] - g_values[i];
      }
      constraint_violations.push_back(violation);
      constraint_bounds.push_back(ubg_values[i]);
    }
  }

  NLPOptimizationLogger::LogIterationDetails(iter, cost_value,
                                              constraint_violations,
                                              constraint_bounds);
#endif

  return is_success;
}


}
