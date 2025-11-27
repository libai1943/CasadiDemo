#pragma once

struct TrajectoryNLPConfig {
  int nfe = 101;
  double tf_max = 30.0;

  /**
   * penalty factor to |omega|_2 cost
   */
  double opti_omega = 5.0;

  /**
   * penalty factor to |a|_2 cost
   */
  double opti_a = 1.0;

  /**
   * penalty factor to |phi|_2 cost (steering angle)
   */
  double opti_phi = 1.0;

  /**
   * Maximum iteration number in LIOM
   */
  int opti_iter_max = 100;

  /**
   * Initial value of weighting parameter w_penalty
   */
  double opti_w_penalty0 = 1e3;

  /**
   * Multiplier to enlarge w_penalty during the iterations
   * Reduced from 10 to 3 for smoother convergence
   */
  double opti_alpha = 3.0;

  /**
   * Maximum penalty weight to prevent ill-conditioning
   * Default: 1e7
   */
  double opti_w_penalty_max = 1e7;

  /**
   * Violation tolerance w.r.t. the softened nonlinear constraints
   */
  double opti_varepsilon_tol = 1e-6;

  /**
   * Solution vector norm difference tolerance for convergence
   * Checks if consecutive iterations produce similar solutions
   */
  double opti_solution_norm_tol = 1e-1;

  int iterative_direct_iterations = 10000;
  int iterative_first_iterations = 4000;
  int iterative_rest_iterations = 10000;

  bool use_iterative = true;

  int corridor_max_iter = 1000;
  double corridor_incremental_limit = 20.0;
};