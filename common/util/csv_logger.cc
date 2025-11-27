#include "common/util/csv_logger.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <chrono>
#include <ctime>
#include <iostream>
#include <map>

namespace common {
namespace util {

// ============================================================
// CSVLogger 静态成员初始化
// ============================================================
std::map<std::string, std::ofstream> CSVLogger::files_;
std::string CSVLogger::output_directory_ = "/home/yssun/Workspace/openspace_ws/src/parking_planner/log";

// ============================================================
// CSVLogger 实现
// ============================================================

// 递归创建目录
static bool CreateDirectoryRecursive(const std::string& path) {
  if (path.empty()) return false;

  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    if (info.st_mode & S_IFDIR) {
      return true; // 目录已存在
    } else {
      std::cerr << "Path exists but is not a directory: " << path << std::endl;
      return false;
    }
  }

  // 找到父目录
  size_t pos = path.find_last_of('/');
  if (pos != std::string::npos && pos > 0) {
    std::string parent = path.substr(0, pos);
    if (!CreateDirectoryRecursive(parent)) {
      return false;
    }
  }

  // 创建当前目录
  if (mkdir(path.c_str(), 0755) != 0) {
    std::cerr << "Failed to create directory: " << path << std::endl;
    return false;
  }

  std::cout << "Created directory: " << path << std::endl;
  return true;
}

void CSVLogger::EnsureDirectoryExists(const std::string& dir) {
#if ENABLE_CSV_LOGGING
  if (!CreateDirectoryRecursive(dir)) {
    std::cerr << "ERROR: Failed to create directory: " << dir << std::endl;
  }
#endif
}

void CSVLogger::SetOutputDirectory(const std::string& dir) {
#if ENABLE_CSV_LOGGING
  output_directory_ = dir;
  EnsureDirectoryExists(dir);
#endif
}

std::string CSVLogger::GetFullPath(const std::string& filename) {
  return output_directory_ + "/" + filename;
}

std::string CSVLogger::CreateTimestampedFilename(const std::string& prefix,
                                                  const std::string& suffix) {
#if ENABLE_CSV_LOGGING
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;

  std::tm tm_now;
  localtime_r(&time_t_now, &tm_now);

  std::ostringstream oss;
  oss << prefix << "_"
      << std::put_time(&tm_now, "%Y%m%d_%H%M%S")
      << "_" << std::setfill('0') << std::setw(3) << ms.count()
      << suffix;
  return oss.str();
#else
  return "";
#endif
}

bool CSVLogger::Open(const std::string& filename,
                     const std::vector<std::string>& header) {
#if ENABLE_CSV_LOGGING
  EnsureDirectoryExists(output_directory_);

  std::string full_path = GetFullPath(filename);

  // 如果文件已经打开，先关闭
  if (files_.find(filename) != files_.end() && files_[filename].is_open()) {
    files_[filename].close();
  }

  files_[filename].open(full_path, std::ios::out | std::ios::trunc);
  if (!files_[filename].is_open()) {
    std::cerr << "Failed to open CSV file: " << full_path << std::endl;
    return false;
  }

  // 写入表头
  for (size_t i = 0; i < header.size(); ++i) {
    files_[filename] << header[i];
    if (i < header.size() - 1) {
      files_[filename] << ",";
    }
  }
  files_[filename] << "\n";
  files_[filename].flush();

  std::cout << "CSV file opened: " << full_path << std::endl;
  return true;
#else
  return true;
#endif
}

void CSVLogger::WriteLine(const std::string& filename,
                         const std::vector<double>& values) {
#if ENABLE_CSV_LOGGING
  if (files_.find(filename) == files_.end() || !files_[filename].is_open()) {
    std::cerr << "CSV file not open: " << filename << std::endl;
    return;
  }

  for (size_t i = 0; i < values.size(); ++i) {
    files_[filename] << std::setprecision(10) << values[i];
    if (i < values.size() - 1) {
      files_[filename] << ",";
    }
  }
  files_[filename] << "\n";
  files_[filename].flush();
#endif
}

void CSVLogger::WriteLine(const std::string& filename,
                         const std::vector<std::string>& values) {
#if ENABLE_CSV_LOGGING
  if (files_.find(filename) == files_.end() || !files_[filename].is_open()) {
    std::cerr << "CSV file not open: " << filename << std::endl;
    return;
  }

  for (size_t i = 0; i < values.size(); ++i) {
    files_[filename] << values[i];
    if (i < values.size() - 1) {
      files_[filename] << ",";
    }
  }
  files_[filename] << "\n";
  files_[filename].flush();
#endif
}

void CSVLogger::Close(const std::string& filename) {
#if ENABLE_CSV_LOGGING
  if (files_.find(filename) != files_.end() && files_[filename].is_open()) {
    files_[filename].close();
    std::cout << "CSV file closed: " << filename << std::endl;
  }
#endif
}

void CSVLogger::CloseAll() {
#if ENABLE_CSV_LOGGING
  for (auto& pair : files_) {
    if (pair.second.is_open()) {
      pair.second.close();
    }
  }
  files_.clear();
  std::cout << "All CSV files closed." << std::endl;
#endif
}

// ============================================================
// HybridAStarLogger 静态成员初始化
// ============================================================
std::string HybridAStarLogger::run_id_ = "";
bool HybridAStarLogger::initialized_ = false;

// ============================================================
// HybridAStarLogger 实现
// ============================================================

void HybridAStarLogger::Initialize(const std::string& run_id) {
#if ENABLE_CSV_LOGGING
  if (initialized_) {
    Finalize();
  }

  if (run_id.empty()) {
    run_id_ = CSVLogger::CreateTimestampedFilename("hastar", "");
  } else {
    run_id_ = run_id;
  }

  CSVLogger::SetOutputDirectory("/home/yssun/Workspace/openspace_ws/src/parking_planner/log/hybrid_astar");

  // 搜索节点文件
  CSVLogger::Open(run_id_ + "_search_nodes.csv",
                 {"x", "y", "phi", "traj_cost", "heuristic_cost",
                  "total_cost", "direction", "steering", "iteration"});

  // Reed-Shepp路径文件（展开的点）
  CSVLogger::Open(run_id_ + "_rs_path_points.csv",
                 {"point_index", "x", "y", "phi", "gear", "path_id"});

  // Reed-Shepp路径统计
  CSVLogger::Open(run_id_ + "_rs_path_summary.csv",
                 {"path_id", "total_length", "num_points"});

  // 最终粗路径
  CSVLogger::Open(run_id_ + "_coarse_path.csv",
                 {"index", "x", "y", "phi", "v", "a", "steer"});

  initialized_ = true;
  std::cout << "HybridAStarLogger initialized with run_id: " << run_id_ << std::endl;
#endif
}

void HybridAStarLogger::LogSearchNode(double x, double y, double phi,
                                      double traj_cost, double heuristic_cost,
                                      bool direction, double steering,
                                      int iteration) {
#if ENABLE_CSV_LOGGING
  if (!initialized_) {
    Initialize();
  }

  std::vector<double> values = {
    x, y, phi, traj_cost, heuristic_cost,
    traj_cost + heuristic_cost,
    direction ? 1.0 : 0.0,
    steering,
    static_cast<double>(iteration)
  };

  CSVLogger::WriteLine(run_id_ + "_search_nodes.csv", values);
#endif
}

void HybridAStarLogger::LogReedSheppPath(const std::vector<double>& x,
                                         const std::vector<double>& y,
                                         const std::vector<double>& phi,
                                         const std::vector<bool>& gear,
                                         double total_length) {
#if ENABLE_CSV_LOGGING
  if (!initialized_) {
    Initialize();
  }

  static int path_id = 0;
  path_id++;

  // 记录路径统计信息
  CSVLogger::WriteLine(run_id_ + "_rs_path_summary.csv",
                      {static_cast<double>(path_id), total_length,
                       static_cast<double>(x.size())});

  // 记录路径的每个点
  for (size_t i = 0; i < x.size(); ++i) {
    std::vector<double> values = {
      static_cast<double>(i), x[i], y[i], phi[i],
      gear[i] ? 1.0 : 0.0,
      static_cast<double>(path_id)
    };
    CSVLogger::WriteLine(run_id_ + "_rs_path_points.csv", values);
  }
#endif
}

void HybridAStarLogger::LogCoarsePath(const std::vector<double>& x,
                                      const std::vector<double>& y,
                                      const std::vector<double>& phi,
                                      const std::vector<double>& v,
                                      const std::vector<double>& a,
                                      const std::vector<double>& steer) {
#if ENABLE_CSV_LOGGING
  if (!initialized_) {
    Initialize();
  }

  size_t n = x.size();
  for (size_t i = 0; i < n; ++i) {
    std::vector<double> values = {
      static_cast<double>(i),
      x[i], y[i], phi[i],
      i < v.size() ? v[i] : 0.0,
      i < a.size() ? a[i] : 0.0,
      i < steer.size() ? steer[i] : 0.0
    };
    CSVLogger::WriteLine(run_id_ + "_coarse_path.csv", values);
  }
#endif
}

void HybridAStarLogger::Finalize() {
#if ENABLE_CSV_LOGGING
  if (initialized_) {
    CSVLogger::CloseAll();
    initialized_ = false;
    std::cout << "HybridAStarLogger finalized." << std::endl;
  }
#endif
}

// ============================================================
// NLPOptimizationLogger 静态成员初始化
// ============================================================
std::string NLPOptimizationLogger::run_id_ = "";
bool NLPOptimizationLogger::initialized_ = false;
int NLPOptimizationLogger::current_iteration_ = 0;

// ============================================================
// NLPOptimizationLogger 实现
// ============================================================

void NLPOptimizationLogger::Initialize(const std::string& run_id) {
#if ENABLE_CSV_LOGGING
  if (initialized_) {
    Finalize();
  }

  if (run_id.empty()) {
    run_id_ = CSVLogger::CreateTimestampedFilename("nlp", "");
  } else {
    run_id_ = run_id;
  }

  CSVLogger::SetOutputDirectory("/home/yssun/Workspace/openspace_ws/src/parking_planner/log/nlp_optimization");

  // 初始猜测
  CSVLogger::Open(run_id_ + "_initial_guess.csv",
                 {"index", "x", "y", "theta", "v", "phi", "a", "omega", "t"});

  // 迭代结果（每一轮优化）
  CSVLogger::Open(run_id_ + "_iterations.csv",
                 {"iteration", "index", "x", "y", "theta", "v", "phi", "a", "omega", "t", "infeasibility"});

  // 最终结果
  CSVLogger::Open(run_id_ + "_final_result.csv",
                 {"index", "x", "y", "theta", "v", "phi", "a", "omega", "t", "success"});

  // 走廊约束
  CSVLogger::Open(run_id_ + "_corridor_constraints.csv",
                 {"index", "x_min", "x_max", "y_min", "y_max", "is_front_disc"});

#if ENABLE_INITIAL_GUESS_INFEAS_CALC
  // Initial guess infeasibility
  CSVLogger::Open(run_id_ + "_initial_guess_infeasibility.csv",
                 {"infeasibility"});
#endif

#if ENABLE_DETAILED_OPT_LOGGING
  // Detailed iteration information (cost, constraints, bounds)
  CSVLogger::Open(run_id_ + "_iteration_details.csv",
                 {"iteration", "cost_value", "num_constraints",
                  "constraint_violations", "constraint_bounds"});
#endif

  current_iteration_ = 0;
  initialized_ = true;
  std::cout << "NLPOptimizationLogger initialized with run_id: " << run_id_ << std::endl;
#endif
}

void NLPOptimizationLogger::LogInitialGuess(double tf,
                                            const std::vector<double>& x,
                                            const std::vector<double>& y,
                                            const std::vector<double>& theta,
                                            const std::vector<double>& v,
                                            const std::vector<double>& phi,
                                            const std::vector<double>& a,
                                            const std::vector<double>& omega) {
#if ENABLE_CSV_LOGGING
  if (!initialized_) {
    Initialize();
  }

  size_t n = x.size();
  for (size_t i = 0; i < n; ++i) {
    double t = tf * static_cast<double>(i) / static_cast<double>(n - 1);
    std::vector<double> values = {
      static_cast<double>(i),
      x[i], y[i], theta[i],
      i < v.size() ? v[i] : 0.0,
      i < phi.size() ? phi[i] : 0.0,
      i < a.size() ? a[i] : 0.0,
      i < omega.size() ? omega[i] : 0.0,
      t
    };
    CSVLogger::WriteLine(run_id_ + "_initial_guess.csv", values);
  }

  std::cout << "Logged initial guess with " << n << " points, tf = " << tf << std::endl;
#endif
}

void NLPOptimizationLogger::LogIterationResult(int iteration,
                                               double infeasibility,
                                               double tf,
                                               const std::vector<double>& x,
                                               const std::vector<double>& y,
                                               const std::vector<double>& theta,
                                               const std::vector<double>& v,
                                               const std::vector<double>& phi,
                                               const std::vector<double>& a,
                                               const std::vector<double>& omega) {
#if ENABLE_CSV_LOGGING
  if (!initialized_) {
    Initialize();
  }

  current_iteration_ = iteration;
  size_t n = x.size();

  for (size_t i = 0; i < n; ++i) {
    double t = tf * static_cast<double>(i) / static_cast<double>(n - 1);
    std::vector<double> values = {
      static_cast<double>(iteration),
      static_cast<double>(i),
      x[i], y[i], theta[i],
      i < v.size() ? v[i] : 0.0,
      i < phi.size() ? phi[i] : 0.0,
      i < a.size() ? a[i] : 0.0,
      i < omega.size() ? omega[i] : 0.0,
      t,
      infeasibility
    };
    CSVLogger::WriteLine(run_id_ + "_iterations.csv", values);
  }

  std::cout << "Logged iteration " << iteration << " with infeasibility = "
            << infeasibility << std::endl;
#endif
}

void NLPOptimizationLogger::LogFinalResult(double tf,
                                          const std::vector<double>& x,
                                          const std::vector<double>& y,
                                          const std::vector<double>& theta,
                                          const std::vector<double>& v,
                                          const std::vector<double>& phi,
                                          const std::vector<double>& a,
                                          const std::vector<double>& omega,
                                          bool success) {
#if ENABLE_CSV_LOGGING
  if (!initialized_) {
    Initialize();
  }

  size_t n = x.size();
  for (size_t i = 0; i < n; ++i) {
    double t = tf * static_cast<double>(i) / static_cast<double>(n - 1);
    std::vector<double> values = {
      static_cast<double>(i),
      x[i], y[i], theta[i],
      i < v.size() ? v[i] : 0.0,
      i < phi.size() ? phi[i] : 0.0,
      i < a.size() ? a[i] : 0.0,
      i < omega.size() ? omega[i] : 0.0,
      t,
      success ? 1.0 : 0.0
    };
    CSVLogger::WriteLine(run_id_ + "_final_result.csv", values);
  }

  std::cout << "Logged final result (success = " << success << ") with "
            << n << " points" << std::endl;
#endif
}

void NLPOptimizationLogger::LogCorridorConstraints(int index,
                                                   double x_min, double x_max,
                                                   double y_min, double y_max,
                                                   bool is_front_disc) {
#if ENABLE_CSV_LOGGING
  if (!initialized_) {
    Initialize();
  }

  std::vector<double> values = {
    static_cast<double>(index),
    x_min, x_max, y_min, y_max,
    is_front_disc ? 1.0 : 0.0
  };

  CSVLogger::WriteLine(run_id_ + "_corridor_constraints.csv", values);
#endif
}

void NLPOptimizationLogger::LogInitialGuessInfeasibility(double infeasibility) {
#if ENABLE_CSV_LOGGING && ENABLE_INITIAL_GUESS_INFEAS_CALC
  if (!initialized_) {
    Initialize();
  }

  std::vector<double> values = {infeasibility};
  CSVLogger::WriteLine(run_id_ + "_initial_guess_infeasibility.csv", values);

  std::cout << "[Initial Guess] Infeasibility = " << infeasibility << std::endl;
#endif
}

void NLPOptimizationLogger::LogIterationDetails(int iteration,
                                                double cost_value,
                                                const std::vector<double>& constraint_violations,
                                                const std::vector<double>& constraint_bounds) {
#if ENABLE_CSV_LOGGING && ENABLE_DETAILED_OPT_LOGGING
  if (!initialized_) {
    Initialize();
  }

  // Convert vectors to comma-separated strings for CSV storage
  std::ostringstream violations_ss, bounds_ss;

  for (size_t i = 0; i < constraint_violations.size(); ++i) {
    violations_ss << constraint_violations[i];
    if (i < constraint_violations.size() - 1) violations_ss << ";";
  }

  for (size_t i = 0; i < constraint_bounds.size(); ++i) {
    bounds_ss << constraint_bounds[i];
    if (i < constraint_bounds.size() - 1) bounds_ss << ";";
  }

  std::vector<std::string> values = {
    std::to_string(iteration),
    std::to_string(cost_value),
    std::to_string(constraint_violations.size()),
    violations_ss.str(),
    bounds_ss.str()
  };

  CSVLogger::WriteLine(run_id_ + "_iteration_details.csv", values);

  std::cout << "[Iteration " << iteration << "] Cost = " << cost_value
            << ", Num constraints = " << constraint_violations.size() << std::endl;
#endif
}

void NLPOptimizationLogger::Finalize() {
#if ENABLE_CSV_LOGGING
  if (initialized_) {
    CSVLogger::CloseAll();
    initialized_ = false;
    std::cout << "NLPOptimizationLogger finalized." << std::endl;
  }
#endif
}

} // namespace util
} // namespace common
