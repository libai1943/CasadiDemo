#ifndef PARKING_PLANNER_CSV_LOGGER_H
#define PARKING_PLANNER_CSV_LOGGER_H

#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <map>

// ============================================================
// Macro Definition Switches: Set to 1 to enable, 0 to disable
// ============================================================
#define ENABLE_CSV_LOGGING 0

// Control whether to calculate and print initial guess infeasibility
// This may add computation overhead, so it can be disabled if not needed
#define ENABLE_INITIAL_GUESS_INFEAS_CALC 1

// Control whether to log detailed optimization information (cost, constraints, bounds)
// This generates more detailed data for debugging but creates larger log files
#define ENABLE_DETAILED_OPT_LOGGING 1

namespace common {
namespace util {

/**
 * @brief CSV日志记录器 - 用于调试和可视化
 *
 * 使用ENABLE_CSV_LOGGING宏控制是否启用记录功能
 * 当设置为0时，所有记录函数都不会执行，不会产生性能开销
 */
class CSVLogger {
public:
  /**
   * @brief 打开CSV文件准备写入
   * @param filename 文件名（相对于输出目录）
   * @param header CSV表头（列名）
   * @return 是否成功打开
   */
  static bool Open(const std::string& filename, const std::vector<std::string>& header);

  /**
   * @brief 写入一行数据
   * @param values 数据值（必须与表头列数匹配）
   */
  static void WriteLine(const std::string& filename, const std::vector<double>& values);

  /**
   * @brief 写入一行数据（支持混合类型）
   * @param values 数据值（字符串形式）
   */
  static void WriteLine(const std::string& filename, const std::vector<std::string>& values);

  /**
   * @brief 关闭CSV文件
   */
  static void Close(const std::string& filename);

  /**
   * @brief 关闭所有打开的CSV文件
   */
  static void CloseAll();

  /**
   * @brief 设置输出目录
   * @param dir 目录路径（如果不存在会自动创建）
   */
  static void SetOutputDirectory(const std::string& dir);

  /**
   * @brief 获取完整的文件路径
   */
  static std::string GetFullPath(const std::string& filename);

  /**
   * @brief 创建唯一的带时间戳的文件名
   * @param prefix 文件名前缀
   * @param suffix 文件名后缀（默认.csv）
   */
  static std::string CreateTimestampedFilename(const std::string& prefix,
                                                const std::string& suffix = ".csv");

private:
  static std::map<std::string, std::ofstream> files_;
  static std::string output_directory_;

  static void EnsureDirectoryExists(const std::string& dir);
};

// ============================================================
// 便捷宏定义 - 根据ENABLE_CSV_LOGGING自动启用/禁用
// ============================================================
#if ENABLE_CSV_LOGGING

#define CSV_OPEN(filename, header) CSVLogger::Open(filename, header)
#define CSV_WRITE_LINE(filename, values) CSVLogger::WriteLine(filename, values)
#define CSV_CLOSE(filename) CSVLogger::Close(filename)
#define CSV_CLOSE_ALL() CSVLogger::CloseAll()
#define CSV_SET_OUTPUT_DIR(dir) CSVLogger::SetOutputDirectory(dir)

#else

// 当禁用时，这些宏不做任何事
#define CSV_OPEN(filename, header) (true)
#define CSV_WRITE_LINE(filename, values) ((void)0)
#define CSV_CLOSE(filename) ((void)0)
#define CSV_CLOSE_ALL() ((void)0)
#define CSV_SET_OUTPUT_DIR(dir) ((void)0)

#endif

// ============================================================
// 专用的日志记录器类 - 用于不同的数据类型
// ============================================================

/**
 * @brief 混合A*搜索日志记录器
 */
class HybridAStarLogger {
public:
  /**
   * @brief 初始化所有CSV文件
   */
  static void Initialize(const std::string& run_id = "");

  /**
   * @brief 记录搜索节点信息
   */
  static void LogSearchNode(double x, double y, double phi,
                           double traj_cost, double heuristic_cost,
                           bool direction, double steering,
                           int iteration);

  /**
   * @brief 记录Reed-Shepp路径
   */
  static void LogReedSheppPath(const std::vector<double>& x,
                              const std::vector<double>& y,
                              const std::vector<double>& phi,
                              const std::vector<bool>& gear,
                              double total_length);

  /**
   * @brief 记录最终的粗路径结果
   */
  static void LogCoarsePath(const std::vector<double>& x,
                           const std::vector<double>& y,
                           const std::vector<double>& phi,
                           const std::vector<double>& v,
                           const std::vector<double>& a,
                           const std::vector<double>& steer);

  /**
   * @brief 关闭所有文件
   */
  static void Finalize();

private:
  static std::string run_id_;
  static bool initialized_;
};

/**
 * @brief NLP优化日志记录器
 */
class NLPOptimizationLogger {
public:
  /**
   * @brief 初始化所有CSV文件
   */
  static void Initialize(const std::string& run_id = "");

  /**
   * @brief 记录初始猜测
   */
  static void LogInitialGuess(double tf,
                             const std::vector<double>& x,
                             const std::vector<double>& y,
                             const std::vector<double>& theta,
                             const std::vector<double>& v,
                             const std::vector<double>& phi,
                             const std::vector<double>& a,
                             const std::vector<double>& omega);

  /**
   * @brief 记录优化迭代结果
   */
  static void LogIterationResult(int iteration,
                                 double infeasibility,
                                 double tf,
                                 const std::vector<double>& x,
                                 const std::vector<double>& y,
                                 const std::vector<double>& theta,
                                 const std::vector<double>& v,
                                 const std::vector<double>& phi,
                                 const std::vector<double>& a,
                                 const std::vector<double>& omega);

  /**
   * @brief 记录最终优化结果
   */
  static void LogFinalResult(double tf,
                            const std::vector<double>& x,
                            const std::vector<double>& y,
                            const std::vector<double>& theta,
                            const std::vector<double>& v,
                            const std::vector<double>& phi,
                            const std::vector<double>& a,
                            const std::vector<double>& omega,
                            bool success);

  /**
   * @brief 记录走廊约束
   */
  static void LogCorridorConstraints(int index,
                                    double x_min, double x_max,
                                    double y_min, double y_max,
                                    bool is_front_disc);

  /**
   * @brief Log initial guess infeasibility
   * @param infeasibility Initial guess constraint violation
   */
  static void LogInitialGuessInfeasibility(double infeasibility);

  /**
   * @brief Log detailed iteration information (cost function, constraints, bounds)
   * @param iteration Iteration number
   * @param cost_value Objective function value
   * @param constraint_violations Vector of constraint violation values
   * @param constraint_bounds Vector of constraint upper bounds
   */
  static void LogIterationDetails(int iteration,
                                  double cost_value,
                                  const std::vector<double>& constraint_violations,
                                  const std::vector<double>& constraint_bounds);

  /**
   * @brief 关闭所有文件
   */
  static void Finalize();

private:
  static std::string run_id_;
  static bool initialized_;
  static int current_iteration_;
};

} // namespace util
} // namespace common

#endif // PARKING_PLANNER_CSV_LOGGER_H

