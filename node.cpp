//
// Parking planner node with JSON environment loading
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

#include "common/math/pose.h"
#include "common/math/vec2d.h"
#include "common/math/polygon2d.h"
#include "common/util/time.h"
#include "visualization_plot.h"
#include "interface.h"
#include "my_env.h"

using namespace common::math;

class ParkingPlannerNode {
public:
  ParkingPlannerNode(): nh_("~") {
    // Initialize visualization with world coordinate frame
    VisualizationPlot::Init(nh_, "world", "/parking_planner_markers");

    // Load environment file path
    env_file_ = nh_.param<std::string>("env_file", "");
    ROS_INFO("Environment file parameter: '%s'", env_file_.c_str());

    // Load environment from JSON if file is specified
    if(env_.Read(env_file_)) {
      boundary_received_ = true;

      // test run
      planner_.env()->points.SetPoints(env_.points);
      planner_.env()->UpdateBounds();
      planner_.env()->obstacles = env_.obstacles;
      env_.Visualize();
      env_.SaveCSV();
      ros::spinOnce();
    }

    // Subscribe to RViz interactive topics
    pose_sub_ = nh_.subscribe("/initialpose", 1, &ParkingPlannerNode::GetStartPoseCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &ParkingPlannerNode::GetGoalPoseCallback, this);
    clicked_sub_ = nh_.subscribe("/clicked_point", 1, &ParkingPlannerNode::GetClickedPointCallback, this);

    ROS_INFO("========================================");
    ROS_INFO("Parking Planner Node Initialized!");
    ROS_INFO("========================================");
    if (!env_file_.empty()) {
      ROS_INFO("Environment file: %s", env_file_.c_str());
      if (boundary_received_) {
        ROS_INFO("Environment loaded successfully");
        ROS_INFO("  - Boundary points: %lu", env_.points.size());
        ROS_INFO("  - Obstacles: %lu", env_.obstacles.size());
      } else {
        ROS_INFO("No environment file found - will create new");
      }
    } else {
      ROS_INFO("No environment file specified - creating from scratch");
    }
    ROS_INFO("========================================");
    ROS_INFO("HOW TO USE:");
    ROS_INFO("========================================");
    ROS_INFO("1. DEFINE ENVIRONMENT (if not loaded):");
    ROS_INFO("   Use 'Publish Point' tool in RViz");
    ROS_INFO("   First polygon = boundary");
    ROS_INFO("   Subsequent polygons = obstacles");
    ROS_INFO("   Click near first point to close each polygon");
    ROS_INFO("   GREEN lines show connections, YELLOW dots show points");
    ROS_INFO("");
    ROS_INFO("2. SET START POSE:");
    ROS_INFO("   Use '2D Pose Estimate' tool (BLUE box)");
    ROS_INFO("");
    ROS_INFO("3. SET GOAL POSE:");
    ROS_INFO("   Use '2D Nav Goal' tool (CYAN box)");
    ROS_INFO("   Planning starts automatically!");
    ROS_INFO("");
    ROS_INFO("4. AUTO-SAVE:");
    ROS_INFO("   Environment is saved automatically after each change");
    ROS_INFO("   File: %s", env_file_.empty() ? "(not specified)" : env_file_.c_str());
    ROS_INFO("========================================");
  }

  void GetStartPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = tf::getYaw(msg->pose.pose.orientation);

    planner_.set_start_pose(x, y, theta);
    planner_.set_current_state(x, y, theta, 0.0);
    start_received_ = true;

    ROS_INFO("Start pose set: (%.2f, %.2f, %.2f rad)", x, y, theta);
    ROS_INFO("Now please set the GOAL pose using '2D Nav Goal' tool");

    // Visualize start box
    auto start_box = planner_.env()->vehicle.GenerateBox(Pose(x, y, theta));
    VisualizationPlot::PlotPolygon(Polygon2d(start_box), 0.2, Color::Blue, 1000, "Start");
    VisualizationPlot::Trigger();
  }

  void GetGoalPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    // Check if start pose has been set first
    if (!start_received_) {
      ROS_WARN("========================================");
      ROS_WARN("Please set START pose first!");
      ROS_WARN("Use '2D Pose Estimate' tool in RViz to set the start position");
      ROS_WARN("========================================");
      return;
    }

    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double theta = tf::getYaw(msg->pose.orientation);

    planner_.set_goal_pose(x, y, theta);
    goal_received_ = true;

    ROS_INFO("Goal pose set: (%.2f, %.2f, %.2f rad)", x, y, theta);

    // Visualize goal box
    auto goal_box = planner_.env()->vehicle.GenerateBox(Pose(x, y, theta));
    VisualizationPlot::PlotPolygon(Polygon2d(goal_box), 0.2, Color::Magenta, 1001, "Goal");
    VisualizationPlot::Trigger();

    // Start planning (we know start is already set)
    PlanAndVisualize();
  }

  void GetClickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg) {
    Vec2d pt(msg->point.x, msg->point.y);

    // Check if clicked point is close to any existing point (to close polygon)
    bool is_closed = std::any_of(clicked_polygon_.begin(), clicked_polygon_.end(), [&](Vec2d &x) {
      return x.DistanceTo(pt) < 1.0;
    });

    if (!is_closed) {
      // Add point to current polygon
      clicked_polygon_.push_back(pt);

      // Visualize temporary polygon with BOTH lines AND points for better visibility
      std::vector<double> xs, ys;
      for (auto& i : clicked_polygon_) {
        xs.push_back(i.x());
        ys.push_back(i.y());
      }

      // Clear old visualization first
      VisualizationPlot::PlotPoints({}, {}, Color::Yellow, 0.3, 10000, "TempPoints");
      VisualizationPlot::Plot({}, {}, 0.15, Color::Yellow, 10001, "TempLines");

      // Draw the lines connecting the points (GREEN for better visibility)
      if (xs.size() > 1) {
        VisualizationPlot::Plot(xs, ys, 0.15, Color::Green, 10001, "TempLines");
      }

      // Draw the points themselves (YELLOW for better visibility)
      VisualizationPlot::PlotPoints(xs, ys, Color::Yellow, 0.3, 10000, "TempPoints");

      VisualizationPlot::Trigger();

      ROS_INFO("Point added: (%.2f, %.2f). Total points: %lu. Click near first point to close polygon.",
               pt.x(), pt.y(), clicked_polygon_.size());
    } else {
      // Close the polygon
      if (boundary_received_) {
        // Subsequent polygons become obstacles
        ROS_INFO("Closing obstacle polygon with %lu points", clicked_polygon_.size());

        planner_.env()->obstacles.emplace_back(clicked_polygon_);
        env_.obstacles = planner_.env()->obstacles;

        // Auto-save environment to file
        if (!env_file_.empty()) {
          env_.Save(env_file_);
          ROS_INFO("✓ Environment auto-saved to: %s", env_file_.c_str());
        }

        ROS_INFO("Obstacle added. Total obstacles: %lu", env_.obstacles.size());
      } else {
        // First polygon becomes the boundary
        ROS_INFO("Closing boundary polygon with %lu points", clicked_polygon_.size());
        clicked_polygon_.push_back(clicked_polygon_.front());  // Close the loop

        // Clear existing boundary points and add new ones
        planner_.env()->points.Clear();
        planner_.env()->points.AddPoints(clicked_polygon_);
        boundary_received_ = true;

        // Update MyEnvironment with processed points from PointCloud
        env_.points = planner_.env()->points.points();
        planner_.env()->UpdateBounds();

        // Auto-save environment to file
        if (!env_file_.empty()) {
          env_.Save(env_file_);
          ROS_INFO("✓ Environment auto-saved to: %s", env_file_.c_str());
        }

        ROS_INFO("Boundary set. Now you can add obstacles by clicking points, or set start/goal poses.");
      }

      // Clear temporary polygon visualization and show final environment
      clicked_polygon_.clear();
      // Clear both temporary markers
      VisualizationPlot::PlotPoints({}, {}, Color::Yellow, 0.3, 10000, "TempPoints");
      VisualizationPlot::Plot({}, {}, 0.15, Color::Yellow, 10001, "TempLines");
      env_.Visualize();
    }
  }

  void PlanAndVisualize() {
    ROS_INFO("========================================");
    ROS_INFO("Starting trajectory planning...");
    ROS_INFO("========================================");

    Trajectory result;
    int parking_type = 1; // goal parking

    double start_time = common::util::GetCurrentTimestamp();

    if (planner_.Plan(&result, parking_type)) {
      double elapsed = common::util::GetCurrentTimestamp() - start_time;

      ROS_INFO("========================================");
      ROS_INFO("Planning SUCCEEDED!");
      ROS_INFO("  - Planning time: %.3f seconds", elapsed);
      ROS_INFO("  - Trajectory points: %lu", result.size());
      ROS_INFO("  - Trajectory duration: %.2f seconds", result.empty() ? 0 : result.back().t);
      ROS_INFO("========================================");

      if (!result.empty()) {
        // Extract trajectory data for visualization
        std::vector<double> x, y, theta;
        for (const auto& pt : result) {
          x.push_back(pt.x);
          y.push_back(pt.y);
          theta.push_back(pt.theta);
        }

        VisualizationPlot::Plot(x, y, 0.2, Color::Green, 1, "Final Trajectory");

        // Visualize start and goal boxes again to ensure they're visible
        auto start_box = planner_.env()->vehicle.GenerateBox(planner_.start_pose());
        auto goal_box = planner_.env()->vehicle.GenerateBox(planner_.goal_pose());

        VisualizationPlot::PlotPolygon(Polygon2d(start_box), 0.15, Color::Blue, 1000, "Start");
        VisualizationPlot::PlotPolygon(Polygon2d(goal_box), 0.15, Color::Magenta, 1001, "Goal");

        // Visualize obstacles
        env_.Visualize();

        VisualizationPlot::Trigger();

        ROS_INFO("Final trajectory visualized in RViz");
      }

    } else {
      ROS_ERROR("========================================");
      ROS_ERROR("Planning FAILED!");
      ROS_ERROR("========================================");
    }

    // Reset flags for next planning
    start_received_ = false;
    goal_received_ = false;
  }

private:
  // Helper function to convert obstacles to geometry_msgs format
  std::vector<std::vector<geometry_msgs::Point>> ConvertObstaclesToGeometryMsgs(
      const std::vector<Polygon2d>& obstacles) {
    std::vector<std::vector<geometry_msgs::Point>> result;

    for (const auto& obs : obstacles) {
      std::vector<geometry_msgs::Point> points;
      for (const auto& pt : obs.points()) {
        geometry_msgs::Point p;
        p.x = pt.x();
        p.y = pt.y();
        p.z = 0.0;
        points.push_back(p);
      }
      result.push_back(points);
    }

    return result;
  }

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_, goal_sub_, clicked_sub_;
  ParkingPlanner planner_;
  MyEnvironment env_;
  std::string env_file_;

  bool start_received_ = false;
  bool goal_received_ = false;
  bool boundary_received_ = false;
  std::vector<Vec2d> clicked_polygon_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "parking_planner_node");

  ParkingPlannerNode node;

  ROS_INFO("Parking planner node is running...");
  ROS_INFO("Waiting for start and goal poses from RViz");

  ros::spin();

  return 0;
}
