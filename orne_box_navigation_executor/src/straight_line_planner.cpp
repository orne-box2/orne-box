#include <pluginlib/class_list_macros.h>
#include "straight_line_planner/straight_line_planner.h"
#include <tf/tf.h>

// プラグインとして登録
PLUGINLIB_EXPORT_CLASS(straight_line_planner::StraightLinePlanner, nav_core::BaseGlobalPlanner)

namespace straight_line_planner {

StraightLinePlanner::StraightLinePlanner() : initialized_(false) {}

StraightLinePlanner::StraightLinePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void StraightLinePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    ROS_INFO("StraightLinePlanner initialized successfully.");
  }
}

bool StraightLinePlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_ERROR("Planner has not been initialized.");
    return false;
  }

  plan.clear();
  plan.push_back(start);

  double dx = goal.pose.position.x - start.pose.position.x;
  double dy = goal.pose.position.y - start.pose.position.y;
  double dist = std::hypot(dx, dy);
  
  double step_size = 0.1; // 10cm刻みでパスを生成
  int steps = std::floor(dist / step_size);

  for (int i = 1; i <= steps; ++i) {
    double ratio = (double)i / (steps + 1);
    geometry_msgs::PoseStamped new_pose = start;
    new_pose.pose.position.x = start.pose.position.x + ratio * dx;
    new_pose.pose.position.y = start.pose.position.y + ratio * dy;
    
    // 向き（クォータニオン）の計算（簡易的に目標地点を向くように設定）
    double yaw = std::atan2(dy, dx);
    new_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    
    plan.push_back(new_pose);
  }

  plan.push_back(goal);
  ROS_INFO("Plan created with %zu points", plan.size());
  return true;
}

};
