#ifndef STRAIGHT_LINE_PLANNER_H_
#define STRAIGHT_LINE_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace straight_line_planner {
  class StraightLinePlanner : public nav_core::BaseGlobalPlanner {
    public:
      StraightLinePlanner();
      StraightLinePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
    
    private:
      bool initialized_;
  };
};
#endif
