#ifndef NAV2_RRT_STAR_PLANNER__RRT_STAR_PLANNER_HPP_
#define NAV2_RRT_STAR_PLANNER__RRT_STAR_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <random>
#include <functional> // <--- Added for std::function

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_rrt_star_planner
{

struct Node {
    double x, y;
    double cost;
    int parent_index;
};

class RRTStarPlanner : public nav2_core::GlobalPlanner
{
public:
    RRTStarPlanner() = default;
    ~RRTStarPlanner() = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    // --- FIX IS HERE: Added cancel_checker argument ---
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancel_checker) override; 

private:
    Node steer(const Node& from_node, const Node& to_node, double extend_length);
    bool checkCollision(const Node& n1, const Node& n2);
    double distance(const Node& n1, const Node& n2);
    int getNearestNodeId(const std::vector<Node>& nodes, const Node& point);
    std::vector<int> getNearbyNodeIds(const std::vector<Node>& nodes, const Node& point, double radius);

    nav2_costmap_2d::Costmap2D * costmap_;
    std::string global_frame_, name_;
    double interpolation_resolution_;
    
    int max_iterations_;
    double step_size_;
    double goal_bias_;
    double search_radius_;
    double goal_tolerance_;
};

}  // namespace nav2_rrt_star_planner

#endif