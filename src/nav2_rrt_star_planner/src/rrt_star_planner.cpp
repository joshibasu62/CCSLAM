#include "nav2_rrt_star_planner/rrt_star_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_rrt_star_planner
{

void RRTStarPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/, // Commented out parameter name to fix unused warning
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    name_ = name;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    auto node = parent.lock();
    
    node->declare_parameter(name + ".max_iterations", 5000);
    node->declare_parameter(name + ".step_size", 0.5);
    node->declare_parameter(name + ".goal_bias", 0.1);
    node->declare_parameter(name + ".search_radius", 1.0);
    node->declare_parameter(name + ".goal_tolerance", 0.2);

    node->get_parameter(name + ".max_iterations", max_iterations_);
    node->get_parameter(name + ".step_size", step_size_);
    node->get_parameter(name + ".goal_bias", goal_bias_);
    node->get_parameter(name + ".search_radius", search_radius_);
    node->get_parameter(name + ".goal_tolerance", goal_tolerance_);

    interpolation_resolution_ = costmap_->getResolution();
}

void RRTStarPlanner::cleanup() {}
void RRTStarPlanner::activate() {}
void RRTStarPlanner::deactivate() {}

// --- FIX IS HERE: Updated Signature ---
nav_msgs::msg::Path RRTStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = global_frame_;

    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
        RCLCPP_ERROR(rclcpp::get_logger("RRTStarPlanner"), "Planner only accepts goals in the Global Frame");
        return path;
    }

    std::vector<Node> nodes;
    nodes.reserve(max_iterations_);
    Node start_node = {start.pose.position.x, start.pose.position.y, 0.0, -1};
    nodes.push_back(start_node);

    Node goal_node_raw = {goal.pose.position.x, goal.pose.position.y, 0.0, -1};

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_rand(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
    std::uniform_real_distribution<> y_rand(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
    std::uniform_real_distribution<> bias_rand(0.0, 1.0);

    int best_goal_node_idx = -1;
    double min_cost_to_goal = std::numeric_limits<double>::max();

    for (int i = 0; i < max_iterations_; ++i) {
        
        // --- FIX: Check if user cancelled the goal ---
        if (cancel_checker()) {
            RCLCPP_INFO(rclcpp::get_logger("RRTStarPlanner"), "Goal was canceled. Stopping planning.");
            return path;
        }

        Node rand_node;
        if (bias_rand(gen) < goal_bias_) {
            rand_node = goal_node_raw;
        } else {
            rand_node = {x_rand(gen), y_rand(gen), 0.0, -1};
        }

        int nearest_idx = getNearestNodeId(nodes, rand_node);
        Node new_node = steer(nodes[nearest_idx], rand_node, step_size_);

        if (!checkCollision(nodes[nearest_idx], new_node)) {
            continue;
        }

        std::vector<int> nearby_indices = getNearbyNodeIds(nodes, new_node, search_radius_);
        int best_parent_idx = nearest_idx;
        double min_cost = nodes[nearest_idx].cost + distance(nodes[nearest_idx], new_node);

        for (int idx : nearby_indices) {
            double cost = nodes[idx].cost + distance(nodes[idx], new_node);
            if (cost < min_cost && checkCollision(nodes[idx], new_node)) {
                min_cost = cost;
                best_parent_idx = idx;
            }
        }

        new_node.cost = min_cost;
        new_node.parent_index = best_parent_idx;
        nodes.push_back(new_node);
        int new_node_idx = nodes.size() - 1;

        for (int idx : nearby_indices) {
            double new_cost = new_node.cost + distance(new_node, nodes[idx]);
            if (new_cost < nodes[idx].cost && checkCollision(new_node, nodes[idx])) {
                nodes[idx].cost = new_cost;
                nodes[idx].parent_index = new_node_idx;
            }
        }

        if (distance(new_node, goal_node_raw) <= goal_tolerance_) {
            if (new_node.cost < min_cost_to_goal) {
                min_cost_to_goal = new_node.cost;
                best_goal_node_idx = new_node_idx;
            }
        }
    }

    if (best_goal_node_idx != -1) {
        Node current = nodes[best_goal_node_idx];
        while (current.parent_index != -1) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = current.x;
            pose.pose.position.y = current.y;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
            current = nodes[current.parent_index];
        }
        path.poses.push_back(start);
        std::reverse(path.poses.begin(), path.poses.end());
        path.poses.push_back(goal);
    }

    return path;
}

Node RRTStarPlanner::steer(const Node& from_node, const Node& to_node, double extend_length) {
    Node new_node;
    double dist = distance(from_node, to_node);
    if (dist < extend_length) {
        new_node = to_node;
    } else {
        double theta = atan2(to_node.y - from_node.y, to_node.x - from_node.x);
        new_node.x = from_node.x + extend_length * cos(theta);
        new_node.y = from_node.y + extend_length * sin(theta);
    }
    return new_node;
}

bool RRTStarPlanner::checkCollision(const Node& n1, const Node& n2) {
    unsigned int mx, my;
    double dist = distance(n1, n2);
    int steps = std::max(1, (int)(dist / interpolation_resolution_));
    double dx = (n2.x - n1.x) / steps;
    double dy = (n2.y - n1.y) / steps;

    double x = n1.x;
    double y = n1.y;

    for (int i = 0; i <= steps; ++i) {
        if (!costmap_->worldToMap(x, y, mx, my)) return false; 
        unsigned char cost = costmap_->getCost(mx, my);
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) return false;
        x += dx;
        y += dy;
    }
    return true;
}

double RRTStarPlanner::distance(const Node& n1, const Node& n2) {
    return std::hypot(n1.x - n2.x, n1.y - n2.y);
}

int RRTStarPlanner::getNearestNodeId(const std::vector<Node>& nodes, const Node& point) {
    int min_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < nodes.size(); ++i) {
        double dist = distance(nodes[i], point);
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }
    return min_idx;
}

std::vector<int> RRTStarPlanner::getNearbyNodeIds(const std::vector<Node>& nodes, const Node& point, double radius) {
    std::vector<int> indices;
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (distance(nodes[i], point) <= radius) {
            indices.push_back(i);
        }
    }
    return indices;
}

}  // namespace nav2_rrt_star_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_star_planner::RRTStarPlanner, nav2_core::GlobalPlanner)