#include "rrt_star_planner/rrt_star_planner.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <limits>

// RRTNode implementation
RRTNode::RRTNode(const geometry_msgs::Point& pos, std::shared_ptr<RRTNode> p, double c)
    : position(pos), parent(p), cost(c) {}

// RRTStarPlanner implementation
RRTStarPlanner::RRTStarPlanner() 
    : nh_(), 
      pnh_("~"), 
      pose_received_(false), 
      pointcloud_received_(false),
      gen_(std::random_device{}()) {
    
    // Load parameters
    pnh_.param("max_step_size", max_step_size_, 0.5);
    pnh_.param("goal_tolerance", goal_tolerance_, 0.3);
    pnh_.param("robot_radius", robot_radius_, 0.5);
    pnh_.param("max_iterations", max_iterations_, 5000);
    pnh_.param("rewire_radius", rewire_radius_, 1.0);
    pnh_.param("goal_bias_probability", goal_bias_probability_, 0.1);
    
    // Sampling bounds
    pnh_.param("x_min", sampling_bounds_[0][0], -10.0);
    pnh_.param("x_max", sampling_bounds_[0][1], 10.0);
    pnh_.param("y_min", sampling_bounds_[1][0], -10.0);
    pnh_.param("y_max", sampling_bounds_[1][1], 10.0);
    pnh_.param("z_min", sampling_bounds_[2][0], 0.0);
    pnh_.param("z_max", sampling_bounds_[2][1], 5.0);
    
    // Initialize distributions
    x_dist_ = std::uniform_real_distribution<double>(sampling_bounds_[0][0], sampling_bounds_[0][1]);
    y_dist_ = std::uniform_real_distribution<double>(sampling_bounds_[1][0], sampling_bounds_[1][1]);
    z_dist_ = std::uniform_real_distribution<double>(sampling_bounds_[2][0], sampling_bounds_[2][1]);
    goal_bias_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);
    
    // Initialize point cloud and kdtree
    obstacle_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    
    // Subscribers
    pose_sub_ = nh_.subscribe("robot_pose", 1, &RRTStarPlanner::poseCallback, this);
    pointcloud_sub_ = nh_.subscribe("obstacle_pointcloud", 1, &RRTStarPlanner::pointcloudCallback, this);
    
    // Service
    plan_service_ = nh_.advertiseService("plan_path", &RRTStarPlanner::planPathService, this);
    // Collision check service client
    pnh_.param<std::string>("check_collision_service", collision_service_name_, std::string("check_collision"));
    collision_client_ = nh_.serviceClient<barracuda_msgs::CheckCollision>(collision_service_name_);
    
    // Publishers
    path_pub_ = nh_.advertise<barracuda_msgs::Waypoints>("rrt_waypoints", 1);
    tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("rrt_tree", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("planning_markers", 1);
    
    ROS_INFO("RRT* 3D Planner initialized");
}

void RRTStarPlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    pose_received_ = true;
}

void RRTStarPlanner::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::fromROSMsg(*msg, *obstacle_cloud_);
    if (!obstacle_cloud_->empty()) {
        kdtree_->setInputCloud(obstacle_cloud_);
        pointcloud_received_ = true;
    }
}

bool RRTStarPlanner::planPathService(barracuda_msgs::PlanPath::Request& req,
                                    barracuda_msgs::PlanPath::Response& res) {
    
    if (!pose_received_) {
        ROS_ERROR("No robot pose received yet!");
        return false;
    }
    
    if (!pointcloud_received_) {
        ROS_WARN("No obstacle point cloud received. Planning without obstacles.");
    }
    
    // Extract start and goal positions
    geometry_msgs::Point start = current_pose_.pose.position;
    geometry_msgs::Point goal = req.goal_pose.pose.position;
    
    ROS_INFO("Planning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
             start.x, start.y, start.z, goal.x, goal.y, goal.z);
    
    // Run RRT* algorithm
    std::vector<geometry_msgs::Point> path = planPath(start, goal);
    
    if (path.empty()) {
        ROS_WARN("No path found!");
        return false;
    }
    
    // Convert path to barracuda_msgs::Waypoints
    barracuda_msgs::Waypoints waypoints_msg;
    waypoints_msg.header.stamp = ros::Time::now();
    waypoints_msg.header.frame_id = current_pose_.header.frame_id;
    waypoints_msg.points = path;
    
    res.waypoints = waypoints_msg;
    path_pub_.publish(waypoints_msg);
    
    ROS_INFO("Path found with %zu waypoints", path.size());
    return true;
}

std::vector<geometry_msgs::Point> RRTStarPlanner::planPath(const geometry_msgs::Point& start,
                                                           const geometry_msgs::Point& goal) {
    // Initialize tree with start node
    auto root = std::make_shared<RRTNode>(start);
    std::vector<std::shared_ptr<RRTNode>> nodes;
    nodes.push_back(root);
    
    std::shared_ptr<RRTNode> goal_node = nullptr;
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Sample random point (with goal bias)
        geometry_msgs::Point random_point = samplePoint(goal);
        
        // Find nearest node in tree
        auto nearest_node = findNearestNode(nodes, random_point);
        
        // Steer towards random point
        geometry_msgs::Point new_point = steer(nearest_node->position, random_point);
        
        // Check collision
        if (!isCollisionFree(nearest_node->position, new_point)) {
            continue;
        }
        
        // Create new node
        double new_cost = nearest_node->cost + distance(nearest_node->position, new_point);
        auto new_node = std::make_shared<RRTNode>(new_point, nearest_node, new_cost);
        
        // Find nearby nodes for rewiring
        std::vector<std::shared_ptr<RRTNode>> nearby_nodes = findNearbyNodes(nodes, new_point, rewire_radius_);
        
        // Choose best parent
        for (auto& near_node : nearby_nodes) {
            double cost = near_node->cost + distance(near_node->position, new_point);
            if (cost < new_cost && isCollisionFree(near_node->position, new_point)) {
                new_node->parent = near_node;
                new_node->cost = cost;
                new_cost = cost;
            }
        }
        
        // Add to parent's children
        new_node->parent->children.push_back(new_node);
        nodes.push_back(new_node);
        
        // Rewire nearby nodes
        for (auto& near_node : nearby_nodes) {
            if (near_node == new_node->parent) continue;
            
            double cost = new_node->cost + distance(new_node->position, near_node->position);
            if (cost < near_node->cost && isCollisionFree(new_node->position, near_node->position)) {
                // Remove from old parent's children
                if (near_node->parent) {
                    auto& children = near_node->parent->children;
                    children.erase(std::remove(children.begin(), children.end(), near_node), children.end());
                }
                
                // Set new parent
                near_node->parent = new_node;
                near_node->cost = cost;
                new_node->children.push_back(near_node);
                
                // Update costs of descendants
                updateDescendantCosts(near_node);
            }
        }
        
        // Check if we reached the goal
        if (distance(new_point, goal) < goal_tolerance_) {
            if (!goal_node || new_node->cost < goal_node->cost) {
                goal_node = new_node;
            }
        }
        
        // Visualize tree periodically
        if (iter % 100 == 0) {
            visualizeTree(nodes, goal_node);
        }
    }
    
    // Extract path
    std::vector<geometry_msgs::Point> path;
    if (goal_node) {
        auto current = goal_node;
        while (current) {
            path.push_back(current->position);
            current = current->parent;
        }
        std::reverse(path.begin(), path.end());
        
        // Smooth path
        path = smoothPath(path);
    }
    
    return path;
}

geometry_msgs::Point RRTStarPlanner::samplePoint(const geometry_msgs::Point& goal) {
    geometry_msgs::Point point;
    
    // Goal biasing
    if (goal_bias_dist_(gen_) < goal_bias_probability_) {
        point = goal;
    } else {
        point.x = x_dist_(gen_);
        point.y = y_dist_(gen_);
        point.z = z_dist_(gen_);
    }
    
    return point;
}

std::shared_ptr<RRTNode> RRTStarPlanner::findNearestNode(const std::vector<std::shared_ptr<RRTNode>>& nodes,
                                                         const geometry_msgs::Point& point) {
    std::shared_ptr<RRTNode> nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    
    for (const auto& node : nodes) {
        double dist = distance(node->position, point);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    
    return nearest;
}

std::vector<std::shared_ptr<RRTNode>> RRTStarPlanner::findNearbyNodes(const std::vector<std::shared_ptr<RRTNode>>& nodes,
                                                                      const geometry_msgs::Point& point,
                                                                      double radius) {
    std::vector<std::shared_ptr<RRTNode>> nearby;
    
    for (const auto& node : nodes) {
        if (distance(node->position, point) < radius) {
            nearby.push_back(node);
        }
    }
    
    return nearby;
}

geometry_msgs::Point RRTStarPlanner::steer(const geometry_msgs::Point& from,
                                           const geometry_msgs::Point& to) {
    double dist = distance(from, to);
    
    if (dist <= max_step_size_) {
        return to;
    }
    
    geometry_msgs::Point new_point;
    double ratio = max_step_size_ / dist;
    new_point.x = from.x + ratio * (to.x - from.x);
    new_point.y = from.y + ratio * (to.y - from.y);
    new_point.z = from.z + ratio * (to.z - from.z);
    
    return new_point;
}

bool RRTStarPlanner::isCollisionFree(const geometry_msgs::Point& from,
                                    const geometry_msgs::Point& to) {
    // Discretize the segment and query collision service
    double dist = distance(from, to);
    int num_checks = std::max(2, static_cast<int>(dist / (robot_radius_ * 0.5)));

    if (!collision_client_.exists()) {
        // Try waiting briefly for service to appear
        collision_client_.waitForExistence(ros::Duration(0.5));
    }

    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;

        geometry_msgs::Point center;
        center.x = from.x + t * (to.x - from.x);
        center.y = from.y + t * (to.y - from.y);
        center.z = from.z + t * (to.z - from.z);

        barracuda_msgs::CheckCollision srv;
        srv.request.center = center;
        srv.request.radius = robot_radius_;

        if (!collision_client_.call(srv)) {
            ROS_WARN_THROTTLE(1.0, "Collision service '%s' call failed; assuming collision.", collision_service_name_.c_str());
            return false;
        }

        if (srv.response.collision) {
            return false;
        }
    }

    return true;
}

void RRTStarPlanner::updateDescendantCosts(std::shared_ptr<RRTNode> node) {
    for (auto& child : node->children) {
        child->cost = node->cost + distance(node->position, child->position);
        updateDescendantCosts(child);
    }
}

double RRTStarPlanner::distance(const geometry_msgs::Point& p1,
                               const geometry_msgs::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<geometry_msgs::Point> RRTStarPlanner::smoothPath(const std::vector<geometry_msgs::Point>& path) {
    if (path.size() <= 2) {
        return path;
    }
    
    std::vector<geometry_msgs::Point> smoothed;
    smoothed.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t j = path.size() - 1;
        
        // Try to connect to furthest visible point
        while (j > i + 1) {
            if (isCollisionFree(path[i], path[j])) {
                smoothed.push_back(path[j]);
                i = j;
                break;
            }
            j--;
        }
        
        if (j == i + 1) {
            smoothed.push_back(path[i + 1]);
            i++;
        }
    }
    
    return smoothed;
}

void RRTStarPlanner::visualizeTree(const std::vector<std::shared_ptr<RRTNode>>& nodes,
                                  std::shared_ptr<RRTNode> goal_node) {
    visualization_msgs::MarkerArray tree_markers;
    
    // Tree edges
    visualization_msgs::Marker edges;
    edges.header.frame_id = current_pose_.header.frame_id;
    edges.header.stamp = ros::Time::now();
    edges.ns = "rrt_edges";
    edges.id = 0;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;
    edges.scale.x = 0.02;
    edges.color.r = 0.0;
    edges.color.g = 0.7;
    edges.color.b = 0.0;
    edges.color.a = 0.7;
    
    for (const auto& node : nodes) {
        if (node->parent) {
            edges.points.push_back(node->parent->position);
            edges.points.push_back(node->position);
        }
    }
    
    tree_markers.markers.push_back(edges);
    
    // Path to goal
    if (goal_node) {
        visualization_msgs::Marker path;
        path.header = edges.header;
        path.ns = "rrt_path";
        path.id = 1;
        path.type = visualization_msgs::Marker::LINE_STRIP;
        path.action = visualization_msgs::Marker::ADD;
        path.scale.x = 0.05;
        path.color.r = 1.0;
        path.color.g = 0.0;
        path.color.b = 0.0;
        path.color.a = 1.0;
        
        auto current = goal_node;
        while (current) {
            path.points.push_back(current->position);
            current = current->parent;
        }
        
        tree_markers.markers.push_back(path);
    }
    
    tree_pub_.publish(tree_markers);
}
