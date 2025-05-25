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
    
    // Publishers
    path_pub_ = nh_.advertise<barracuda_msgs::Waypoints>("planned_path", 1);
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

bool RRTStarPlanner::planPathService(rrt_star_planner::PlanPath::Request& req,
                                    rrt_star_planner::PlanPath::Response& res) {
    
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
... (302 lines left)