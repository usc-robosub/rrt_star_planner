#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/MarkerArray.h>
#include <barracuda_msgs/Waypoints.h>

// Custom service
#include "rrt_star_planner/PlanPath.h"

#include <random>
#include <vector>
#include <memory>

class RRTNode {
public:
    geometry_msgs::Point position;
    std::shared_ptr<RRTNode> parent;
    double cost;
    std::vector<std::shared_ptr<RRTNode>> children;
    
    RRTNode(const geometry_msgs::Point& pos, std::shared_ptr<RRTNode> p = nullptr, double c = 0.0);
};

class RRTStarPlanner {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // Subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber pointcloud_sub_;
    
    // Service
    ros::ServiceServer plan_service_;
    
    // Publishers for visualization
    ros::Publisher path_pub_;
    ros::Publisher tree_pub_;
    ros::Publisher marker_pub_;
    
    // Current robot pose
    geometry_msgs::PoseStamped current_pose_;
    bool pose_received_;
    
    // Point cloud for obstacles
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    bool pointcloud_received_;
    
    // RRT* parameters
    double max_step_size_;
    double goal_tolerance_;
    double robot_radius_;
    int max_iterations_;
    double rewire_radius_;
    double sampling_bounds_[3][2]; // min and max for x, y, z
    
    // Random number generation
    std::mt19937 gen_;
    std::uniform_real_distribution<double> x_dist_;
    std::uniform_real_distribution<double> y_dist_;
    std::uniform_real_distribution<double> z_dist_;
    std::uniform_real_distribution<double> goal_bias_dist_;
    double goal_bias_probability_;
    
    // Private methods
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool planPathService(rrt_star_planner::PlanPath::Request& req,
                        rrt_star_planner::PlanPath::Response& res);
    
    std::vector<geometry_msgs::Point> planPath(const geometry_msgs::Point& start,
                                               const geometry_msgs::Point& goal);
    
    geometry_msgs::Point samplePoint(const geometry_msgs::Point& goal);
    
    std::shared_ptr<RRTNode> findNearestNode(const std::vector<std::shared_ptr<RRTNode>>& nodes,
                                             const geometry_msgs::Point& point);
    
    std::vector<std::shared_ptr<RRTNode>> findNearbyNodes(const std::vector<std::shared_ptr<RRTNode>>& nodes,
                                                          const geometry_msgs::Point& point,
                                                          double radius);
    
    geometry_msgs::Point steer(const geometry_msgs::Point& from,
                               const geometry_msgs::Point& to);
    
    bool isCollisionFree(const geometry_msgs::Point& from,
                        const geometry_msgs::Point& to);
    
    void updateDescendantCosts(std::shared_ptr<RRTNode> node);
    
    double distance(const geometry_msgs::Point& p1,
                   const geometry_msgs::Point& p2);
    
    std::vector<geometry_msgs::Point> smoothPath(const std::vector<geometry_msgs::Point>& path);
    
    void visualizeTree(const std::vector<std::shared_ptr<RRTNode>>& nodes,
                      std::shared_ptr<RRTNode> goal_node);

public:
    RRTStarPlanner();
    ~RRTStarPlanner() = default;
};

#endif // RRT_STAR_PLANNER_H