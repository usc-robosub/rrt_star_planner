#include "rrt_star_planner/rrt_star_planner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_star_planner");
    
    RRTStarPlanner planner;
    
    ros::spin();
    
    return 0;
}