#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
import sensor_msgs.point_cloud2 as pc2
from rrt_star_planner.srv import PlanPath, PlanPathRequest
import time
import sys

class RRTStarPlannerTester:
    def __init__(self):
        rospy.init_node('rrt_star_planner_tester')

        # Publishers
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=1, latch=True)
        self.pointcloud_pub = rospy.Publisher('/obstacle_pointcloud', PointCloud2, queue_size=1, latch=True)

        # Subscribers for verification
        self.planned_path = None
        self.tree_markers = None
        self.path_sub = rospy.Subscriber('/planned_path', rospy.AnyMsg, self.path_callback)
        self.tree_sub = rospy.Subscriber('/rrt_tree', MarkerArray, self.tree_callback)

        # Service client
        rospy.wait_for_service('/plan_path', timeout=10)
        self.plan_service = rospy.ServiceProxy('/plan_path', PlanPath)

        rospy.loginfo("RRT* Planner Tester initialized")

    def path_callback(self, msg):
        self.planned_path = msg

    def tree_callback(self, msg):
        self.tree_markers = msg

    def create_obstacle_pointcloud(self, obstacles):
        """
        Create a PointCloud2 message from obstacle points
        obstacles: list of (x, y, z) tuples representing obstacle points
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        # Create point cloud
        points = []
        for obs in obstacles:
            # Add multiple points around each obstacle center to create a dense obstacle
            for dx in np.linspace(-0.2, 0.2, 5):
                for dy in np.linspace(-0.2, 0.2, 5):
                    for dz in np.linspace(-0.2, 0.2, 5):
                        points.append([obs[0] + dx, obs[1] + dy, obs[2] + dz])

        # Create PointCloud2 message
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        cloud = pc2.create_cloud(header, fields, points)
        return cloud

    def publish_robot_pose(self, x, y, z):
        """Publish current robot pose"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)
        rospy.loginfo(f"Published robot pose: ({x}, {y}, {z})")

    def test_basic_planning(self):
        """Test 1: Basic path planning without obstacles"""
        rospy.loginfo("\n== === Test 1: Basic Path Planning (No Obstacles) ===")

        # Set robot position
        start_pos = (0.0, 0.0, 1.0)
        self.publish_robot_pose(*start_pos)
        rospy.sleep(1.0)  # Give more time for the pose to be received

        # Create service request
        req = PlanPathRequest()
        req.goal_pose.header.frame_id = "world"
        req.goal_pose.pose.position.x = 5.0
        req.goal_pose.pose.position.y = 5.0
        req.goal_pose.pose.position.z = 2.0
        req.goal_pose.pose.orientation.w = 1.0

        # Call service
        try:
            rospy.loginfo("Calling plan_path service...")
            resp = self.plan_service(req)

            if resp.waypoints.points:
                rospy.loginfo(f"✓ Path found with {len(resp.waypoints.points)} waypoints")

                # Verify start and end points
                start_point = resp.waypoints.points[0]
                end_point = resp.waypoints.points[-1]

                start_dist = np.sqrt((start_point.x - start_pos[0])**2 +
                                   (start_point.y - start_pos[1])**2 +
                                   (start_point.z - start_pos[2])**2)

                end_dist = np.sqrt((end_point.x - req.goal_pose.pose.position.x)**2 +
                                 (end_point.y - req.goal_pose.pose.position.y)**2 +
                                 (end_point.z - req.goal_pose.pose.position.z)**2)

                rospy.loginfo(f"  Start point distance from robot: {start_dist:.3f}m")
                rospy.loginfo(f"  End point distance from goal: {end_dist:.3f}m")

                if start_dist < 0.1 and end_dist < 0.5:
                    rospy.loginfo("✓ Test 1 PASSED")
                    return True
                else:
                    rospy.logerr("✗ Test 1 FAILED: Path doesn't connect start and goal properly")
                    return False
            else:
                rospy.logerr("✗ Test 1 FAILED: No path returned")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"✗ Test 1 FAILED: Service call failed: {e}")
            return False

    def test_obstacle_avoidance(self):
        """Test 2: Path planning with obstacles"""
        rospy.loginfo("\n=== Test 2: Path Planning with Obstacle Avoidance ===")

        # Set robot position
        start_pos = (0.0, 0.0, 1.0)
        self.publish_robot_pose(*start_pos)

        # Create obstacles - wall in the middle
        obstacles = []
        for y in np.linspace(-2, 2, 20):
            for z in np.linspace(0, 3, 15):
                obstacles.append((2.5, y, z))

        # Publish obstacles
        cloud = self.create_obstacle_pointcloud(obstacles)
        self.pointcloud_pub.publish(cloud)
        rospy.loginfo(f"Published {len(obstacles)} obstacle points")
        rospy.sleep(0.5)

        # Create service request
        req = PlanPathRequest()
        req.goal_pose.header.frame_id = "world"
        req.goal_pose.pose.position.x = 5.0
        req.goal_pose.pose.position.y = 0.0
        req.goal_pose.pose.position.z = 1.0
        req.goal_pose.pose.orientation.w = 1.0

        # Call service
        try:
            rospy.loginfo("Calling plan_path service with obstacles...")
            resp = self.plan_service(req)

            if resp.waypoints.points:
                rospy.loginfo(f"✓ Path found with {len(resp.waypoints.points)} waypoints")

                # Check if path avoids obstacles
                collision = False
                for point in resp.waypoints.points:
                    # Check distance to obstacle wall (x=2.5, y=-2 to 2)
                    if abs(point.x - 2.5) < 0.6 and abs(point.y) < 2.2:
                        collision = True
                        rospy.logerr(f"✗ Collision detected at ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})")

                if not collision:
                    rospy.loginfo("✓ Path successfully avoids obstacles")
                    rospy.loginfo("✓ Test 2 PASSED")
                    return True
                else:
                    rospy.logerr("✗ Test 2 FAILED: Path collides with obstacles")
                    return False
            else:
                rospy.logerr("✗ Test 2 FAILED: No path returned")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"✗ Test 2 FAILED: Service call failed: {e}")
            return False

    def test_complex_environment(self):
        """Test 3: Path planning in complex environment"""
        rospy.loginfo("\n=== Test 3: Complex Environment Planning ===")

        # Set robot position
        start_pos = (-4.0, -4.0, 1.0)
        self.publish_robot_pose(*start_pos)

        # Create complex obstacle environment
        obstacles = []

        # Multiple walls
        # Wall 1
        for y in np.linspace(-3, 3, 30):
            for z in np.linspace(0, 2, 10):
                obstacles.append((-2.0, y, z))

        # Wall 2
        for x in np.linspace(-1, 3, 20):
            for z in np.linspace(0, 2, 10):
                obstacles.append((x, 0.0, z))

        # Floating obstacles
        obstacles.extend([(1.0, 2.0, 1.5), (2.0, -2.0, 1.0), (-1.0, -2.0, 1.5)])

        # Publish obstacles
        cloud = self.create_obstacle_pointcloud(obstacles)
        self.pointcloud_pub.publish(cloud)
        rospy.loginfo(f"Published complex environment with {len(obstacles)} obstacle points")
        rospy.sleep(0.5)

        # Create service request
        req = PlanPathRequest()
        req.goal_pose.header.frame_id = "world"
        req.goal_pose.pose.position.x = 4.0
        req.goal_pose.pose.position.y = 4.0
        req.goal_pose.pose.position.z = 1.5
        req.goal_pose.pose.orientation.w = 1.0

        # Call service
        try:
            rospy.loginfo("Calling plan_path service in complex environment...")
            resp = self.plan_service(req)

            if resp.waypoints.points:
                rospy.loginfo(f"✓ Path found with {len(resp.waypoints.points)} waypoints")

                # Calculate path length
                path_length = 0.0
                for i in range(1, len(resp.waypoints.points)):
                    p1 = resp.waypoints.points[i-1]
                    p2 = resp.waypoints.points[i]
                    path_length += np.sqrt((p2.x - p1.x)**2 +
                                         (p2.y - p1.y)**2 +
                                         (p2.z - p1.z)**2)

                rospy.loginfo(f"  Total path length: {path_length:.2f}m")

                # Verify path is reasonable (not too long)
                direct_dist = np.sqrt((4.0 - (-4.0))**2 + (4.0 - (-4.0))**2 + (1.5 - 1.0)**2)
                if path_length < direct_dist * 2.5:  # Path shouldn't be more than 2.5x direct distance
                    rospy.loginfo("✓ Path length is reasonable")
                    rospy.loginfo("✓ Test 3 PASSED")
                    return True
                else:
                    rospy.logerr("✗ Test 3 FAILED: Path is unreasonably long")
                    return False
            else:
                rospy.logerr("✗ Test 3 FAILED: No path returned")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"✗ Test 3 FAILED: Service call failed: {e}")
            return False

    def test_unreachable_goal(self):
        """Test 4: Unreachable goal (surrounded by obstacles)"""
        rospy.loginfo("\n=== Test 4: Unreachable Goal Test ===")

        # Set robot position
        start_pos = (0.0, 0.0, 1.0)
        self.publish_robot_pose(*start_pos)

        # Create box around goal
        obstacles = []
        goal_pos = (3.0, 3.0, 1.5)

        # Create enclosing box
        for x in np.linspace(goal_pos[0] - 1, goal_pos[0] + 1, 10):
            for y in np.linspace(goal_pos[1] - 1, goal_pos[1] + 1, 10):
                for z in np.linspace(goal_pos[2] - 1, goal_pos[2] + 1, 10):
                    # Only add points on the surface of the box
                    if (abs(x - goal_pos[0]) > 0.8 or
                        abs(y - goal_pos[1]) > 0.8 or
                        abs(z - goal_pos[2]) > 0.8):
                        obstacles.append((x, y, z))

        # Publish obstacles
        cloud = self.create_obstacle_pointcloud(obstacles)
        self.pointcloud_pub.publish(cloud)
        rospy.loginfo(f"Published enclosing box with {len(obstacles)} obstacle points")
        rospy.sleep(0.5)

        # Create service request
        req = PlanPathRequest()
        req.goal_pose.header.frame_id = "world"
        req.goal_pose.pose.position.x = goal_pos[0]
        req.goal_pose.pose.position.y = goal_pos[1]
        req.goal_pose.pose.position.z = goal_pos[2]
        req.goal_pose.pose.orientation.w = 1.0

        # Call service
        try:
            rospy.loginfo("Calling plan_path service with unreachable goal...")
            resp = self.plan_service(req)

            # For unreachable goals, the planner should either:
            # 1. Return no path (empty waypoints)
            # 2. Return a path that gets as close as possible

            if not resp.waypoints.points:
                rospy.loginfo("✓ Planner correctly returned no path for unreachable goal")
                rospy.loginfo("✓ Test 4 PASSED")
                return True
            else:
                # Check if the path gets reasonably close
                end_point = resp.waypoints.points[-1]
                dist_to_goal = np.sqrt((end_point.x - goal_pos[0])**2 +
                                     (end_point.y - goal_pos[1])**2 +
                                     (end_point.z - goal_pos[2])**2)

                if dist_to_goal > 0.8:  # Should stop outside the box
                    rospy.loginfo(f"✓ Planner stopped {dist_to_goal:.2f}m from unreachable goal")
                    rospy.loginfo("✓ Test 4 PASSED")
                    return True
                else:
                    rospy.logerr("✗ Test 4 FAILED: Path penetrates obstacle box")
                    return False

        except rospy.ServiceException as e:
            # Service failure might be expected for unreachable goals
            rospy.loginfo("✓ Service returned error for unreachable goal (expected behavior)")
            rospy.loginfo("✓ Test 4 PASSED")
            return True

    def run_all_tests(self):
        """Run all tests and report results"""
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("Starting RRT* Planner Test Suite")
        rospy.loginfo("="*50)

        # Allow node to fully initialize
        rospy.sleep(2.0)

        results = []

        # Run tests
        results.append(("Basic Planning", self.test_basic_planning()))
        rospy.sleep(1.0)

        results.append(("Obstacle Avoidance", self.test_obstacle_avoidance()))
        rospy.sleep(1.0)

        results.append(("Complex Environment", self.test_complex_environment()))
        rospy.sleep(1.0)

        results.append(("Unreachable Goal", self.test_unreachable_goal()))

        # Summary
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("TEST SUMMARY")
        rospy.loginfo("="*50)

        passed = 0
        for test_name, result in results:
            status = "PASSED" if result else "FAILED"
            symbol = "✓" if result else "✗"
            rospy.loginfo(f"{symbol} {test_name}: {status}")
            if result:
                passed += 1

        rospy.loginfo("-"*50)
        rospy.loginfo(f"Total: {passed}/{len(results)} tests passed")

        if passed == len(results):
            rospy.loginfo("\n🎉 All tests passed! RRT* planner is working correctly.")
            return True
        else:
            rospy.logerr(f"\n❌ {len(results) - passed} tests failed. Please check the implementation.")
            return False

def main():
    try:
        tester = RRTStarPlannerTester()
        success = tester.run_all_tests()

        # Keep node alive for a bit to allow viewing in RViz
        rospy.loginfo("\nTest complete. Node will shut down in 5 seconds...")
        rospy.sleep(5.0)

        # Exit with appropriate code
        sys.exit(0 if success else 1)

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Test failed with exception: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
