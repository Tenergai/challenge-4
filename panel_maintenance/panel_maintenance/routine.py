import random
import time
import rclpy

from enum import Enum

from rclpy.node import Node

from rclpy.node import Publisher
from rclpy.node import Subscription

# messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


import sys
sys.path.append("./src/panel_maintenance/panel_maintenance/")

from utils.navigate import Navigator
from utils.state_handling import RobotStateLevel, RobotContext, RoutineInfo
from utils.state_handling import handle_state, get_waypoint, WAYPOINTS

class Routine(Node):
    def __init__(self):
        super().__init__("routine_node")

        # Publishers and Subscriptions
        self.vel_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/burger/odom",
            self.odom_callback,
            10
        )
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            10
        )

        # Initialization
        self.navigation = Navigator(
            self.vel_publisher
        )
        
        # no waypoint to begin with
        # in waiting state
        self.waypoints = [[0.0, 0.0]]
        self.current_waypoint = self.waypoints[0]
        # context by default begins in waiting
        self.robot_context = RobotContext()

        # Create timers to check collisions or alignement

    def get_routine_info(self) -> RoutineInfo:
        return RoutineInfo(
            self.current_waypoint,
            self.waypoints,
            self.navigation,
            self.robot_context
        )
    
    def set_routine_info(self, routine_node: RoutineInfo):
        self.current_waypoint = routine_node.current_waypoint
        self.waypoints = routine_node.waypoints
        self.navigation = routine_node.navigation
        self.robot_context = routine_node.robot_context

    def odom_callback(self, msg: Odometry):
        print("Odom callback cycle")
        time.sleep(1/10)
        
        # verify state
        routine_node = handle_state(
            self.get_routine_info(),
            msg
        )
        self.set_routine_info(routine_node)

    def laser_callback(self, msg: LaserScan):
        pass

    def stop(self):
        self.navigation.set_vel(0.0, 0.0)
        self.navigation.set_angvel(0.0)

def main(args=None):
    rclpy.init(args=args)

    # this will go to state handling
    # when a GOING_TO state is triggered
    # waypoint = get_waypoint(WAYPOINTS)
    # print("Run waypoint")

    routine = Routine()

    try:
        rclpy.spin(routine)
    except KeyboardInterrupt:
        routine.stop()
    
    routine.destroy_node()
    rclpy.shutdown()

if __name__ == "__name__":
    main()