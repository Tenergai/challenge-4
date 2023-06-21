import random
import time

from dataclasses import dataclass
from enum import Enum

import sys
sys.path.append("./src/panel_maintenance/panel_maintenance/utils/")
from navigate import Navigator


WAYPOINTS: dict = {
    "section1_1": [
         [2.0, -1.25], 
         [2.0, -3.0]
    ],
    "section1_2": [
         [2.0, 1.25],
         [2.0, 3.0]
    ],
    "section2_1": [
         [3.5, -1.25],
         [3.5, -3.0]
    ],
    "section2_2": [
         [3.5, 1.25],
         [3.5, 3.0]
    ]
}

STD_VEL: float = 0.1

class RobotStateLevel(Enum):
    # in the waiting state it is assumed that the robot is at the origin
    WAITING = 0
    STATIC = 1
    # in this state it is assumed that it just moves foward (fill a coordinate)
    GOING_TO = 2
    # in this state it is assumed that it aligns with a waypoint or an intermediate waypoint
    ALIGN = 3
    WORKING = 4

class RobotContext:
    _state: RobotStateLevel = RobotStateLevel.WAITING

    def __init__(self) -> None:
        pass

    def transition_to(
        self, 
        state: RobotStateLevel,
        nav: Navigator
        ):

        print(f"Robot state changed from {self._state} to {state}")
        if self._state == RobotStateLevel.WAITING and state == RobotStateLevel.GOING_TO:
            nav.set_vel(STD_VEL, 0.0)

        self._state = state

# datatype with relevant routine node information
@dataclass
class RoutineInfo():
    current_waypoint: list
    waypoints: list
    navigation: Navigator
    robot_context: RobotContext

def get_waypoint(waypoints: dict):
    section = random.choice(list(waypoints.keys()))

    return waypoints[section]

def handle_state(
        routine_node: RoutineInfo
    ) -> RoutineInfo:

    if routine_node.robot_context._state == RobotStateLevel.WAITING:
        routine_node = handle_waiting(routine_node)

    elif routine_node.robot_context._state == RobotStateLevel.STATIC:
        routine_node = handle_static(routine_node)

    elif routine_node.robot_context._state == RobotStateLevel.GOING_TO:
        routine_node = handle_going_to(routine_node)
    
    elif routine_node.robot_context._state == RobotStateLevel.ALIGN:
        routine_node = handle_align(routine_node)
    
    else:
        print("Invalid State Input!")

    return routine_node
        

def handle_waiting(routine_node: RoutineInfo) -> RoutineInfo:
    print("Waiting for Anomaly...")
    while routine_node.robot_context._state == RobotStateLevel.WAITING:
        print(".", end="")
        time.sleep(1)
        flip_coin = random.random()
        if flip_coin < 0.2:
            waypoints = get_waypoint(WAYPOINTS)

            print("")
            print("Anomaly reported at a section with waypoints: ", waypoints)
            routine_node.waypoints = waypoints
            routine_node.current_waypoint = routine_node.waypoints[0]
            routine_node.robot_context.transition_to(
                RobotStateLevel.GOING_TO,
                routine_node.navigation
            )

    return routine_node

def handle_static(routine_node: RoutineInfo) -> RoutineInfo:
    print("Handling STATIC Mode")

    return routine_node

def handle_going_to(
        routine_node: RoutineInfo
    ) -> RoutineInfo:

    print("Handling GOING_TO Mode")
    print("Currently going to: ", routine_node.current_waypoint)
    print("Total Waypoints to cover: ", routine_node.waypoints)

    return routine_node

def handle_align(
    routine_node: RoutineInfo
    ) -> RoutineInfo:

    print("Handling ALIGN Mode")

    return routine_node