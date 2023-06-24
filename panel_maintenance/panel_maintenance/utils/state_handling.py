import random
import time
import math
import random
from paho.mqtt import client as mqtt_client
from utils import sensorSubscriber

from dataclasses import dataclass
from enum import Enum

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import sys
sys.path.append("./src/panel_maintenance/panel_maintenance/utils/")
from navigate import Navigator

# first point is always the entrance to the section
PANEL_DIM: float = [1.7, 1.0]
ROW1: float = 2.0 - PANEL_DIM[0]/2
ROW2: float = 3.5 - PANEL_DIM[0]/2
WAYPOINTS: dict = {
    "section1_1": [
        [ROW1, 0.0],
        [ROW1, -1.25], 
        [ROW1, -3.0]
    ],
    "section1_2": [
        [ROW1, 0.0],
        [ROW1, 1.25],
        [ROW1, 3.0]
    ],
    "section2_1": [
        [ROW2, 0.0],
        [ROW2, -1.25],
        [ROW2, -3.0]
    ],
    "section2_2": [
        [ROW2, 0.0],
        [ROW2, 1.25],
        [ROW2, 3.0]
    ]
}

STD_VEL: float = 0.1
STD_ANGVEL: float = 0.1

# angle presicion
DA: float = 0.05
# tollerance for x and y coordinate
DX = 0.05
DY = 0.05

def get_waypoint(waypoints: dict):
    section = random.choice(list(waypoints.keys()))

    return waypoints[section]

def to_euler_angles(q: Quaternion):
    roll = math.atan2(
        2 * (q.w * q.x + q.y * q.z),
        1 - 2 * (q.x * q.x + q.y * q.y)
    )
    pitch = 2 * math.atan2(
        math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z)),
        math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    ) - math.pi / 2
    yaw = math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )

    return roll, pitch, yaw

class RobotStateLevel(Enum):
    # in the waiting state it is assumed that the robot is at the origin
    WAITING = 0
    STATIC = 1
    # in this state it is assumed that it just moves foward (fill a coordinate)
    GOING_TO = 2
    # to return to origin
    RETURN = 3
    # in this state it is assumed that it aligns with a waypoint or an intermediate waypoint
    ALIGN = 4
    WORKING = 5

class RobotContext:
    _state: RobotStateLevel = RobotStateLevel.WAITING

    def __init__(self) -> None:
        pass

    def transition_to(
        self, 
        state: RobotStateLevel,
        nav: Navigator,
        position: Point=Point(),
        orientation: Quaternion=Quaternion(),
        waypoint: list=[],
        ):

        print(f"Robot state changed from {self._state} to {state}")

        # define one time behaviour when transition happens
        if self._state == RobotStateLevel.WAITING and state == RobotStateLevel.GOING_TO or (
            self._state == RobotStateLevel.ALIGN and state == RobotStateLevel.GOING_TO
        ):
            nav.set_vel(STD_VEL, 0.0)

        elif (self._state == RobotStateLevel.GOING_TO and state == RobotStateLevel.ALIGN) or (
            self._state == RobotStateLevel.WORKING and state == RobotStateLevel.ALIGN
        ):
            nav.set_vel(0.0, 0.0)
            if position.y < waypoint[1]:
                # left of the waypoint
                # rotate clock-wise
                nav.set_angvel(STD_ANGVEL)
            else:
                # rotate counter-clock-wise
                nav.set_angvel(-STD_ANGVEL)

        elif self._state == RobotStateLevel.GOING_TO and state == RobotStateLevel.WORKING:
            nav.set_vel(0.0, 0.0)
            _, _, yaw = to_euler_angles(orientation)
            print("Orientation before working: ", yaw)
            if yaw < 0:
                # rotate clockwise
                nav.set_angvel(STD_ANGVEL)
            else:
                # rotate counter-clock-wise
                nav.set_angvel(-STD_ANGVEL)

        self._state = state

# datatype with relevant routine node information
@dataclass
class RoutineInfo():
    current_waypoint: list
    waypoints: list
    navigation: Navigator
    robot_context: RobotContext


def handle_state(
        routine_node: RoutineInfo,
        odom: Odometry
    ) -> RoutineInfo:

    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation

    if routine_node.robot_context._state == RobotStateLevel.WAITING:
        routine_node = handle_waiting(routine_node, orientation)

    elif routine_node.robot_context._state == RobotStateLevel.STATIC:
        routine_node = handle_static(routine_node)

    elif routine_node.robot_context._state == RobotStateLevel.GOING_TO:
        routine_node = handle_going_to(routine_node, position, orientation)
    
    elif routine_node.robot_context._state == RobotStateLevel.RETURN:
        routine_node = handle_return(routine_node, position)
    
    elif routine_node.robot_context._state == RobotStateLevel.ALIGN:
        routine_node = handle_align(routine_node, position, orientation)
    
    elif routine_node.robot_context._state == RobotStateLevel.WORKING:
        routine_node = handle_working(routine_node, position, orientation)
    
    else:
        print("Invalid State Input!")

    return routine_node

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    global last_reading
    last_reading = str(msg.payload.decode("utf-8"))
    
def handle_waiting(routine_node: RoutineInfo, orientation: Quaternion) -> RoutineInfo:
    _, _, yaw = to_euler_angles(orientation)
    # angle presicion
    if abs(yaw) > DA:
        print("Orienting towards solar farm.")
    else:
        client = mqtt_client.Client()
        client.on_connect = sensorSubscriber.on_connect
        client.on_message = on_message
        client.connect('broker.emqx.io', 1883)
        client.subscribe("anomaly/drone")
        last_reading = None
        routine_node.navigation.set_angvel(0.0)
        routine_node.navigation.set_vel(0.0, 0.0)
        print("Waiting for Anomaly...")
        while routine_node.robot_context._state == RobotStateLevel.WAITING:
            print(".", end="")
            client.loop()
            print("last_reading",last_reading)
            time.sleep(10)
            if last_reading != None:
                waypoints = get_waypoint(WAYPOINTS)
                
                section = None
                for waypoint in waypoints:
                    if last_reading == waypoint:
                        section = waypoint
                        break
                print("section",section)
                time.sleep(10)
                if section != None:
                    print("")
                    print("Anomaly reported at a section with waypoints: ", waypoints)
                    routine_node.waypoints = waypoints
                    routine_node.current_waypoint = last_reading
                    routine_node.robot_context.transition_to(
                        RobotStateLevel.GOING_TO,
                        routine_node.navigation
                    )
                else:
                    print("No section identified")

                last_reading = None

    return routine_node


#####
# functions that will be called repeatidly when in its respective state

def handle_static(routine_node: RoutineInfo) -> RoutineInfo:
    print("Handling STATIC Mode")

    return routine_node

def handle_going_to(
        routine_node: RoutineInfo,
        position: Point,
        orientation: Quaternion
    ) -> RoutineInfo:

    print("Handling GOING_TO Mode")
    print("Currently going to: ", routine_node.current_waypoint)
    print("Total Waypoints to cover: ", routine_node.waypoints)

    print("Relative Position: ",
        round(abs(position.x - routine_node.current_waypoint[0]), 4),
        round(abs(position.y - routine_node.current_waypoint[1]), 4)
    )
    if (abs(position.x - routine_node.current_waypoint[0]) < DX) and (
        abs(position.y - routine_node.current_waypoint[1]) < DY
    ):
        print("Robot Reached Turning Point.")
        if routine_node.waypoints[0] == routine_node.current_waypoint:
            # reached intermediate waypoint
            routine_node.current_waypoint = routine_node.waypoints[1]
            routine_node.robot_context.transition_to(
                RobotStateLevel.ALIGN,
                routine_node.navigation,
                position=position,
                waypoint=routine_node.current_waypoint
            )

        # add an elif for the origin point which ends up being the return
        elif abs(position.x) < DX and abs(position.y) < DY:
            # it is the origin
            print("Reached origin")
            routine_node.navigation.set_angvel(STD_ANGVEL)
            routine_node.robot_context.transition_to(
                RobotStateLevel.WAITING,
                routine_node.navigation
            )


        else:
            # point that he should be working on
            routine_node.robot_context.transition_to(
            RobotStateLevel.WORKING,
            routine_node.navigation,
            orientation=orientation
        )
        
    else:
        if routine_node.waypoints[0] == routine_node.current_waypoint:
            print("Robot is on its way to the entrance of the section.")
        elif routine_node.current_waypoint == [0.0, 0.0]:
            print("Going to origin.")
        else:
            print("Robot is on its way to work.")

    return routine_node

def handle_return(
        routine_node: RoutineInfo,
        position: Point
    ) -> RoutineInfo:

    print("Handling RETURN Mode")

    return routine_node

def handle_align(
    routine_node: RoutineInfo,
    position: Point,
    orientation: Quaternion
    ) -> RoutineInfo:

    print("Handling ALIGN Mode")
    print("Aligning to waypoint ", routine_node.current_waypoint)

    _, _, yaw = to_euler_angles(orientation)
    unit_vector = [math.cos(yaw), math.sin(yaw)]
    to_waipoint_vec = [
        routine_node.current_waypoint[0] - position.x,
        routine_node.current_waypoint[1] - position.y
    ]

    ang = math.acos(
        (unit_vector[0] * to_waipoint_vec[0] + unit_vector[1] * to_waipoint_vec[1]) /
        (math.sqrt(unit_vector[0] * unit_vector[0] + unit_vector[1] * unit_vector[1]) * 
         math.sqrt(to_waipoint_vec[0] * to_waipoint_vec[0] + to_waipoint_vec[1] * to_waipoint_vec[1]))
    )

    print("Angle between robot orientation and waypoint: ", round(ang, 4))
    if abs(ang) < DA:
        print("Robot is Aligned!")
        routine_node.navigation.set_angvel(0.0)
        routine_node.robot_context.transition_to(
            RobotStateLevel.GOING_TO,
            routine_node.navigation
        )

    return routine_node

def handle_working(
        routine_node: RoutineInfo,
        position: Point,
        orientation: Quaternion
    ) -> RoutineInfo:

    print("Handling WORKING Mode")

    # update current waypoint if not the last one
    _, _, yaw = to_euler_angles(orientation)
    print("Angle of orientation towards solar panel: ", yaw)
    if abs(yaw) < DA:
        # stop robot and publish to LED
        routine_node.navigation.set_angvel(0.0)
        print("ROBOT IS WORKING FOR 5 SECONDS!")
        time.sleep(5)
        if routine_node.waypoints.index(routine_node.current_waypoint) == len(routine_node.waypoints) - 1:
            # reached the last waypoint
            print("Last waypoint. Robot is going to return to origin.")
            # first intermediate waypoint
            routine_node.waypoints = [routine_node.waypoints[0]]
            routine_node.waypoints.append([0.0, 0.0])
            routine_node.current_waypoint = routine_node.waypoints[0]
            routine_node.robot_context.transition_to(
                RobotStateLevel.ALIGN,
                routine_node.navigation,
                position=position,
                waypoint=routine_node.current_waypoint
            )
        else:
            # align to next waypoint
            print("Proceding to next solar panel.")
            routine_node.current_waypoint = routine_node.waypoints[routine_node.waypoints.index(routine_node.current_waypoint) + 1]
            print("Changed waypoint to the next one: ", routine_node.current_waypoint)
            routine_node.robot_context.transition_to(
                RobotStateLevel.ALIGN,
                routine_node.navigation,
                position=position,
                waypoint=routine_node.current_waypoint
            )

    return routine_node