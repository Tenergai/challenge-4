from rclpy.node import Publisher
from rclpy.node import Subscription

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Navigator():
    def __init__(
        self, 
        vel_publisher: Publisher,
        odom_subscriber: Subscription,
        laser_subscriber: Subscription) -> None:
         
        self.vel_publisher = vel_publisher
        self.odom_subscriber = odom_subscriber
        self.laser_subscriber = laser_subscriber


    def set_vel(self, vx: float, vy: float):
        print("Setted Twist to: ", vx, vy)
        vel_cmd = Twist()
        vel_cmd.linear.x = vx
        vel_cmd.linear.y = vy
        
        self.vel_publisher.publish(vel_cmd)

        return vel_cmd
    
    def check_alignment(self, msg: Odometry, waypoint):
        pass
    
    def check_collision(self, msg: LaserScan):
        pass