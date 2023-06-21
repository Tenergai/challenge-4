from rclpy.node import Publisher

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Navigator():
    def __init__(
        self, 
        vel_publisher: Publisher
        ) -> None:
         
        self.vel_publisher = vel_publisher

    def set_vel(self, vx: float, vy: float):
        print("Velocity was set to: ", vx, vy)
        vel_cmd = Twist()
        vel_cmd.linear.x = vx
        vel_cmd.linear.y = vy
        
        self.vel_publisher.publish(vel_cmd)

        return vel_cmd
    
    def set_angvel(self, wz):
        print("Angular Velocity was set to: ", wz)
        angvel_cmd = Twist()
        angvel_cmd.angular.z = wz

        self.vel_publisher.publish(angvel_cmd)

        return angvel_cmd
    
    def check_alignment(self, msg: Odometry, waypoint):
        pass
    
    def check_collision(self, msg: LaserScan):
        pass