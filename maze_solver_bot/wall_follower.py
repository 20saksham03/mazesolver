import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import math
import time

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.regions = {}
        self.last_position = None
        self.last_moved_time = self.get_clock().now().to_msg().sec
        self.imu_orientation = None

    def laser_callback(self, msg):
        self.regions = {
            'right': min(min(msg.ranges[0:60]), 3.0),
            'front': min(min(msg.ranges[300:360] + msg.ranges[0:60]), 3.0),
            'left': min(min(msg.ranges[300:360]), 3.0)
        }

    def odom_callback(self, msg):
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_position:
            moved_distance = math.dist(current_position, self.last_position)
            if moved_distance > 0.05:
                self.last_moved_time = self.get_clock().now().to_msg().sec
        self.last_position = current_position

    def imu_callback(self, msg):
        self.imu_orientation = msg.orientation  # Not used yet, but can be expanded for drift correction

    def control_loop(self):
        now_sec = self.get_clock().now().to_msg().sec
        if now_sec - self.last_moved_time > 25:
            self.get_logger().warn("Bot might be stuck! Attempt recovery...")
            self.recovery_behavior()
            self.last_moved_time = now_sec
            return

        if not self.regions:
            return

        msg = Twist()
        if self.regions['front'] < 0.4:
            msg.angular.z = 0.5
            msg.linear.x = 0.0
        elif self.regions['left'] < 0.5:
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.3

        self.cmd_pub.publish(msg)

    def recovery_behavior(self):
        msg = Twist()
        msg.linear.x = -0.1
        msg.angular.z = 0.5
        for _ in range(10):
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
