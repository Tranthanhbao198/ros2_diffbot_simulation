import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.pub = self.create_publisher(Twist, '/diffbot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update_velocity)
        self.start_time = self.get_clock().now()
        self.max_speed = 5.0
        self.accel_time = 5.0  # time to reach max speed
        self.decel_time = 1.0  # time to stop
        self.active = True

    def update_velocity(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        msg = Twist()
        if self.active:
            if elapsed <= self.accel_time:
                msg.linear.x = self.max_speed * (elapsed / self.accel_time)
            else:
                self.get_logger().info('Reached max speed. Stopping in 1s...')
                time.sleep(self.decel_time)
                msg.linear.x = 0.0
                self.active = False
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

