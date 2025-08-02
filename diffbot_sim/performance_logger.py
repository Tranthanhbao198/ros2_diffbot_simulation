#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
import time

class PerformanceLogger(Node):
    def __init__(self):
        super().__init__('performance_logger')
        
        # Tạo một subscriber cho chủ đề /odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10) # QoS profile depth
            
        # Đường dẫn và tệp CSV để lưu dữ liệu
        home_dir = os.path.expanduser('~')
        self.csv_file_path = os.path.join(home_dir, 'robot_performance_log.csv')
        
        try:
            # Mở tệp CSV trong chế độ ghi
            self.csv_file = open(self.csv_file_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Ghi dòng tiêu đề
            self.csv_writer.writerow(['ros_time', 'linear_velocity_x'])
            
            self.get_logger().info(f"Đã mở tệp log tại: {self.csv_file_path}")
            self.get_logger().info("Bắt đầu ghi dữ liệu... Nhấn Ctrl+C để dừng.")
            
        except IOError as e:
            self.get_logger().error(f"Không thể mở tệp CSV để ghi: {e}")
            self.destroy_node() # Tự hủy node nếu không mở được tệp

    def odom_callback(self, msg):
        """Callback được gọi mỗi khi có tin nhắn Odometry mới."""
        # Lấy thời gian từ header của tin nhắn (chính xác nhất)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Lấy vận tốc thẳng theo trục x
        linear_velocity_x = msg.twist.twist.linear.x
        
        # Ghi một dòng mới vào tệp CSV
        self.csv_writer.writerow([timestamp, linear_velocity_x])

    def on_shutdown(self):
        """Đảm bảo tệp được đóng đúng cách khi node dừng lại."""
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.get_logger().info("Đóng tệp log.")
            self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    logger_node = PerformanceLogger()
    
    try:
        # Giữ cho node chạy cho đến khi bị ngắt (Ctrl+C)
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        logger_node.get_logger().info('Đã nhận tín hiệu dừng (Ctrl+C).')
    finally:
        # Dọn dẹp
        logger_node.on_shutdown()
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
