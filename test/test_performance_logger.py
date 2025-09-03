import csv
import os

import pytest
import rclpy
from nav_msgs.msg import Odometry

from diffbot_sim.performance_logger import PerformanceLogger


def test_performance_logger_writes_csv(tmp_path, monkeypatch):
    # Redirect HOME to temporary path so logger writes within tmp directory
    monkeypatch.setenv('HOME', str(tmp_path))

    rclpy.init()
    try:
        node = PerformanceLogger()
        msg = Odometry()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = 200
        msg.twist.twist.linear.x = 0.5

        node.odom_callback(msg)
        node.on_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    csv_path = tmp_path / 'robot_performance_log.csv'
    assert csv_path.exists()

    with csv_path.open() as f:
        reader = csv.reader(f)
        header = next(reader)
        row = next(reader)

    assert header == ['ros_time', 'linear_velocity_x']
    assert float(row[0]) == pytest.approx(1.0000002)
    assert float(row[1]) == pytest.approx(0.5)
