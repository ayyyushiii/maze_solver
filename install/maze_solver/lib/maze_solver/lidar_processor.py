#!/usr/bin/env python3

import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarProcessor(Node):

    def __init__(self):
        super().__init__('lidar_processor')

        self.pub = self.create_publisher(Float32MultiArray, '/obstacle_data', 10)

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info("Lidar Processor Ready")

    def get_min(self, ranges, center_deg, width=20):

        vals = []

        for d in range(center_deg - width//2, center_deg + width//2 + 1):
            i = d % 360
            val = ranges[i]

            if math.isfinite(val):
                vals.append(val)

        if len(vals) == 0:
            return 10.0

        return min(vals)

    def scan_callback(self, msg):

        ranges = list(msg.ranges)

        front = self.get_min(ranges, 0)
        right = self.get_min(ranges, 270)
        left  = self.get_min(ranges, 90)
        back  = self.get_min(ranges, 180)

        out = Float32MultiArray()
        out.data = [front, right, left, back]

        self.pub.publish(out)


def main(args=None):

    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
