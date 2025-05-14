#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # Cambia a tu tópico de LIDAR
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(
            LaserScan,
            '/filtered_scan',  # Tópico para las lecturas filtradas
            10
        )

    def lidar_callback(self, msg):
        filtered_ranges = []
        angle_increment = msg.angle_increment
        for i, range_value in enumerate(msg.ranges):
            angle = msg.angle_min + i * angle_increment
            if not self.is_angle_filtered(angle):
                filtered_ranges.append(range_value)
            else:
                filtered_ranges.append(float('inf'))  # O un valor alto para ignorar

        msg.ranges = filtered_ranges
        self.publisher.publish(msg)

    def is_angle_filtered(self, angle):
        # Filtrar ángulos deseados en radianes
        if (0.523 <= angle <= 1.047) or (2.094 <= angle <= 2.618) or (-2.618 <= angle <= -2.094) or (-1.047 <= angle <= 0):
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
