#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.occupancy_pub = self.create_publisher(Vector3, '/occupancy_state', 10)
        self.depth_image = None

    def depth_cb(self, msg):
        # Convertir imagen de profundidad a matriz NumPy
        self.depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        occupancy = Vector3()
        # Dividir imagen en 3 regiones (izq, centro, der)
        height, width = self.depth_image.shape
        left_region = self.depth_image[:, :width//3]
        center_region = self.depth_image[:, width//3:2*width//3]
        right_region = self.depth_image[:, 2*width//3:]
        # Detectar obstáculos (<= 0.5 metros)
        occupancy.x = 1 if np.any(left_region <= 0.5) else 0
        occupancy.y = 1 if np.any(center_region <= 0.5) else 0
        occupancy.z = 1 if np.any(right_region <= 0.5) else 0
        self.occupancy_pub.publish(occupancy)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()