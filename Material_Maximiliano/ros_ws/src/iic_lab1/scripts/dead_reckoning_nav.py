#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import math
import time

class DeadReckoningNav(Node):
    def __init__(self):
        super().__init__('dead_reckoning_nav')

        self.cmd_vel_pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.create_subscription(PoseArray, '/goal_list', self.accion_mover_cb, 10)
        self.create_subscription(Vector3, '/occupancy_state', self.obstacle_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # Variables de control
        self.current_pose = None
        self.obstacle_detected = False
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 1.0  # rad/s
        self.correction_factor = 0.95  # Factor de corrección para giros

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def obstacle_cb(self, msg):
        if msg.x == 1 or msg.y == 1 or msg.z == 1:
            self.obstacle_detected = True
            self.get_logger().warn("Obstáculo detectado! Deteniendo movimiento.")
        else:
            self.obstacle_detected = False

    def aplicar_velocidad(self, speed_commands):
        for v, omega, t in speed_commands:
            if self.obstacle_detected:
                while self.obstacle_detected:
                    time.sleep(0.1) 
            start_time = time.time()
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = omega
            while (time.time() - start_time) < t and not self.obstacle_detected:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            self.cmd_vel_pub.publish(Twist())

    def mover_robot_a_destino(self, goal_pose):
        dx = goal_pose.position.x - self.current_pose.position.x
        dy = goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Movimiento lineal
        t_linear = distance / self.linear_speed
        linear_command = (self.linear_speed, 0.0, t_linear)

        # Giro (ajustar ángulo)
        target_theta = math.atan2(dy, dx)
        d_theta = target_theta - self.current_pose.orientation.z
        t_angular = abs(d_theta) / self.angular_speed * self.correction_factor
        angular_command = (0.0, self.angular_speed * (d_theta/abs(d_theta)), t_angular)

        self.aplicar_velocidad([linear_command, angular_command])

    def accion_mover_cb(self, msg):
        for pose in msg.poses:
            self.mover_robot_a_destino(pose)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()