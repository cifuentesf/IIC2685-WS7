#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from pathlib import Path
from math import pi
dir_pkg = Path(__file__).resolve().parent.parent   # /lab1


class PoseLoader(Node):
    def __init__(self):
        super().__init__("pose_loader")
        self.pub = self.create_publisher(PoseArray, "goal_list", 10)
    
    def load_pose_array(self):
        pose = Pose()
        pose_array = PoseArray()
        file = open(f"{dir_pkg}/data/pose_list.txt", 'r')
        for line in file:
            try:
                pose_data = line[1:-2].split(',')
                pose.position.x = float(pose_data[0])
                pose.position.y = float(pose_data[1])
                pose.orientation.z = float(pose_data[2]) * pi/180
                pose_array.poses.append(pose)
            except ValueError:
                break
        file.close()
        self.pub.publish(pose_array)




rclpy.init()
nodo = PoseLoader()
nodo.load_pose_array()
rclpy.spin(nodo)
rclpy.shutdown()