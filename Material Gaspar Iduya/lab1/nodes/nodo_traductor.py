#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
from kobuki_ros_interfaces.msg import BumperEvent



class NodoTraductor(Node):
    def __init__(self):
        super().__init__("Traductor")
        self.pub = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.twist_msg = Twist()
        self.sub = self.create_subscription(BumperEvent, "/events/bumper", self.revisar_bumpers, 10)
        self.create_timer(0.2, self.enviar_mensaje)
        self.keyboard_listener = keyboard.Listener(self.manage_tecla_apretada, self.manage_tecla_soltada)
        self.keyboard_listener.start()
    
    def manage_tecla_apretada(self, key):
        print()
        try:
            if key.char == 'i':
                self.twist_msg.linear.x = 0.2
            elif key.char == 'j':
                self.twist_msg.linear.x = -0.2
            elif key.char == 'a':
                self.twist_msg.angular.z = 1.0
            elif key.char == 's':
                self.twist_msg.angular.z = -1.0
        except AttributeError:
            pass

    def manage_tecla_soltada(self, key):
        try:
            if key.char == 'i':
                self.twist_msg.linear.x = 0.0
            elif key.char == 'j':
                self.twist_msg.linear.x = 0.0
            elif key.char == 'a':
                self.twist_msg.angular.z = 0.0
            elif key.char == 's':
                self.twist_msg.angular.z = 0.0
        except AttributeError:
            pass
    
    def enviar_mensaje(self):
        self.pub.publish(self.twist_msg)
    
    def revisar_bumpers(self, msg):
        if msg.state == 1:
            self.keyboard_listener.stop()
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
        if msg.state == 0:
            self.keyboard_listener = keyboard.Listener(self.manage_tecla_apretada, self.manage_tecla_soltada)
            self.keyboard_listener.start()



rclpy.init()
nodo = NodoTraductor()
rclpy.spin(nodo)
rclpy.shutdown()