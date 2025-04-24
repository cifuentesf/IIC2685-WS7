#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist, Pose
from math import pi, atan
from time import sleep
from threading import Thread



class DeadReckoningNav(Node):
    def __init__(self):
        super().__init__("dead_reckoning_nav")
        self.sub = self.create_subscription(PoseArray, "goal_list", self.delegar_a_thread, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel_mux/input/navigation", 10)
        self.msg_velocidad = Twist()
        self.msg_velocidad.linear.x = 0.0
        self.msg_velocidad.angular.z = 0.0
        self.create_timer(0.1, self.publicar_velocidad)
    
    def delegar_a_thread(self, data):
        th = Thread(target=self.accion_mover_cb, args=(data,))
        th.start()


    def accion_mover_cb(self, pose_array:PoseArray):
        for pose in pose_array.poses:
            self.mover_robot_a_destino(pose)



    def mover_robot_a_destino(self, pose:Pose):
        x_final = pose.position.x
        y_final = pose.position.y
        theta_movimiento = atan(y_final/x_final)#################
        theta_final = pose.orientation.z - theta_movimiento
        theta_final = ajustar_theta(theta_final)
    
        print(theta_movimiento*180/pi, theta_final*180/pi, pose.orientation.z, "###############################################")

        distancia_lineal = (x_final**2 + y_final**2)**0.5
        rapidez_lineal = 0.2
        rapidez_angular_1 = 1.0 if theta_movimiento >= 0 else -1.0
        rapidez_angular_2 = 1.0 if theta_final >= 0 else -1.0

        tiempo_de_rotacion_1 = abs(theta_movimiento/rapidez_angular_1)
        tiempo_de_traslacion = distancia_lineal/rapidez_lineal
        tiempo_de_rotacion_2 = abs(theta_final/rapidez_angular_2)

        lista_de_comandos = [(0.0, rapidez_angular_1, tiempo_de_rotacion_1),
                             (rapidez_lineal, 0.0, tiempo_de_traslacion),
                             (0.0, rapidez_angular_2, tiempo_de_rotacion_2)]
        print(lista_de_comandos)
        self.aplicar_velocidad(lista_de_comandos)


    def aplicar_velocidad(self, lista_de_comandos:list):
        for i in range(len(lista_de_comandos)):
            self.msg_velocidad.linear.x = lista_de_comandos[i][0]
            self.msg_velocidad.angular.z = lista_de_comandos[i][1]
            sleep(lista_de_comandos[i][2])
        self.msg_velocidad.linear.x = 0.0
        self.msg_velocidad.angular.z = 0.0


    def publicar_velocidad(self):
        print(f"Publicando velocidad: lin {self.msg_velocidad.linear.x}  ang {self.msg_velocidad.angular.z}")
        self.pub.publish(self.msg_velocidad)




def ajustar_theta(theta):
    if abs(theta) > 2*pi:
        theta = theta % (2*pi)
    if theta > pi:
        theta -= 2*pi
    elif theta < -pi:
        theta += 2*pi
    return theta




rclpy.init()
nodo = DeadReckoningNav()
rclpy.spin(nodo)
rclpy.shutdown()