#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist, Pose
from nav_msgs.msg import Odometry
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
        self.sub_real_pose = self.create_subscription(Pose, "/real_pose", self.get_real_position, 1)
        self.real_position = None
        #self.sub_odom_pose = self.create_subscription(Odometry, "/odom", self.get_odom_position, 1)
        #self.odom_position = None
        self.factor_de_correccion = 1.1
        self.correr_callback = True
    
    def delegar_a_thread(self, data):
        if self.correr_callback:
            self.correr_callback = False
            print("Pose array recivido")
            th = Thread(target=self.accion_mover_cb, args=(data,), daemon=True)
            th.start()


    def accion_mover_cb(self, pose_array:PoseArray):
        print(f"Posición real: {(self.real_position)}")
        #print(f"Posición según odometría: {(self.odom_position)}")
        for pose in pose_array.poses:
            self.mover_robot_a_destino(pose)



    def mover_robot_a_destino(self, pose:Pose):
        x_final = pose.position.x
        y_final = pose.position.y
        theta_movimiento = get_theta_mov(x_final, y_final)
        theta_final = pose.orientation.z - theta_movimiento
        theta_final = ajustar_theta(theta_final)

        distancia_lineal = (x_final**2 + y_final**2)**0.5
        rapidez_lineal = 0.2
        rapidez_angular_1 = 1.0 if theta_movimiento >= 0 else -1.0
        rapidez_angular_2 = 1.0 if theta_final >= 0 else -1.0

        tiempo_de_rotacion_1 = abs(theta_movimiento/rapidez_angular_1) * self.factor_de_correccion
        tiempo_de_traslacion = distancia_lineal/rapidez_lineal
        tiempo_de_rotacion_2 = abs(theta_final/rapidez_angular_2) * self.factor_de_correccion

        lista_de_comandos = [(0.0, rapidez_angular_1, tiempo_de_rotacion_1),
                             (rapidez_lineal, 0.0, tiempo_de_traslacion),
                             (0.0, rapidez_angular_2, tiempo_de_rotacion_2)]
        self.aplicar_velocidad(lista_de_comandos)


    def aplicar_velocidad(self, lista_de_comandos:list):
        for i in range(len(lista_de_comandos)):
            self.msg_velocidad.linear.x = lista_de_comandos[i][0]
            self.msg_velocidad.angular.z = lista_de_comandos[i][1]
            sleep(lista_de_comandos[i][2])
        self.msg_velocidad.linear.x = 0.0
        self.msg_velocidad.angular.z = 0.0
        print(f"Posición real: {self.real_position}")
        #print(f"Posición según odometría: {self.odom_position}")


    def publicar_velocidad(self):
        self.pub.publish(self.msg_velocidad)
    

    def get_real_position(self, real_pose:Pose):
        self.real_position = (real_pose.position.x, real_pose.position.y)
    
    def get_odom_position(self, odom:Odometry):
        self.odom_position = (odom.pose.pose.position.x + 1, odom.pose.pose.position.y + 1)



def get_theta_mov(x, y):
    if x == 0:
        if y == 0:
            theta_movimiento = 0.0
        elif y < 0:
            theta_movimiento = -pi/2
        else:
            theta_movimiento = pi/2
    elif x < 0 and y < 0:
        theta_movimiento = pi - atan(y/x)
    elif x < 0:
        theta_movimiento = pi - atan(-y/x)
    elif y < 0:
        theta_movimiento = -atan(-y/x)
    else:
        theta_movimiento = atan(y/x)
    return theta_movimiento



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
nodo.destroy_node()
rclpy.shutdown()