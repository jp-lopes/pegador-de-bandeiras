#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from scipy.spatial.transform import Rotation as R

from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum


class Estados(Enum):
    EXPLORANDO = 1
    BANDEIRA_DETECTADA = 2
    NAVEGANDO_PARA_BANDEIRA = 3
    POSICIONANDO_PARA_COLETA = 4


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            Image, '/robot_cam/colored_map', self.camera_callback, 10)

        # Utilizado para converter imagens ROS -> OpenCV
        self.bridge = CvBridge()

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Estado interno
        self.obstaculo_a_frente = False
        self.bandeira_a_frente = False
        self.estado_atual = Estados.EXPLORANDO

        self.pos_x_bandeira = -1
        self.centro_x = -1

    def scan_callback(self, msg: LaserScan):
        # Verifica uma faixa estreita ao redor de 0° (frente)
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Índices de -30° a +30° (equivalente a 330 até 30)
        indices_frente = list(range(330, 360)) + list(range(0, 31))

        # Filtra distancias
        distancias = [msg.ranges[i] for i in indices_frente]

        if distancias and min(distancias) < 0.5:
            self.obstaculo_a_frente = True
            # self.get_logger().info('Obstáculo detectado a {:.2f}m à frente'.format(min(distancias)))
        else:
            self.obstaculo_a_frente = False

    def imu_callback(self, msg: Imu):
        pass

    def odom_callback(self, msg: Odometry):
        # Mensagens de Odometria das rodas!
        pass

    def camera_callback(self, msg: Image):
        # Converte mensagem ROS para imagem OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[0], frame.shape[1]
        self.centro_x = w//2

        # Define a cor-alvo em BGR
        target_color = np.array([227, 73, 0])  # OBS: OpenCV usa BGR
        cor_cilidro = np.array([236, 74, 0])  # OBS: OpenCV usa BGR

        # Cria máscara para cor exata
        mask = cv2.inRange(frame, target_color, target_color)

        # # Mostra a máscara em uma janela para debug
        # cv2.imshow('Mascara de Blobs #00f2ab', mask)
        # cv2.waitKey(1)  # Tempo mínimo para a janela atualizar (1 ms)

        # Detecta contornos (blobs)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.bandeira_a_frente = len(contours) > 0

        for i, cnt in enumerate(contours):
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # self.get_logger().info(f'  Blob {i+1}: posição (x={cx}, y={cy})')
                self.pos_x_bandeira = cx

    def move_robot(self):
        twist = Twist()
        dx = 5

        if self.estado_atual == Estados.EXPLORANDO:
            if not self.obstaculo_a_frente:
                twist.linear.x = 0.5  # Move para frente
            else:
                twist.angular.z = -0.3  # Gira em torno do proprio eixo
            if self.bandeira_a_frente:
                self.estado_atual = Estados.BANDEIRA_DETECTADA

        elif self.estado_atual == Estados.BANDEIRA_DETECTADA:

            if self.pos_x_bandeira < self.centro_x - dx:
                twist.angular.z = 0.3  # Gira em torno do proprio eixo
            elif self.pos_x_bandeira > self.centro_x + dx:
                twist.angular.z = -0.3  # Gira em torno do proprio eixo
            elif self.pos_x_bandeira < self.centro_x + dx and self.pos_x_bandeira > self.centro_x - dx:
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA

        if self.estado_atual == Estados.NAVEGANDO_PARA_BANDEIRA:
            if not self.obstaculo_a_frente:
                if self.bandeira_a_frente and not (self.pos_x_bandeira < self.centro_x + dx and self.pos_x_bandeira > self.centro_x - dx):
                    self.estado_atual = Estados.BANDEIRA_DETECTADA
                twist.linear.x = 0.5  # Move para frente
            else:
                twist.angular.z = -0.3  # Gira em torno do proprio eixo

            if not self.bandeira_a_frente:
                self.estado_atual = Estados.EXPLORANDO

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
