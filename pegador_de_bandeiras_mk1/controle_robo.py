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
    DESVIANDO_DE_OBSTACULO = 5


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(
            Image, '/robot_cam/colored_map', self.camera_callback, 10)

        # Utilizado para converter imagens ROS -> OpenCV
        self.bridge = CvBridge()

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Estado interno
        self.obstaculo_a_frente = False
        self.obstaculo_a_frente_esquerda = False
        self.obstaculo_a_frente_direita = False
        self.obstaculo_a_esquerda = False
        self.obstaculo_a_direita = False
        self.bandeira_a_frente = False
        self.porcentagem_bandeira_na_camera = 0
        self.pos_x_bandeira_camera = -1
        self.centro_x_camera = -1
        self.distancias = None
        self.tempo_desviando = -1
        self.direcao_desvio = -1

        self.estado_atual = Estados.EXPLORANDO
        self.estado_anterior = None

    def scan_callback(self, msg: LaserScan):
        # Verifica uma faixa estreita ao redor de 0° (frente)
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return
        
        distancia_max_obstaculo = 0.67

        # DETECÇÃO DE OBSTACULOS À FRENTE - Índices de -40° a +40°
        indices_frente_esquerda = list(range(0, 40))
        indices_frente_direita = list(range(320, 360))
        indices_frente = indices_frente_esquerda + indices_frente_direita

        self.distancias = [msg.ranges[i] for i in indices_frente]
        self.obstaculo_a_frente = self.distancias and min(self.distancias) < distancia_max_obstaculo

        if self.obstaculo_a_frente:
            self.obstaculo_a_frente_esquerda = False
            self.obstaculo_a_frente_direita = False
            # obstaculo à frente direita
            if msg.ranges.index(min(self.distancias)) in indices_frente_esquerda:
                # self.get_logger().info("Obstaculo detectado à esquerda")
                self.obstaculo_a_frente_esquerda = True
            # obstaculo à frente esquerda
            if msg.ranges.index(min(self.distancias)) in indices_frente_direita:
                # self.get_logger().info("Obstaculo detectado à direita")
                self.obstaculo_a_frente_direita = True

        # DETECÇÃO DE OBSTACULOS À ESQUERDA - Índices de +40° a +90°
        indices_esquerda = list(range(40, 90))
        self.distancias = [msg.ranges[i] for i in indices_esquerda]
        self.obstaculo_a_esquerda = self.distancias and min(self.distancias) < distancia_max_obstaculo
        # DETECÇÃO DE OBSTACULOS À DIREITA - Índices de -40° a -90°
        indices_direita = list(range(270, 320))
        self.distancias = [msg.ranges[i] for i in indices_direita]
        self.obstaculo_a_direita = self.distancias and min(self.distancias) < distancia_max_obstaculo


    def imu_callback(self, msg: Imu):
        return
        # Extraindo o quaternion da mensagem
        orientation_q = msg.orientation
        quat = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]
        # Conversão para Euler usando SciPy
        r = R.from_quat(quat)
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        # Exibindo resultados
        self.get_logger().info('IMU Data Received:')
        self.get_logger().info(
            f'Orientation (Euler): Roll={roll:.2f}°, '
            f'Pitch={pitch:.2f}°, Yaw={yaw:.2f}°'
        )
        self.get_logger().info(
            f'Angular velocity: [{msg.angular_velocity.x:.2f}, '
            f'{msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}] rad/s'
        )
        self.get_logger().info(
            f'Linear acceleration: [{msg.linear_acceleration.x:.2f}, '
            f'{msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}] m/s²'
        )

    def odom_callback(self, msg: Odometry):
        return
        # Mensagens de Odometria das rodas!
        # self.get_logger().info(f"x: {msg.pose.pose.position.x}")
        # self.get_logger().info(f"y: {msg.pose.pose.position.y}")
        # self.get_logger().info(f"z: {msg.pose.pose.position.z}")

    def camera_callback(self, msg: Image):
        # Converte mensagem ROS para imagem OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Calcula centro da camera (x)
        h, w = frame.shape[0], frame.shape[1]
        self.centro_x_camera = w//2

        # Verifica se tem bandeira na câmera:
        # Cor da bandeira na camera semantica
        target_color = np.array([227, 73, 0])
        mask_bandeira = cv2.inRange(frame, target_color, target_color)
        
        contours_bandeira, _ = cv2.findContours(
            mask_bandeira, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # zera a parte superior da mascara para ver apenas o mastro da bandeira
        mask_mastro = mask_bandeira.copy()
        limite_corte = int(h * 0.5)
        mask_mastro[0:limite_corte, :] = 0
        contours_mastro, _ = cv2.findContours(
            mask_mastro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.bandeira_a_frente = len(contours_bandeira) > 0
        if self.bandeira_a_frente:
            cnt = max(contours_bandeira, key=cv2.contourArea)
            M_bandeira = cv2.moments(cnt)
            # usa mascara inteira para calcular porcentagem
            if M_bandeira['m00'] != 0:
                # calcula area da camera ocupada pela bandeira
                area_total = w*h
                area_bandeira = cv2.contourArea(cnt)
                self.porcentagem_bandeira_na_camera = (
                    area_bandeira / area_total) * 100.0
            # usa mascara cortada (apenas mastro) com prioridade para encontrar a posição X da bandeira relativa à camera
            if len(contours_mastro) > 0:
                cnt_mastro = max(contours_mastro, key=cv2.contourArea)
                M_mastro = cv2.moments(cnt_mastro)
                if M_mastro['m00'] != 0:
                    self.pos_x_bandeira_camera = int(
                        M_mastro['m10'] / M_mastro['m00'])
            else:
                if M_bandeira['m00'] != 0:
                    self.pos_x_bandeira_camera = int(
                        M_bandeira['m10'] / M_bandeira['m00'])
        else:
            self.bandeira_a_frente = False
            self.porcentagem_bandeira_na_camera = 0.0

    def move_robot(self):
        if self.estado_anterior != self.estado_atual:
            self.get_logger().info(f"## {self.estado_atual.name} ##")
            self.estado_anterior = self.estado_atual

        twist = Twist()
        dx = 25
        base_vel_angular = 0.3
        base_vel_linear = 0.4
        bandeira_centralizada = self.pos_x_bandeira_camera <= self.centro_x_camera + dx and self.pos_x_bandeira_camera >= self.centro_x_camera - dx
        direcao_bandeira = 1 if (self.pos_x_bandeira_camera < self.centro_x_camera - dx) else -1

        ## EXPLORANDO: o robô vai para frente
        if self.estado_atual == Estados.EXPLORANDO:
            if not self.obstaculo_a_frente:
                # Se encontrou a bandeira e não tem obstáculo no caminho, vai para o próximo estado
                if self.bandeira_a_frente:
                    self.get_logger().info("Bandeira detectada, iniciando alinhamento")
                    self.estado_atual = Estados.BANDEIRA_DETECTADA
                # Caso contrario, continua andando para frente
                else:
                    twist.linear.x = base_vel_linear
            # Se tem obstaculo no caminho, desvia para o lado oposto
            else:
                self.tempo_desviando = 15  # desvia por 15 loops
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO

        ## DESVIANDO DE OBSTACULO: o robô vira para o lado oposto do obstáculo e anda para frente
        elif self.estado_atual == Estados.DESVIANDO_DE_OBSTACULO:
            if self.obstaculo_a_frente:
                self.direcao_desvio = 1 if self.obstaculo_a_frente_direita else -1
            
                if self.obstaculo_a_frente_direita and self.obstaculo_a_frente_esquerda:
                    self.get_logger().info("Obstaculo detectado à frente (dos dois lados)")
                    twist.angular.z = base_vel_angular * self.direcao_desvio * -1
                    twist.linear.x = base_vel_linear * 0.5 * -1
                else:
                    self.get_logger().info("Obstaculo detectado à " + ("direita" if self.obstaculo_a_frente_direita else "esquerda"))
                    twist.angular.z = base_vel_angular * self.direcao_desvio

                self.tempo_desviando = 15

            elif self.tempo_desviando > 0:
                twist.angular.z = base_vel_angular * self.direcao_desvio
                twist.linear.x = base_vel_linear * 0.7
                self.tempo_desviando -= 1

            else:
                self.estado_atual = Estados.EXPLORANDO

        ## BANDEIRA DETECTADA: o robô alinha-se com a bandeira
        elif self.estado_atual == Estados.BANDEIRA_DETECTADA:
            # se perdeu a bandeira por algum motivo, volta a explorar
            if not self.bandeira_a_frente:
                self.get_logger().info("Bandeira perdida, voltando para o estado explorando")
                self.estado_atual = Estados.EXPLORANDO
            # Detectou obstaculo que não é a bandeira durante a centralização da bandeira, volta a explorar
            elif self.obstaculo_a_frente and self.porcentagem_bandeira_na_camera < 15:
                self.get_logger().info("Obstaculo detectado, desviando")
                self.tempo_desviando = 15  # desvia por 15 loops
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO

            # Alinha robo com a bandeira
            elif not bandeira_centralizada:
                # Se tem obstaculo na direção da bandeira, não vira, só vai pra frente
                if (direcao_bandeira > 0 and self.obstaculo_a_esquerda) or (direcao_bandeira < 0 and self.obstaculo_a_direita):  
                    twist.linear.x = base_vel_linear
                else:
                    twist.angular.z = base_vel_angular * direcao_bandeira
            else:
                # alinhado, vai para o próximo estado
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA

        ## NAVEGANDO PARA BANDEIRA: o robô anda na direção da bandeira
        elif self.estado_atual == Estados.NAVEGANDO_PARA_BANDEIRA:
            # se perdeu a bandeira por algum motivo, volta a explorar
            if not self.bandeira_a_frente:
                self.get_logger().info("Bandeira perdida, voltando para o estado explorando")
                self.estado_atual = Estados.EXPLORANDO

            elif self.obstaculo_a_frente:
                if self.porcentagem_bandeira_na_camera > 10:
                    # Se a bandeira esta grande na camera, chegou perto da bandeira
                    twist.linear.x = 0.0
                    self.get_logger().info("Bandeira alcançada!")
                    self.estado_atual = Estados.POSICIONANDO_PARA_COLETA

                else:
                    # Caso contrário, é um obstaculo normal
                    self.get_logger().info("Obstaculo detectado, desviando")
                    self.tempo_desviando = 15  # desvia por 15 loops
                    self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            else:
                # Vai em direção à bandeira
                twist.linear.x = base_vel_linear
                if not bandeira_centralizada:
                    twist.angular.z = (base_vel_angular * 0.5) * direcao_bandeira

        ## POSICIONANDO PARA COLETA: o robô a se alinha com o mastro da bandeira
        elif self.estado_atual == Estados.POSICIONANDO_PARA_COLETA:
            # Permanece parado na frente da bandeira
            twist.linear.x = 0.0
            dx = 5
            # Alinha robo com a bandeira
            if not bandeira_centralizada:
                twist.angular.z = (base_vel_angular * 0.5) * direcao_bandeira

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
