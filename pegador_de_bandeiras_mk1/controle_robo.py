#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
import random


class Estados(Enum):
    EXPLORANDO = 1
    DESVIANDO_DE_OBSTACULO = 2
    NAVEGANDO_PARA_BANDEIRA = 3
    POSICIONANDO_PARA_COLETA = 4
    CAPTURANDO_BANDEIRA = 5

class DirecoesRotacao(Enum):
    ESQUERDA = 1
    DIREITA = -1


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher para controle da garra
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
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
        self.tempo_desviando = -1
        self.direcao_desvio = -1

        # DETECÇÃO DE OBSTACULOS À FRENTE - Índices de -35° a +35°
        self.indices_frente_esquerda = list(range(0, 35))
        self.indices_frente_direita = list(range(325, 360))


        self.estado_atual = Estados.EXPLORANDO
        self.estado_anterior = None

    def scan_callback(self, msg: LaserScan):
        # Verifica uma faixa estreita ao redor de 0° (frente)
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return
        
        distancia_max_obstaculo_frente = 0.62
        distancia_max_obstaculo_lados = 0.40

        # DETECÇÃO DE OBSTACULOS À FRENTE - Índices de -35° a +35°
        distancias = [msg.ranges[i] for i in self.indices_frente_esquerda]
        self.obstaculo_a_frente_esquerda = distancias and min(distancias) < distancia_max_obstaculo_frente

        distancias = [msg.ranges[i] for i in self.indices_frente_direita]
        self.obstaculo_a_frente_direita = distancias and min(distancias) < distancia_max_obstaculo_frente

        self.obstaculo_a_frente = self.obstaculo_a_frente_direita or self.obstaculo_a_frente_esquerda

        # DETECÇÃO DE OBSTACULOS À ESQUERDA - Índices de +35° a +90°
        indices_esquerda = list(range(35, 90))
        distancias = [msg.ranges[i] for i in indices_esquerda]
        self.obstaculo_a_esquerda = distancias and min(distancias) < distancia_max_obstaculo_lados
        # DETECÇÃO DE OBSTACULOS À DIREITA - Índices de -35° a -90°
        indices_direita = list(range(270, 325))
        distancias = [msg.ranges[i] for i in indices_direita]
        self.obstaculo_a_direita = distancias and min(distancias) < distancia_max_obstaculo_lados


    def imu_callback(self, msg: Imu):
        return
        # Extraindo o quaternion da mensagem
        # orientation_q = msg.orientation
        # quat = [
        #     orientation_q.x,
        #     orientation_q.y,
        #     orientation_q.z,
        #     orientation_q.w
        # ]
        # # Conversão para Euler usando SciPy
        # r = R.from_quat(quat)
        # roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        # # Exibindo resultados
        # self.get_logger().info('IMU Data Received:')
        # self.get_logger().info(
        #     f'Orientation (Euler): Roll={roll:.2f}°, '
        #     f'Pitch={pitch:.2f}°, Yaw={yaw:.2f}°'
        # )
        # self.get_logger().info(
        #     f'Angular velocity: [{msg.angular_velocity.x:.2f}, '
        #     f'{msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}] rad/s'
        # )
        # self.get_logger().info(
        #     f'Linear acceleration: [{msg.linear_acceleration.x:.2f}, '
        #     f'{msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}] m/s²'
        # )

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


    def mover_garra(self, gripper_extension, right_gripper_joint, left_gripper_joint):
            msg = Float64MultiArray()
            msg.data = [gripper_extension, right_gripper_joint, left_gripper_joint]
            self.gripper_pub.publish(msg)


    def move_robot(self):
        if self.estado_anterior != self.estado_atual:
            self.get_logger().info(f"## {self.estado_atual.name} ##")
            self.estado_anterior = self.estado_atual

        twist = Twist()
        dx = 15
        base_vel_angular = 0.3
        base_vel_linear = 0.4
        bandeira_centralizada = self.pos_x_bandeira_camera <= self.centro_x_camera + dx and self.pos_x_bandeira_camera >= self.centro_x_camera - dx
        direcao_bandeira = 1 if (self.pos_x_bandeira_camera < self.centro_x_camera - dx) else -1

        ## EXPLORANDO: o robô vai para frente
        if self.estado_atual == Estados.EXPLORANDO:
            if not self.obstaculo_a_frente:
                # Se encontrou a bandeira e não tem obstáculo no caminho, vai para o próximo estado
                if self.bandeira_a_frente:
                    self.get_logger().info("Bandeira detectada! Iniciando navegação em direção à bandeira.")
                    self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
                # Caso contrario, continua andando para frente
                else:
                    twist.linear.x = base_vel_linear
                    if self.obstaculo_a_esquerda and not self.obstaculo_a_direita:
                        # Se tiver algo à esquerda, gira levemente para a direita
                        twist.angular.z = base_vel_angular * DirecoesRotacao.DIREITA.value * 0.5 
                    elif self.obstaculo_a_direita and not self.obstaculo_a_esquerda:
                        # Se tiver algo à direita, gira levemente para a esquerda
                        twist.angular.z = base_vel_angular * DirecoesRotacao.ESQUERDA.value * 0.5

            # Se tem obstaculo no caminho, entra no modo de desvio
            else:
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO

        ## DESVIANDO DE OBSTACULO: o robô vira para o lado oposto do obstáculo e anda para frente
        elif self.estado_atual == Estados.DESVIANDO_DE_OBSTACULO:
            if self.obstaculo_a_frente:
                obstaculo_lados = self.obstaculo_a_direita or self.obstaculo_a_esquerda
                # Obstaculo apenas à frente direita
                if self.obstaculo_a_frente_direita and not self.obstaculo_a_frente_esquerda and not obstaculo_lados:
                    self.direcao_desvio = DirecoesRotacao.ESQUERDA
                # Obstáculo apenas à frente esquerda
                elif self.obstaculo_a_frente_esquerda and not self.obstaculo_a_frente_direita and not obstaculo_lados:
                    self.direcao_desvio = DirecoesRotacao.DIREITA
                # Obstáculo apenas nas 2 frentes:
                elif not obstaculo_lados:
                    # vira para lado aleatório
                    self.direcao_desvio = random.choice([DirecoesRotacao.ESQUERDA, DirecoesRotacao.DIREITA])
                # Obstáculo nas duas frentes E na direita:
                elif self.obstaculo_a_direita:
                    self.direcao_desvio = DirecoesRotacao.ESQUERDA
                # Obstáculo nas duas frentes E na esquerda:
                elif self.obstaculo_a_esquerda:
                    self.direcao_desvio = DirecoesRotacao.DIREITA
                
                twist.angular.z = base_vel_angular * self.direcao_desvio.value * 0.75
                self.tempo_desviando = 5  # depois de sair da frente do obstaculo, desvia por mais 5 loops

            elif self.tempo_desviando > 0:
                twist.angular.z = base_vel_angular * self.direcao_desvio.value * 0.75
                twist.linear.x = base_vel_linear * 0.5
                self.tempo_desviando -= 1

            else:
                self.estado_atual = Estados.EXPLORANDO

        ## NAVEGANDO PARA BANDEIRA: o robô anda na direção da bandeira
        elif self.estado_atual == Estados.NAVEGANDO_PARA_BANDEIRA:
            # se perdeu a bandeira por algum motivo, volta a explorar
            if not self.bandeira_a_frente:
                self.get_logger().info("Bandeira perdida, voltando para o estado de exploração.")
                self.estado_atual = Estados.EXPLORANDO
            # Detectou obstaculo que não é a bandeira durante a centralização da bandeira, volta a explorar
            elif self.obstaculo_a_frente and self.porcentagem_bandeira_na_camera < 5.0:
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            # se já está alinhado e chegou perto da bandeira (5% ou mais na tela) vai p/ o próximo estado
            elif self.porcentagem_bandeira_na_camera > 5.0:
                    twist.linear.x = 0.0
                    self.get_logger().info("Bandeira alcançada! Iniciando alinhamento para coleta.")
                    self.estado_atual = Estados.POSICIONANDO_PARA_COLETA
            # Alinha robo com a bandeira
            elif (direcao_bandeira > 0 and self.obstaculo_a_esquerda) or (direcao_bandeira < 0 and self.obstaculo_a_direita):
                twist.linear.x = base_vel_linear * 0.70
                twist.angular.z = base_vel_angular * 0.20 * (direcao_bandeira if not bandeira_centralizada else 0)
            else:
                twist.linear.x = base_vel_linear 
                twist.angular.z = base_vel_angular * (direcao_bandeira if not bandeira_centralizada else 0)

        ## POSICIONANDO PARA COLETA: o robô a se alinha com o mastro da bandeira
        elif self.estado_atual == Estados.POSICIONANDO_PARA_COLETA:
            dx = 5  # diminui margem para centralizar melhor
            bandeira_centralizada = self.pos_x_bandeira_camera <= self.centro_x_camera + dx and self.pos_x_bandeira_camera >= self.centro_x_camera - dx
            direcao_bandeira = 1 if (self.pos_x_bandeira_camera < self.centro_x_camera - dx) else -1
            # se perdeu a bandeira por algum motivo, volta a explorar
            if not self.bandeira_a_frente:
                self.get_logger().info("Bandeira perdida, voltando para o estado de exploração.")
                self.estado_atual = Estados.EXPLORANDO
            # Alinha robo com a bandeira
            elif not bandeira_centralizada:
                self.get_logger().info("Centralizando garra com o mastro da bandeira...")
                twist.angular.z = (base_vel_angular * 0.25) * direcao_bandeira
            # Chega o mais próximo o possível da bandeira
            elif not self.obstaculo_a_frente:
                self.get_logger().info("Aproximando da bandeira...")
                twist.linear.x = 0.1
            else:
                self.get_logger().info("Garra posicionada!")
                self.estado_atual = Estados.CAPTURANDO_BANDEIRA

        ## CAPTURANDO BANDEIRA
        elif self.estado_atual == Estados.CAPTURANDO_BANDEIRA:
            # Garante que o robô fique parado
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.mover_garra(0.36, -0.05, 0.05)

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
