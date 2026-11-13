
#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        # ==========================================
        # CONFIGURACION DEL RECORRIDO
        # ==========================================

        # Distancias de los cuatro lados y el tramo final.
        self.side_lengths = [9.0, 7.0, 22.0, 7.0, 13.0]

        # Distancia antes de cada esquina para comenzar a girar.
        self.turn_advance = 0.35

        # Velocidades (m/s).
        self.straight_speed = 1.0
        self.turn_speed = 0.7

        # Control de direccion.
        self.max_steering = 0.42
        self.min_steering = 0.12
        self.turn_tolerance = 0.06
        self.turn_gain = 1.2

        # Distancia de tolerancia para finalizar.
        self.finish_tolerance = 0.25

        # ==========================================
        # ESTADO DEL VEHICULO
        # ==========================================

        self.state = 'WAIT_ODOM'
        self.initialized = False

        # Posicion inicial absoluta.
        self.initial_x = 0.0
        self.initial_y = 0.0

        # Posicion de inicio del tramo actual.
        self.start_x = 0.0
        self.start_y = 0.0

        self.current_yaw = 0.0
        self.initial_yaw = 0.0
        self.target_yaw = 0.0
        self.segment_start_yaw = 0.0

        self.completed_turns = 0
        self.total_turns = 4

        # ==========================================
        # TOPICOS ROS 2
        # ==========================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.get_logger().info(
            'Square Driver iniciado. Esperando odometria...'
        )

    # ==============================================
    # PUBLICAR COMANDOS ACKERMANN
    # ==============================================

    def publish_drive(self, speed, steering):

        msg = AckermannDriveStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)

        self.drive_pub.publish(msg)

    # ==============================================
    # CONVERTIR CUATERNION A YAW
    # ==============================================

    def get_yaw(self, orientation):

        siny_cosp = 2.0 * (
            orientation.w * orientation.z +
            orientation.x * orientation.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            orientation.y ** 2 +
            orientation.z ** 2
        )

        return math.atan2(siny_cosp, cosy_cosp)

    # ==============================================
    # CALLBACK DE ODOMETRIA
    # ==============================================

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.current_yaw = self.get_yaw(
            msg.pose.pose.orientation
        )

        # ------------------------------------------
        # INICIALIZACION
        # ------------------------------------------

        if not self.initialized:

            self.initial_x = x
            self.initial_y = y

            self.start_x = x
            self.start_y = y

            self.initial_yaw = self.current_yaw
            self.target_yaw = self.current_yaw
            self.segment_start_yaw = self.current_yaw

            self.initialized = True
            self.state = 'STRAIGHT'

            self.get_logger().info(
                f'Posicion inicial: x={x:.2f}, y={y:.2f}, '
                f'yaw={self.current_yaw:.2f}'
            )

            return

        # ------------------------------------------
        # SI YA TERMINO EL RECORRIDO
        # ------------------------------------------

        if self.state == 'DONE':

            self.publish_drive(0.0, 0.0)
            return

        # ------------------------------------------
        # DISTANCIA RECORRIDA EN EL TRAMO ACTUAL
        # ------------------------------------------

        dx = x - self.start_x
        dy = y - self.start_y

        distance = (
            dx * math.cos(self.segment_start_yaw) +
            dy * math.sin(self.segment_start_yaw)
        )

        # ==========================================
        # ESTADO: AVANCE EN LINEA RECTA
        # ==========================================

        if self.state == 'STRAIGHT':

            # --------------------------------------
            # CONTROL DE ORIENTACION
            # --------------------------------------

            steering_error = normalize_angle(
                self.target_yaw - self.current_yaw
            )

            steering = clamp(
                1.5 * steering_error,
                -0.10,
                0.10
            )

            # --------------------------------------
            # COMPROBAR SI ES EL TRAMO FINAL
            # --------------------------------------

            if self.completed_turns == self.total_turns:

                # Distancia euclidiana hasta el punto inicial.
                distance_to_start = math.sqrt(
                    (x - self.initial_x) ** 2 +
                    (y - self.initial_y) ** 2
                )

                # Frenar cuando se aproxime al punto inicial.
                if distance_to_start <= self.finish_tolerance:

                    self.state = 'DONE'
                    self.publish_drive(0.0, 0.0)

                    self.get_logger().info(
                        '¡Recorrido finalizado! '
                    )

                    self.get_logger().info(
                        f'Posicion final: x={x:.2f}, y={y:.2f}'
                    )

                    return

                # Reducir velocidad al aproximarse al inicio.
                if distance_to_start < 2.0:
                    self.publish_drive(0.5, steering)
                else:
                    self.publish_drive(
                        self.straight_speed,
                        steering
                    )

                return

            # --------------------------------------
            # COMPROBAR DISTANCIA DEL LADO ACTUAL
            # --------------------------------------

            current_side_length = self.side_lengths[
                self.completed_turns
            ]

            if distance >= (
                current_side_length - self.turn_advance
            ):

                # Preparar el siguiente giro de 90 grados.
                self.target_yaw = normalize_angle(
                    self.target_yaw + math.pi / 2.0
                )

                self.state = 'TURN'

                self.get_logger().info(
                    f'Iniciando giro '
                    f'{self.completed_turns + 1}/4'
                )

                self.publish_drive(self.turn_speed, 0.0)
                return

            self.publish_drive(
                self.straight_speed,
                steering
            )

        # ==========================================
        # ESTADO: GIRO DE 90 GRADOS
        # ==========================================

        elif self.state == 'TURN':

            angle_error = normalize_angle(
                self.target_yaw - self.current_yaw
            )

            # Control proporcional de direccion.
            steering = self.turn_gain * angle_error

            steering = clamp(
                steering,
                -self.max_steering,
                self.max_steering
            )

            if abs(angle_error) > self.turn_tolerance:

                if abs(steering) < self.min_steering:

                    steering = math.copysign(
                        self.min_steering,
                        angle_error
                    )

            # --------------------------------------
            # COMPROBAR SI TERMINO EL GIRO
            # --------------------------------------

            if abs(angle_error) <= self.turn_tolerance:

                self.completed_turns += 1

                self.get_logger().info(
                    f'Giro completado: '
                    f'{self.completed_turns}/4'
                )

                # Reiniciar medicion del siguiente tramo.
                self.start_x = x
                self.start_y = y

                self.segment_start_yaw = self.target_yaw

                self.state = 'STRAIGHT'

                self.publish_drive(
                    self.straight_speed,
                    0.0
                )

                return

            self.publish_drive(
                self.turn_speed,
                steering
            )


# ==============================================
# FUNCION PRINCIPAL
# ==============================================

def main(args=None):

    rclpy.init(args=args)

    node = SquareDriver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

