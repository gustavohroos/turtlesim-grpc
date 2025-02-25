#!/usr/bin/env python3
import os
import subprocess
import time
import threading

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import grpc
from concurrent import futures
import robot_pb2
import robot_pb2_grpc


class RobotTurtle(Node):
    def __init__(self):
        super().__init__("robot_turtle")
        # Parâmetros do círculo
        self.center_x = 5.5  # centro padrão X
        self.center_y = 5.5  # centro padrão Y
        self.radius = 2.0  # raio do círculo
        self.v = 1.0  # velocidade linear
        self.moving = False  # flag de movimento circular

        self._lock = threading.Lock()
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Cliente para o serviço TeleportAbsolute
        self.teleport_client = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Esperando pelo serviço TeleportAbsolute...")

        # Para obter a posição (se precisar futuramente)
        self.subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.current_pose = Pose()
        self._movement_thread = None

    def pose_callback(self, msg):
        self.current_pose = msg

    def start_circle(self):
        with self._lock:
            if self.moving:
                return False
            # Teleporta o turtle para o ponto inicial do círculo: (center_x + radius, center_y)
            req = TeleportAbsolute.Request()
            req.x = self.center_x + self.radius
            req.y = self.center_y
            req.theta = 1.5708  # Orientação para cima (pi/2)
            future = self.teleport_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            # Inicia o movimento circular
            self.moving = True
            self._movement_thread = threading.Thread(
                target=self._publish_circle, daemon=True
            )
            self._movement_thread.start()
            return True

    def _publish_circle(self):
        twist = Twist()
        # Comando para movimento circular: linear e angular constantes
        twist.linear.x = self.v
        twist.angular.z = self.v / self.radius
        while self.moving:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        # Ao parar, envia comando de zero velocidade
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def stop_circle(self):
        with self._lock:
            if not self.moving:
                return False
            self.moving = False
            if self._movement_thread is not None:
                self._movement_thread.join()
                self._movement_thread = None
            return True

    def set_center(self, x, y):
        with self._lock:
            self.center_x = x
            self.center_y = y
            # Se estiver em movimento, reposiciona o turtle para iniciar o círculo no novo centro
            if self.moving:
                req = TeleportAbsolute.Request()
                req.x = self.center_x + self.radius
                req.y = self.center_y
                req.theta = 1.5708
                future = self.teleport_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
            return True


# Implementação do serviço gRPC para controle do robô
class RobotService(robot_pb2_grpc.RobotServiceServicer):
    def __init__(self, robot_turtle: RobotTurtle):
        self.robot_turtle = robot_turtle

    def Start(self, request, context):
        if self.robot_turtle.start_circle():
            return robot_pb2.RobotResponse(
                success=True, message="Movimento circular iniciado"
            )
        else:
            return robot_pb2.RobotResponse(
                success=False, message="Turtle já está se movendo"
            )

    def Stop(self, request, context):
        if self.robot_turtle.stop_circle():
            return robot_pb2.RobotResponse(
                success=True, message="Movimento interrompido"
            )
        else:
            return robot_pb2.RobotResponse(
                success=False, message="Turtle não está em movimento"
            )

    def SetPosition(self, request, context):
        # Altera o centro do círculo
        if self.robot_turtle.set_center(request.x, request.y):
            return robot_pb2.RobotResponse(
                success=True, message="Centro do círculo atualizado"
            )
        else:
            return robot_pb2.RobotResponse(
                success=False, message="Falha ao atualizar o centro"
            )

    def GetPosition(self, request, context):
        pose = self.robot_turtle.current_pose
        return robot_pb2.PositionResponse(x=pose.x, y=pose.y)


def serve(robot_id):
    # Inicia o servidor gRPC e o nó ROS2
    rclpy.init()
    robot_turtle = RobotTurtle()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_pb2_grpc.add_RobotServiceServicer_to_server(
        RobotService(robot_turtle), server
    )
    server.add_insecure_port("[::]:50051")
    server.start()
    print(f"Servidor gRPC do robô {robot_id} iniciado na porta 50051")
    try:
        while rclpy.ok():
            rclpy.spin_once(robot_turtle, timeout_sec=0.1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop(0)
        robot_turtle.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    robot_id = os.environ.get("ROBOT_ID", "robot1")
    serve(robot_id)
