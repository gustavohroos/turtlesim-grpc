#!/usr/bin/env python3
import os
import subprocess
import time
import threading

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose

import grpc
from concurrent import futures
import robot_pb2
import robot_pb2_grpc


# Nó ROS2 para interagir com o turtlesim
class RobotTurtle(Node):
    def __init__(self):
        super().__init__("robot_turtle")
        self.teleport_client = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Esperando pelo serviço Teleport...")
        self.subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.current_pose = Pose()

    def pose_callback(self, msg):
        self.current_pose = msg

    def set_position(self, x, y):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = 0.0
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


# Variáveis globais para gerenciar o processo do turtlesim e o nó ROS2
turtlesim_process = None
rclpy_thread = None
robot_turtle = None


def start_turtlesim():
    global turtlesim_process, rclpy_thread, robot_turtle
    if turtlesim_process is None:
        # Inicia o turtlesim usando o comando ROS2 (já presente na imagem do ROS2 Humble)
        turtlesim_process = subprocess.Popen(
            ["ros2", "run", "turtlesim", "turtlesim_node"]
        )
        # Aguarda o turtlesim iniciar
        time.sleep(3)
        # Inicializa o rclpy e o nó de integração
        rclpy.init()
        robot_turtle = RobotTurtle()
        rclpy_thread = threading.Thread(
            target=rclpy.spin, args=(robot_turtle,), daemon=True
        )
        rclpy_thread.start()
        return True
    return False


def stop_turtlesim():
    global turtlesim_process, robot_turtle
    if turtlesim_process is not None:
        turtlesim_process.terminate()
        turtlesim_process.wait()
        turtlesim_process = None
        if robot_turtle is not None:
            robot_turtle.destroy_node()
            rclpy.shutdown()
        return True
    return False


# Implementação do serviço gRPC
class RobotService(robot_pb2_grpc.RobotServiceServicer):
    def Start(self, request, context):
        if start_turtlesim():
            return robot_pb2.RobotResponse(success=True, message="Turtlesim iniciado")
        else:
            return robot_pb2.RobotResponse(
                success=False, message="Turtlesim já está rodando"
            )

    def Stop(self, request, context):
        if stop_turtlesim():
            return robot_pb2.RobotResponse(success=True, message="Turtlesim parado")
        else:
            return robot_pb2.RobotResponse(
                success=False, message="Turtlesim não está rodando"
            )

    def SetPosition(self, request, context):
        if robot_turtle is None:
            return robot_pb2.RobotResponse(
                success=False, message="Turtlesim não está rodando"
            )
        try:
            robot_turtle.set_position(request.x, request.y)
            return robot_pb2.RobotResponse(success=True, message="Posição definida")
        except Exception as e:
            return robot_pb2.RobotResponse(success=False, message=str(e))

    def GetPosition(self, request, context):
        if robot_turtle is None:
            return robot_pb2.PositionResponse(x=0.0, y=0.0)
        pose = robot_turtle.current_pose
        return robot_pb2.PositionResponse(x=pose.x, y=pose.y)


def serve(robot_id):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_pb2_grpc.add_RobotServiceServicer_to_server(RobotService(), server)
    server.add_insecure_port("[::]:50051")
    server.start()
    print(f"Servidor gRPC do robô {robot_id} iniciado na porta 50051")
    try:
        while True:
            time.sleep(86400)
    except KeyboardInterrupt:
        server.stop(0)


if __name__ == "__main__":
    robot_id = os.environ.get("ROBOT_ID", "robot1")
    serve(robot_id)
