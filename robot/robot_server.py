#!/usr/bin/env python3
import os
import time
import threading
import subprocess

import grpc
import robot_pb2
import robot_pb2_grpc

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist

SERVER_ADDRESS = os.environ.get("SERVER_ADDRESS", "localhost:50051")
ROBOT_ID = os.environ.get("ROBOT_ID", "robot1")

class RobotTurtle(Node):
    def __init__(self):
        super().__init__('robot_turtle')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for TeleportAbsolute service...')
        self.moving = False
        self._movement_thread = None
        self.x_center = 5.5
        self.y_center = 5.5
        self.radius = 2.0
        self.speed = 1.0

    def start_circle(self):
        if self.moving:
            return
        req = TeleportAbsolute.Request()
        req.x = self.x_center + self.radius
        req.y = self.y_center
        req.theta = 1.5708
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.moving = True
        self._movement_thread = threading.Thread(target=self._publish_circle)
        self._movement_thread.start()

    def _publish_circle(self):
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.speed / self.radius
        while self.moving:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def stop_circle(self):
        self.moving = False
        if self._movement_thread:
            self._movement_thread.join()

def gRPC_client(robot_turtle: RobotTurtle):
    channel = grpc.insecure_channel(SERVER_ADDRESS)
    stub = robot_pb2_grpc.RobotServiceStub(channel)

    def request_generator():
        reg = robot_pb2.Registration(robot_id=ROBOT_ID)
        msg = robot_pb2.RobotMessage(registration=reg)
        yield msg
        while True:
            time.sleep(60)

    try:
        responses = stub.Connect(request_generator())
        for msg in responses:
            if msg.HasField("command"):
                cmd = msg.command
                if cmd.type == robot_pb2.CommandType.START:
                    robot_turtle.get_logger().info("Received START command")
                    robot_turtle.start_circle()
                elif cmd.type == robot_pb2.CommandType.STOP:
                    robot_turtle.get_logger().info("Received STOP command")
                    robot_turtle.stop_circle()
                elif cmd.type == robot_pb2.CommandType.SET_POSITION:
                    robot_turtle.get_logger().info(f"Received SET_POSITION command: x={cmd.x}, y={cmd.y}")
                    robot_turtle.x_center = cmd.x
                    robot_turtle.y_center = cmd.y
                    req = TeleportAbsolute.Request()
                    req.x = robot_turtle.x_center + robot_turtle.radius
                    req.y = robot_turtle.y_center
                    req.theta = 1.5708
                    future = robot_turtle.teleport_client.call_async(req)
                    rclpy.spin_until_future_complete(robot_turtle, future)
                elif cmd.type == robot_pb2.CommandType.GET_POSITION:
                    robot_turtle.get_logger().info("Received GET_POSITION command")
                    # Implementar resposta com posição se necessário
                else:
                    robot_turtle.get_logger().warning("Unknown command received")
    except Exception as e:
        robot_turtle.get_logger().error("gRPC client error: " + str(e))

def main():

    rclpy.init()
    robot_turtle = RobotTurtle()

    grpc_thread = threading.Thread(target=gRPC_client, args=(robot_turtle,))
    grpc_thread.daemon = True
    grpc_thread.start()

    try:
        rclpy.spin(robot_turtle)
    except KeyboardInterrupt:
        robot_turtle.get_logger().info("Shutting down robot")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
