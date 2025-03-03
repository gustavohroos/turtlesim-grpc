import grpc
import time
import queue
import threading
import os
import time
import threading
import subprocess

import protobuf.robot_service_pb2 as robot_pb2
import protobuf.robot_service_pb2_grpc as robot_pb2_grpc
import protobuf.common_pb2 as common_pb2

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist

SERVER_ADDRESS = os.environ.get("SERVER_ADDRESS", "localhost:50051")
ROBOT_ID = os.environ.get("ROBOT_ID", "robot1")


class RobotTurtle(Node):
    def __init__(self):
        super().__init__("robot_turtle")
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.teleport_client = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for TeleportAbsolute service...")
        self.moving = False
        self._movement_thread = None
        self.x_center = 5.5
        self.y_center = 5.5
        self.radius = 2.0
        self.speed = 2.0

    def start_circle(self):
        if self.moving:
            return
        req = TeleportAbsolute.Request()
        req.x = self.x_center + self.radius
        req.y = self.y_center
        req.theta = 1.5708
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done():
            self.get_logger().error("Teleport service did not complete in time")
            return
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

    def set_position(self, x, y):
        self.x_center = x
        self.y_center = y
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = 1.5708
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def get_position(self):
        return self.x_center, self.y_center


outgoing_queue = queue.Queue()


def request_generator():
    registration = robot_pb2.Registration(robot_id=ROBOT_ID)
    outgoing_queue.put(robot_pb2.RobotMessage(registration=registration))
    while True:
        msg = outgoing_queue.get()
        yield msg


def keep_alive_worker():
    while True:
        time.sleep(60)
        keep_alive = common_pb2.RobotResponse(success=True, message="Keep-alive")
        outgoing_queue.put(robot_pb2.RobotMessage(response=keep_alive))


def run_grpc_client(robot_turtle: RobotTurtle):
    threading.Thread(target=keep_alive_worker, daemon=True).start()

    with grpc.insecure_channel(SERVER_ADDRESS) as channel:
        stub = robot_pb2_grpc.RobotServiceStub(channel)
        responses = stub.Connect(request_generator())
        for msg in responses:
            if msg.HasField("command"):
                cmd = msg.command
                if cmd.type == common_pb2.CommandType.START:
                    robot_turtle.get_logger().info("Received START command")
                    try:
                        robot_turtle.start_circle()
                    except Exception as e:
                        print(e)
                    response = common_pb2.RobotResponse(
                        success=True, message="Robot started"
                    )
                elif cmd.type == common_pb2.CommandType.STOP:
                    robot_turtle.get_logger().info("Received STOP command")
                    try:
                        robot_turtle.stop_circle()
                    except Exception as e:
                        print(e)
                    response = common_pb2.RobotResponse(
                        success=True, message="Robot stopped"
                    )
                elif cmd.type == common_pb2.CommandType.SET_POSITION:
                    robot_turtle.get_logger().info(
                        f"Received SET_POSITION command: x={cmd.x}, y={cmd.y}"
                    )
                    try:
                        robot_turtle.set_position(cmd.x, cmd.y)
                    except Exception as e:
                        print(e)
                    response = common_pb2.RobotResponse(
                        success=True, message="Position set", x=cmd.x, y=cmd.y
                    )
                elif cmd.type == common_pb2.CommandType.GET_POSITION:
                    robot_turtle.get_logger().info("Received GET_POSITION command")
                    current_x, current_y = robot_turtle.get_position()
                    response = common_pb2.RobotResponse(
                        success=True,
                        message="Current position",
                        x=current_x,
                        y=current_y,
                    )
                else:
                    print("Received unknown command")
                    response = common_pb2.RobotResponse(
                        success=False, message="Unknown command"
                    )

                print("Sending response:", response)
                outgoing_queue.put(robot_pb2.RobotMessage(response=response))


def main():
    subprocess.Popen(["ros2", "run", "turtlesim", "turtlesim_node"])
    time.sleep(2)

    rclpy.init()
    robot_turtle = RobotTurtle()

    grpc_thread = threading.Thread(target=run_grpc_client, args=(robot_turtle,))
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
