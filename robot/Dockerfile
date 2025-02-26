FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-turtlesim \
    ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install grpcio grpcio-tools

WORKDIR /app
COPY robot.proto robot_server.py entrypoint.sh ./
RUN chmod +x entrypoint.sh

RUN python3 -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. robot.proto

CMD ["./entrypoint.sh"]
