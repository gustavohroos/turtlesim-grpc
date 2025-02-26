FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-turtlesim \
    ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install grpcio grpcio-tools

WORKDIR /app
COPY robot.proto .
COPY robot.py .

RUN python3 -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. robot.proto

CMD ["python3", "robot.py"]
