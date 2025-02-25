FROM ros:humble

# Instalar pacotes adicionais, incluindo o turtlesim
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# Instala as dependências do gRPC
RUN pip3 install grpcio grpcio-tools

WORKDIR /app
COPY robot.proto .
COPY robot_server.py .
COPY entrypoint.sh .

# Gera os arquivos a partir do .proto
RUN python3 -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. robot.proto

EXPOSE 50051
ENTRYPOINT ["/app/entrypoint.sh"]
