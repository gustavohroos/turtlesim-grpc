#!/bin/bash
# Inicia o turtlesim em background
ros2 run turtlesim turtlesim_node &
# Aguarda alguns segundos para garantir que o turtlesim inicie e publique seus serviços
sleep 5
# Executa o servidor gRPC que integra o ROS2
python3 robot_server.py