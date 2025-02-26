# Turtlesim gRPC

## Requests possíveis
1. ListRobots
   - Descrição: Retorna a lista de IDs de todos os robôs conectados.
   - Comando:
     grpcurl -plaintext -d '{}' localhost:50051 pb.CommandService/ListRobots

2. Start
   - Descrição: Solicita que um robô inicie sua operação (por exemplo, iniciar movimento circular).
   - Comando:
     grpcurl -plaintext -d '{"robot_id": "robot1"}' localhost:50051 pb.CommandService/Start

3. Stop
   - Descrição: Solicita que um robô pare sua operação.
   - Comando:
     grpcurl -plaintext -d '{"robot_id": "robot1"}' localhost:50051 pb.CommandService/Stop

4. SetPosition
   - Descrição: Atualiza a posição central do movimento circular do robô.
   - Comando:
     grpcurl -plaintext -d '{"robot_id": "robot1", "x": 5.0, "y": 3.0}' localhost:50051 pb.CommandService/SetPosition

5. GetPosition
   - Descrição: Solicita a posição atual do robô.
   - Comando:
     grpcurl -plaintext -d '{"robot_id": "robot1"}' localhost:50051 pb.CommandService/GetPosition
