from fastapi import FastAPI, HTTPException
import grpc
import robot_pb2
import robot_pb2_grpc
from pydantic import BaseModel

app = FastAPI()


# Modelo para a posição
class Position(BaseModel):
    x: float
    y: float


def get_robot_stub(robot_id: str):
    # Utiliza o nome do contêiner (ex: robot1 ou robot2) para a resolução DNS no Docker Compose
    channel = grpc.insecure_channel(f"{robot_id}:50051")
    stub = robot_pb2_grpc.RobotServiceStub(channel)
    return stub


@app.post("/robots/{robot_id}/start")
async def start_robot(robot_id: str):
    stub = get_robot_stub(robot_id)
    req = robot_pb2.RobotRequest(robot_id=robot_id)
    resp = stub.Start(req)
    return {"success": resp.success, "message": resp.message}


@app.post("/robots/{robot_id}/stop")
async def stop_robot(robot_id: str):
    stub = get_robot_stub(robot_id)
    req = robot_pb2.RobotRequest(robot_id=robot_id)
    resp = stub.Stop(req)
    return {"success": resp.success, "message": resp.message}


@app.post("/robots/{robot_id}/position")
async def set_position(robot_id: str, pos: Position):
    stub = get_robot_stub(robot_id)
    req = robot_pb2.PositionRequest(robot_id=robot_id, x=pos.x, y=pos.y)
    resp = stub.SetPosition(req)
    return {"success": resp.success, "message": resp.message}


@app.get("/robots/{robot_id}/position")
async def get_position(robot_id: str):
    stub = get_robot_stub(robot_id)
    req = robot_pb2.GetPositionRequest(robot_id=robot_id)
    resp = stub.GetPosition(req)
    return {"x": resp.x, "y": resp.y}
