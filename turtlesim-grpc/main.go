package main

import (
	"context"
	"fmt"
	"log"
	"net"
	"sync"
	"time"

	"google.golang.org/grpc"
	"google.golang.org/grpc/reflection"
	"google.golang.org/protobuf/types/known/emptypb"

	pb "turtlesim-grpc/server/models"
)

type RobotConnection struct {
	robotID    string
	stream     pb.RobotService_ConnectServer
	responseCh chan *pb.RobotMessage
	mu         sync.Mutex
}

var (
	robotConnections   = make(map[string]*RobotConnection)
	robotConnectionsMu sync.RWMutex
)

type robotServiceServer struct {
	pb.UnimplementedRobotServiceServer
}

func (s *robotServiceServer) Connect(stream pb.RobotService_ConnectServer) error {
	msg, err := stream.Recv()
	if err != nil {
		return err
	}
	reg := msg.GetRegistration()
	if reg == nil {
		return fmt.Errorf("primeira mensagem deve ser de registro")
	}
	robotID := reg.RobotId
	conn := &RobotConnection{
		robotID:    robotID,
		stream:     stream,
		responseCh: make(chan *pb.RobotMessage),
	}

	robotConnectionsMu.Lock()
	robotConnections[robotID] = conn
	robotConnectionsMu.Unlock()

	log.Printf("Robot %s conectado", robotID)

	go func() {
		for {
			msg, err := stream.Recv()
			if err != nil {
				log.Printf("Robot %s desconectado: %v", robotID, err)
				break
			}
			if msg.GetResponse() != nil {
				conn.responseCh <- msg
			}
		}
		robotConnectionsMu.Lock()
		delete(robotConnections, robotID)
		robotConnectionsMu.Unlock()
	}()

	<-make(chan struct{})
	return nil
}

func sendCommandToRobot(robotID string, cmd *pb.RobotCommand) (*pb.RobotResponse, error) {
	robotConnectionsMu.RLock()
	conn, exists := robotConnections[robotID]
	robotConnectionsMu.RUnlock()
	if !exists {
		return nil, fmt.Errorf("robot %s nao conectado", robotID)
	}

	msg := &pb.RobotMessage{
		Payload: &pb.RobotMessage_Command{
			Command: cmd,
		},
	}

	conn.mu.Lock()
	err := conn.stream.Send(msg)
	conn.mu.Unlock()
	if err != nil {
		return nil, err
	}

	select {
	case respMsg := <-conn.responseCh:
		return respMsg.GetResponse(), nil
	case <-time.After(5 * time.Second):
		return nil, fmt.Errorf("timeout aguardando resposta do robot %s", robotID)
	}
}

type commandServiceServer struct {
	pb.UnimplementedCommandServiceServer
}

func (s *commandServiceServer) Start(ctx context.Context, req *pb.RobotCommandRequest) (*pb.RobotResponse, error) {
	cmd := &pb.RobotCommand{
		Type: pb.CommandType_START,
	}
	return sendCommandToRobot(req.RobotId, cmd)
}

func (s *commandServiceServer) Stop(ctx context.Context, req *pb.RobotCommandRequest) (*pb.RobotResponse, error) {
	cmd := &pb.RobotCommand{
		Type: pb.CommandType_STOP,
	}
	return sendCommandToRobot(req.RobotId, cmd)
}

func (s *commandServiceServer) SetPosition(ctx context.Context, req *pb.PositionRequest) (*pb.RobotResponse, error) {
	cmd := &pb.RobotCommand{
		Type: pb.CommandType_SET_POSITION,
		X:    req.X,
		Y:    req.Y,
	}
	return sendCommandToRobot(req.RobotId, cmd)
}

func (s *commandServiceServer) GetPosition(ctx context.Context, req *pb.RobotCommandRequest) (*pb.RobotResponse, error) {
	cmd := &pb.RobotCommand{
		Type: pb.CommandType_GET_POSITION,
	}
	return sendCommandToRobot(req.RobotId, cmd)
}

func (s *commandServiceServer) ListRobots(ctx context.Context, _ *emptypb.Empty) (*pb.RobotListResponse, error) {
	robotConnectionsMu.RLock()
	defer robotConnectionsMu.RUnlock()

	robotIDs := make([]string, 0, len(robotConnections))
	for robotID := range robotConnections {
		robotIDs = append(robotIDs, robotID)
	}

	return &pb.RobotListResponse{RobotIds: robotIDs}, nil
}

func main() {
	lis, err := net.Listen("tcp", ":50051")
	if err != nil {
		log.Fatalf("Falha ao escutar: %v", err)
	}
	grpcServer := grpc.NewServer()

	pb.RegisterRobotServiceServer(grpcServer, &robotServiceServer{})
	pb.RegisterCommandServiceServer(grpcServer, &commandServiceServer{})

	reflection.Register(grpcServer)

	log.Printf("Servidor rodando em %v", lis.Addr())
	if err := grpcServer.Serve(lis); err != nil {
		log.Fatalf("Falha ao servir: %v", err)
	}
}
