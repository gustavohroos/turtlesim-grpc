.PHONY: setup generate-proto clean

setup:
	go install google.golang.org/protobuf/cmd/protoc-gen-go@latest
	go install google.golang.org/grpc/cmd/protoc-gen-go-grpc@latest

generate-proto:
	protoc -I=. --go_out=. --go-grpc_out=. ./server/protobuf/*.proto

vendor:
	go mod vendor

clean:
	rm -f server/proto/robot.pb.go server/proto/robot_grpc.pb.go robot/robot_pb2.py robot/robot_pb2_grpc.py
