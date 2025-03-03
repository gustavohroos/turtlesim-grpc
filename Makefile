generate-proto:
	protoc -I=. --go_out=. --go-grpc_out=. ./protobuf/*.proto
	python -m grpc_tools.protoc -I=. --python_out=./robot/models --grpc_python_out=./robot/models ./protobuf/*.proto