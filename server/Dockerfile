FROM python:3.9-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
# Gerar os arquivos Python a partir do .proto
RUN python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. robot.proto
EXPOSE 5000
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "5000"]
