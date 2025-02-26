FROM golang:latest as builder

WORKDIR /app
ENV CGO_ENABLED=0

COPY go.mod go.sum ./
COPY main.go ./
COPY server/ ./server/

RUN go mod download

RUN go build -o app .

FROM alpine:latest
WORKDIR /app
COPY --from=builder /app/app .
EXPOSE 50051
CMD ["./app"]
