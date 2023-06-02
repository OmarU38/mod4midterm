package main

import (
	"context"
	"log"
	"net"

	pb "example.com/grpgw/gen/proto"

	"google.golang.org/grpc"
)

type GreenObjectServer struct {
	pb.UnimplementedGreenObjectServiceServer
}

func (s *GreenObjectServer) PublishObjectCoordinates(ctx context.Context, req *pb.Empty) (*pb.ObjectCoordinates, error) {
	// Establecer conexión gRPC como cliente
	conn, err := grpc.Dial("localhost:50051", grpc.WithInsecure())
	if err != nil {
		log.Println(err)
		return nil, err
	}
	defer conn.Close()

	// Crear cliente gRPC para el servidor externo
	coordsCommClient := pb.NewGreenObjectServiceClient(conn)

	// Solicitar los valores de PointStamped al servidor externo
	resp, err := coordsCommClient.PublishObjectCoordinates(context.Background(), &pb.Empty{})
	if err != nil {
		log.Println(err)
		return nil, err
	}

	return resp, nil
}

func main() {
	listener, err := net.Listen("tcp", ":50052")
	if err != nil {
		log.Fatalf("Failed to listen: %v", err)
	}

	grpcServer := grpc.NewServer()

	pb.RegisterGreenObjectServiceServer(grpcServer, &GreenObjectServer{})

	log.Println("Server started, listening on :50052")

	if err := grpcServer.Serve(listener); err != nil {
		log.Fatalf("Failed to serve: %v", err)
	}
}