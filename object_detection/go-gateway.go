package main

import (
	"context"
	"log"
	"net"

	"github.com/OmarU38/mod4midterm/object_detection" // Reemplaza con la ruta correcta del paquete generado
	"google.golang.org/grpc"
)

// Implementa la interfaz del servicio GreenObjectService
type greenObjectServer struct{}

// Implementa el método PublishObjectCoordinates del servicio GreenObjectService
func (s *greenObjectServer) PublishObjectCoordinates(ctx context.Context, empty *object_detection.Empty) (*object_detection.ObjectCoordinates, error) {
	// Lógica para obtener las coordenadas del objeto y crear el mensaje ObjectCoordinates
	// Reemplaza con tu propia implementación

	coordinates := &object_detection.ObjectCoordinates{
		X:         0.0, 
		Y:         0.0,
		Timestamp: 0.0, 
	}

	return coordinates, nil
}

func main() {
	// Inicializa el servidor gRPC
	server := grpc.NewServer()

	// Registra el servicio GreenObjectService en el servidor gRPC
	object_detection.RegisterGreenObjectServiceServer(server, &greenObjectServer{})

	// Escucha en el puerto 50051
	listener, err := net.Listen("tcp", ":50051")
	conn, err := grpc.Dial(":50051", grpc.WithInsecure())

	if err != nil {
		log.Fatalf("Failed to listen: %v", err)
	}

	// Inicia el servidor gRPC
	if err := server.Serve(listener); err != nil {
		log.Fatalf("Failed to serve: %v", err)
	}
}