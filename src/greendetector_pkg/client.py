import grpc
import cx100_pb2
import cx100_pb2_grpc

def run_client():
    """
    @brief Función para ejecutar el cliente gRPC.

    Esta función crea un canal gRPC, crea un cliente (stub) y realiza una llamada al método remoto.

    """
    # Crear un canal gRPC
    channel = grpc.insecure_channel('localhost:50051')
    
    request = cx100_pb2.Empty()
    # Crear un cliente (stub)
    stub = cx100_pb2_grpc.GreenObjectServiceStub(channel)
    # Crear un mensaje de solicitud
    # Realizar la llamada al método remoto
    response = stub.PublishObjectCoordinates(request)

    # Procesar la respuesta
    print("Received coordinate: x={}, y={}".format(response.x, response.y))

if __name__ == '__main__':
    while True:
        run_client()