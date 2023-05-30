#!/usr/bin/python3
import rospy
from std_msgs.msg import Int32MultiArray
import grpc
from concurrent import futures
import cx100_pb2 as cx100_pb
import cx100_pb2_grpc as cx100_pb_grpc

class GreenObjectServicer(cx100_pb_grpc.GreenObjectServiceServicer):
    """@class GreenObjectServicer
    @brief Clase que implementa el servicio gRPC GreenObjectService.

    Esta clase es responsable de manejar las solicitudes del cliente y proporcionar
    respuestas correspondientes.
    """

    def __init__(self):
        """@brief Constructor de la clase GreenObjectServicer.

        Se encarga de inicializar el servidor gRPC y agregar los servicios proporcionados.
        """
        self.x = 0
        self.y = 0
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        cx100_pb_grpc.add_GreenObjectServiceServicer_to_server(self, self.server)
        self.server.add_insecure_port('[::]:50051')

    def PublishObjectCoordinates(self, request, context):
        """@brief Método para publicar las coordenadas del objeto.

        Recibe una solicitud del cliente y devuelve las coordenadas del objeto como respuesta.

        @param request: Solicitud del cliente.
        @param context: Contexto de la solicitud.

        @return Respuesta con las coordenadas del objeto.
        """
        # Obtener las coordenadas del mensaje recibido
        # Crear el mensaje Int32MultiArray
        # Create a request message
        result = cx100_pb.ObjectCoordinates()
        result.x = self.x
        result.y = self.y
        result.timestamp = self.timestamp
        # Devolver las mismas coordenadas como respuesta
        return result

    def coord_callback(self, data:Int32MultiArray):
        """@brief Método de devolución de llamada para procesar las coordenadas del objeto.

        Recibe las coordenadas del objeto y actualiza los valores correspondientes.

        @param data: Coordenadas del objeto.
        """
        self.x = data.data[0]
        self.y = data.data[1]
        self.timestamp = data.data[3]

def serve():
    """@brief Método para iniciar el servidor gRPC y suscribirse al tópico de coordenadas del objeto."""
    rospy.init_node('cx100_server')

    # Crear un servidor gRPC
    service = GreenObjectServicer()

    # Inicializar el subscriber del tópico
    rospy.Subscriber('green_object_coordinates', Int32MultiArray, service.coord_callback)

    # Iniciar el servidor
    service.server.start()

    rospy.loginfo("Green Object Servera started")
    rospy.spin()

if __name__ == '__main__':
    serve()