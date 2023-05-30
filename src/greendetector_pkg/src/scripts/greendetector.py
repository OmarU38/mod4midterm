#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import ctypes
from std_msgs.msg import Int32MultiArray

class publ: 
    def __init__(self):
        rospy.init_node('greendetector_node', anonymous=True)
        self.result_pub = rospy.Publisher('/green_object_coordinates', Int32MultiArray, queue_size=1)
        
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        """
        @brief Función de devolución de llamada para el tópico de imagen.

        Esta función se ejecuta cada vez que se recibe un mensaje de imagen.
        Convierte el mensaje de imagen a una imagen de OpenCV y realiza el procesamiento de detección de objetos verdes.
        Luego, encuentra el objeto verde más grande, calcula sus coordenadas y publica los resultados.

        @param msg Mensaje de imagen recibido.
        """
        # Convertir el mensaje de imagen a una imagen OpenCV
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convertir la imagen a formato HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Definir los rangos de color para objetos verdes en HSV
        lower_green = (40, 50, 50)
        upper_green = (80, 255, 255)

        # Aplicar un filtro de color para detectar objetos verdes
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Encontrar contornos de los objetos verdes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Dibujar los contornos en la imagen original
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

        # Mostrar la imagen con los contornos en una ventana separada
        cv2.imshow("Contornos", frame)
        cv2.waitKey(1)

        # Procesar los contornos y encontrar el objeto verde más grande
        largest_area = 0
        largest_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
                largest_contour = contour

        # Calcular las coordenadas X y Y del objeto verde
        lib = ctypes.CDLL('/home/omaru/Desktop/Robotics/ROS/ws_midterm/src/greendetector_pkg/libcoordinate_multiplier.so')

        lib.multiplyCoordinates.argtypes = [ctypes.c_int]
        lib.multiplyCoordinates.restype = ctypes.c_int

        cx100 = 0
        cy100 = 0

        if largest_contour is not None:
            M = cv2.moments(largest_contour)
            centroid_x = int(M['m10'] / M['m00'])
            centroid_y = int(M['m01'] / M['m00'])
            cx100 = lib.multiplyCoordinates(ctypes.c_int(centroid_x)) 
            cy100 = lib.multiplyCoordinates(ctypes.c_int(centroid_y))
            rospy.loginfo("Green object found at coordinates X: " + str(cx100) + ", Y: " + str(cy100) + "\ln")

        # Crear un nuevo mensaje Image con los resultados y establecer el timestamp
        result_msg = Int32MultiArray()
        result_msg.data = [cx100, cy100, rospy.Time.now().secs, rospy.Time.now().nsecs]  

        # Verificar si result_pub está inicializado antes de publicar el mensaje
        self.result_pub.publish(result_msg)

if __name__ == '__main__':
    my_publ = publ()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass