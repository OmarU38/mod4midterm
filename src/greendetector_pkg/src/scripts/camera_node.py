#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import ctypes

def image_callback(msg):
    """
    @brief Función de devolución de llamada para el tópico de imagen.

    Esta función se ejecuta cada vez que se recibe un mensaje de imagen.
    Convierte el mensaje de imagen a una imagen de OpenCV y realiza el procesamiento de detección de objetos verdes.
    Luego, publica las coordenadas del objeto detectado en el tópico '/green_object_coordinates'.

    @param msg Mensaje de imagen recibido.
    """
    # Convertir el mensaje de imagen a una imagen OpenCV
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Procesar la imagen para detectar objetos verdes
    # Aquí debes implementar tu lógica de detección de objetos verdes
    # y calcular las coordenadas X y Y del objeto detectado

    # Publicar las coordenadas del objeto detectado
    pub = rospy.Publisher('/green_object_coordinates', Point, queue_size=1)
    point = Point()
    point.x = x
    point.y = y 
    pub.publish(point)  

def camera_node():
    """
    @brief Nodo principal para capturar imágenes de la cámara y publicarlas.

    Este nodo crea un objeto CvBridge para convertir imágenes entre OpenCV y ROS,
    crea un objeto Publisher para publicar imágenes en el tópico '/camera/image_raw',
    y configura la frecuencia de publicación de imágenes.
    Luego, captura imágenes de la cámara, las convierte y las publica en el tópico '/camera/image_raw'.
    """
    rospy.init_node('camera_node', anonymous=True)

    # Crear un objeto CvBridge
    bridge = CvBridge()

    # Crear un objeto Publisher para publicar imágenes en el tópico '/camera/image_raw'
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

    # Crear un objeto VideoCapture para leer imágenes de la cámara
    camera = cv2.VideoCapture(0)

    # Configurar la frecuencia de publicación de imágenes
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Leer una imagen de la cámara
        ret, frame = camera.read()

        if ret:
            # Convertir la imagen OpenCV a un mensaje de imagen de ROS
            img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")

            # Publicar la imagen en el tópico '/camera/image_raw'
            image_pub.publish(img_msg)

        # Esperar para mantener la frecuencia de publicación
        rate.sleep()

    # Liberar la cámara y cerrar la ventana OpenCV al finalizar
    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass