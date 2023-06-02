# Midterm de Módulo de Interfaces
Creado por A00827095 Omar Enrique Gonzalez Uresti

El siguiente repositorio contiene un proyecto de interfaces desarrollado para cumplir con requerimientos de una actividad; la cual consiste en mantener una imagen de una cámara y poder detectar objetos de color verdes. Utilizando una librería para multiplicar x 100 las coordenadas y herramientas como ROS, se pudo completar el proyecto que cumpliera con compartir esas coordenadas a través de gRPC. Para correr el proyecto es necesario descargar o clonar el proyecto entero, y correr mediante ROS los archivos greendetector.py (src/greendetector_pkg/src/scripts) y camera_node.py. Así como correr mediante python el wrapper.py (src/greendetector_pkg); para correr el cliente, se debe correr de forma manual el App.java (demo/src/main/java/com/example)

[Video del funcionamiento del proyecto 1-6](https://drive.google.com/file/d/18alpSspqbAz9N9cohV8vlG9BrF_xSmMz/view?usp=sharing)

## Diagrama de DFD
![DFD](https://github.com/OmarU38/mod4midterm/assets/65744355/b599d05b-d02c-44ec-9767-06a6d395f653)

## Sobre la documentación en Doxygen
Doxygen nos permite documentar de forma efectiva y estandarizada documentos importantes de proyectos como el presentado, sin embargo, es importante reconocer que no siempre es adecuado el realizar este tipo de documentación ya que hay archivos que siguen una construcción específica como los archivos .proto o los generados a través del mismo; los cuales se usaron mucho durante el proyecto; lo cual podria generar problemas de compatibilidad y accesibilidad para los demas elementos del proyecto.

# Extras
## Acerca de CORS
CORS (Cross-Origin Resource Sharing) es una política de seguridad usada en navegadores web para proteger a los usuarios contra solicitudes maliciosas entre diferentes dominios.

Cuando un navegador realiza una solicitud HTTP desde una página web a un origen diferente al de la página actual, se considera una solicitud de origen cruzado. El origen se define por el esquema, el dominio y el puerto.

La política de CORS impone restricciones en las solicitudes de origen cruzado para evitar posibles ataques, como el robo de datos o la ejecución de acciones no deseadas en nombre del usuario. Cuando una aplicación web intenta realizar una solicitud de origen cruzado, el navegador realiza una "preflight request" utilizando el método HTTP OPTIONS. Esta solicitud de pre-vuelo incluye encabezados especiales, como "Origin" y "Access-Control-Request-Method", para determinar si el servidor permite la solicitud real.

El servidor debe responder a la solicitud de pre-vuelo con encabezados de respuesta adecuados para permitir o denegar la solicitud real. Estos encabezados incluyen "Access-Control-Allow-Origin", que especifica los dominios permitidos para realizar solicitudes, y "Access-Control-Allow-Methods", que define los métodos HTTP permitidos en la solicitud real. Si la respuesta del servidor permite la solicitud de origen cruzado, el navegador permitirá que la aplicación web acceda a los recursos solicitados y procese la respuesta. De lo contrario, el navegador bloqueará la solicitud y la aplicación web no podrá acceder a los recursos.

Una manera de implementarlo en el proyecto es durante la aplicación despues de Golang usando Postman, a través de esto podríamos proteger nuestro proyecto y datos para poder realizar una aplicación real mucho más segura.

CORS afecta a una aplicación web al imponer restricciones en las solicitudes de origen cruzado realizadas desde el navegador. Esto se hace para proteger la seguridad y privacidad del usuario, evitando posibles ataques maliciosos. Para permitir solicitudes de origen cruzado, el servidor debe configurar adecuadamente los encabezados de respuesta para cumplir con la política de CORS establecida por los navegadores.
