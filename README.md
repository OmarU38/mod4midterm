# Midterm de Módulo de Interfaces
Creado por A00827095 Omar Enrique Gonzalez Uresti

El siguiente repositorio contiene un proyecto de interfaces desarrollado para cumplir con requerimientos de una actividad; la cual consiste en mantener una imagen de una cámara y poder detectar objetos de color verdes. Utilizando una librería para multiplicar x 100 las coordenadas y herramientas como ROS, se pudo completar el proyecto que cumpliera con compartir esas coordenadas a través de gRPC. Para correr el proyecto es necesario descargar o clonar el proyecto entero, y correr mediante ROS los archivos greendetector.py (src/greendetector_pkg/src/scripts) y camera_node.py. Así como correr mediante python el wrapper.py (src/greendetector_pkg); para correr el cliente, se debe correr de forma manual el App.java (demo/src/main/java/com/example)

[Video del funcionamiento del proyecto 1-6](https://drive.google.com/file/d/18alpSspqbAz9N9cohV8vlG9BrF_xSmMz/view?usp=sharing)

## Diagrama de DFD
![BPMN 2 0 (1)](https://github.com/OmarU38/mod4midterm/assets/65744355/d17b8ac0-8237-47b9-b100-046e4b8ca1ab)

## Sobre la documentación en Doxygen
Doxygen nos permite documentar de forma efectiva y estandarizada documentos importantes de proyectos como el presentado, sin embargo, es importante reconocer que no siempre es adecuado el realizar este tipo de documentación ya que hay archivos que siguen una construcción específica como los archivos .proto o los generados a través del mismo; los cuales se usaron mucho durante el proyecto; lo cual podria generar problemas de compatibilidad y accesibilidad para los demas elementos del proyecto.
