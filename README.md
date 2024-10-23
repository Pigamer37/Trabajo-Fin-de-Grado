# Parte de LGV, sistema de Visión Artificial (CV):
## Citación IEEE:
L. Gómez, "*NAVEGACIÓN BASADA EN SEGUIMIENTO DE LÍNEAS MEDIANTE VISIÓN ARTIFICIAL PARA EL ROBOT ROBOGAIT SPORT*", Madrid, España, Sep. 2024. Available: [https://oa.upm.es/84445/](https://oa.upm.es/84445/) or [https://kbs-lucas.neocities.org/TFG_Gomez_Velayos_Lucas.pdf](https://kbs-lucas.neocities.org/TFG_Gomez_Velayos_Lucas.pdf)

Solo de este repositorio:

L. Gómez, “Trabajo-Fin-de-Grado,” *GitHub*, Accessed Sep. 09, 2024. [Online]. Available: [https://github.com/Pigamer37/Trabajo-Fin-de-Grado](https://github.com/Pigamer37/Trabajo-Fin-de-Grado)

## Índice de archivos y explicación de requerimientos

### Notas:
Los nodos de ROS se desarrollaron en el entorno de docker provisto en [20], y se comprobó el funcionamiento de los paquetes necesarios para leer mensajes de tipo vector de enteros también en la Jetson Orin, cuyo SO es una versión de Ubuntu. No se puede garantizar que estos funcionen en otros SO, aunque debido a la naturaleza de ROS 2, Linux y el propio C++, deberían ser portables. Todo el código externo usado permite su reproducción y uso libre personal y comercial. El código externo o preliminar a ROS también debería ser portable al depender en su mayoría de OpenCV y C++, que son multiplataforma. El código que depende de libcamera se sospecha que sólo funcionará en sistemas UNIX, se ha desarrollado específicamente y sólo se ha testeado en estos.

### Requisitos:
En este apartado se incluyen las librerías o dependencias necesarias para cada parte del código, y las formas de instalación en Ubuntu cuando sea posible. Además se marcará más tarde cada pieza de software con sus dependencias.

- OpenCV 👁️: sudo apt install libopencv-dev python3-opencv (para compilar y ejecutar respectivamente, necesarios en cualquier app que use imágenes).
- libcamera 📷: sudo apt install libcamera (para usar la cámara de la Raspberry).
- ROS 2 🤖: depende del sistema operativo (para cualquier nodo de ros2).
- spline.h: Incluída en el repositorio. (para funcionalidad de síntesis/aproximación de líneas).

### Explicación de los archivos/carpetas/piezas de software (LGV):
- OpenCV-test: test preliminar de uso de la librería en Visual Studio, en Windows. 👁️
- Paquetes de ROS 🤖:
  - vec_package: paquete que alberga el mensaje propio Vector, de este paquete dependen todos los que usan este tipo de mensaje.
  - my_package: publisher básico de mensajes de ROS propios, de tipo Vector (std::vector<int> en C++).
  - vec_sub: subscriber/listener básico de mensajes de ROS propios, de tipo Vector (std::vector<int> en C++).
  - Cam_package 👁️: algoritmo TrackApprox en entorno ROS, capaz de publicar datos internos en un topic. Contiene también ROSPublish.hpp, útil para tener una primera clase básica que permite publicar mensajes relevantes del algoritmo en ROS.

- Aux_OpenCV.cpp(*.hpp) 👁️: intento anticuado de librería custom para facilitar el uso de OpenCV. Sustituida después por OCV_Funcs.cpp(*.hpp).
- OCV_Funcs.cpp(*.hpp) 👁️: librería con variedad de funciones para implementar algoritmos relacionados con OpenCV, compatibilización con spline.h, extracción de datos, creación de los histograma descritos, seguimiento inteligente de líneas con la clase laneLogic. 
- DetectTrack.cpp 👁️: Versión anterior de TrackApprox.cpp.
- TrackApprox.cpp 👁️: Implementación de algoritmo en imágenes y vídeos. 
- RaspiTrack.cpp 📷👁️: Implementación de TrackApprox con la cámara de la Raspberry, lo que supone un cambio de estructura.

## Cómo lanzar implementación del código/ tutorial de uso con Docker:
Existe ya en la placa, como se ha mencionado, un contenedor de docker permanente en el que se puede compilar, almacenar y desarrollar código. Para entrar en una sesión de cli en este, se debe lanzar este comando:<br>
`sudo docker start -a -i 2302d08e8bd2`

Para copiar los contenidos de la carpeta ros_ws (los archivos relevantes del desarrollo en ROS 2) a nuestro sistema de archivos:<br>
`sudo docker cp 2302d08e8bd2:/root/ros2_ws "/home/ROBOGait/Documents/Ros2 docker"`

Para poder disponer de comunicación con ROS 2 y de la pantalla de la Raspberry, se deberá lanzar un contenedor temporal con una instrucción similar a esta:<br>
`sudo docker run -it --rm --network host --name rostmp -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ros:humble-ros-core`

`--network host` : habilita que los puertos del contenedor se compartan con la Raspberry.<br>
`-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw` : habilita el uso por parte del contenedor de la pantalla de la Raspberry.<br>
__Nota__: es necesario **en la Raspberry** lanzar este comando también : `xhost +`<br>
`-name rostmp` : simplemente para hacer más fácil la identificación. Si se omite, Docker asignará un nombre aleatorio a nuestro contenedor.<br>

Para copiar los contenidos de la carpeta ros_ws del contenedor permanente que tenemos en /home/ROBOGait/Documents/Ros2 docker (no se puede copiar información directamente entre contenedores) al contenedor temporal:<br>
`sudo docker cp "/home/ROBOGait/Documents/Ros2 docker/ros2_ws" rostmp:/root`

Si no se hace --network host, la herramienta apt-get funcionará. Todos los paquetes necesarios se pueden instalar con:<br>
`apt-get update`<br>
`apt-get install libopencv-dev`<br>
`apt-get install python3-opencv`<br>
`sudo apt install python3-colcon-common-extensions`<br>
`apt-get -y install \
    firefox \
    libcanberra-gtk-module \
    libcanberra-gtk3-module`

Respectivamente, cada línea: actualiza, instala OpenCV para desarrollo, instala OpenCV para ejecución, instala las herramientas de colcon para compilar en ROS, instala las dependencias para que OpenCV pueda presentarse en pantallas.
