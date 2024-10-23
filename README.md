# English: Parte de LGV, Computer Vision system (CV):
## IEEE citation:
L. Gómez, "*NAVEGACIÓN BASADA EN SEGUIMIENTO DE LÍNEAS MEDIANTE VISIÓN ARTIFICIAL PARA EL ROBOT ROBOGAIT SPORT*", Madrid, Spain, Sep. 2024. Available: [https://oa.upm.es/84445/](https://oa.upm.es/84445/) or [https://kbs-lucas.neocities.org/TFG_Gomez_Velayos_Lucas.pdf](https://kbs-lucas.neocities.org/TFG_Gomez_Velayos_Lucas.pdf)

Just this repository:

L. Gómez, “Trabajo-Fin-de-Grado,” *GitHub*, Accessed Sep. 09, 2024. [Online]. Available: [https://github.com/Pigamer37/Trabajo-Fin-de-Grado](https://github.com/Pigamer37/Trabajo-Fin-de-Grado)

## File index and requirements explanation

### Notes:
ROS nodes were developed in the docker container provided by ROS for Raspberry Pi OS, and the necessary packages to read integer vector type mesages were also tested on a Jetson Orin, which OS is an Ubuntu variant. Is not guaranteed that these will work on other OS's, but because of the portable nature of ROS 2, Linux and C++, they should. All external code used permits its distribution, personal and commercial use. Preliminar code to ROS implementations should be portable because it only depends on OpenCV and C++, which are multiplatform. libcamera dependent code will probably just work on UNIX based systems, it has been developed and tested on these only.

__CMake__ was used to compile C++ code. The CMakeLists.txt files are provided in the relevant folder.

### Requisites:
In this chapter we include the necessary libraries or dependencies for each part of the codebase, and the ways you can install them in Ubuntu whenever possible. Each script will have their own dependencies marked later on.

- OpenCV 👁️: `sudo apt install libopencv-dev python3-opencv` (to compile and execute respectively, necessary whenever images are involved).
- libcamera 📷: `sudo apt install libcamera` (to use Raspberry's Cam Module, otherwise, like when using a USB webcam, OpenCV's VideoCapture will suffice).
- ROS 2 🤖: depends on the operating system (for any ros2 node).
- spline.h: Included on the repository (for line synthesis/aproximmation functionallity).

### Software files/folders/pieces explanation (LGV):
- OpenCV-test: preliminar test of the library in Visual Studio, Windows. 👁️
- ROS packages 🤖:
  - vec_package: package that hosts the Vector message, all others that use this type of message depend on it.
  - my_package: basic custom ROS 2 message publisher, Vector type (`std::vector<int>` in C++).
  - vec_sub: basic custom ROS 2 message subscriber/listener, Vector type (`std::vector<int>` in C++).
  - Cam_package 👁️: TrackApprox algorythm in the ROS enviroment, capable of publishing internal data in a topic. Also contains ROSPublish.hpp, useful to have a first basic class that makes it possible to publish relevant messages from the algorythm in ROS.

- Aux_OpenCV.cpp(and .hpp) 👁️: antiquated try of a curtom library to try and ease use of OpenCV. Substituted later by OCV_Funcs.cpp(_*.hpp).
- OCV_Funcs.cpp(*.hpp) 👁️: library with a variety of functions to implement algorithms related to OpenCV, making it compatible with spline.h, extracting data, creating the histogram objects described in the article, intelligent line following with the `laneLogic` class. 
- DetectTrack.cpp 👁️: Old version of TrackApprox.cpp.
- TrackApprox.cpp 👁️: Implementation of the image and video algorithms. 
- RaspiTrack.cpp 📷👁️: TrackApprox implementation with the Raspberry cam, which means a structure change.

## How to lauch the code/ Docker use tutorial:
There's a permanent docker container already on the Rasspberry, as mentioned in the article, in which code can be compiled, stored and developed. To enter it via cli session, this command must be used:<br>
`sudo docker start -a -i 2302d08e8bd2`

To copy the contents of the ros_ws folder (the relevant files for ROS 2 developement) to our own filesystem:<br>
`sudo docker cp 2302d08e8bd2:/root/ros2_ws "/home/ROBOGait/Documents/Ros2 docker"`

To be able to communicate with ROS 2 and the Raspberry's screen, a temporary container must be launched with a similar instruction to the following:<br>
`sudo docker run -it --rm --network host --name rostmp -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ros:humble-ros-core`

`--network host` : makes the container's ports shared with the Rasspberry's.<br>
`-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw` : makes it so the container can use the Raspberry's display.<br>
__Note__: it is necessary to launch the following command **on the Raspberry** too : `xhost +`<br>
`-name rostmp` : just makes the container's identification easier. If ommited, Docker will assign a random name to our container.<br>

To copy the contents of the ros_ws folder from the permanent container wich we have on /home/ROBOGait/Documents/Ros2 docker (information can't be copied between containers directly) to the temporary container:<br>
`sudo docker cp "/home/ROBOGait/Documents/Ros2 docker/ros2_ws" rostmp:/root`

If we don't execute `--network host`, the apt-get tool will work. All necessary packages can be installed with:<br>
`apt-get update`<br>
`apt-get install libopencv-dev`<br>
`apt-get install python3-opencv`<br>
`sudo apt install python3-colcon-common-extensions`<br>
`apt-get -y install \
    firefox \

# Español: Parte de LGV, sistema de Visión Artificial (CV):
## Citación IEEE:
L. Gómez, "*NAVEGACIÓN BASADA EN SEGUIMIENTO DE LÍNEAS MEDIANTE VISIÓN ARTIFICIAL PARA EL ROBOT ROBOGAIT SPORT*", Madrid, España, Sep. 2024. Available: [https://oa.upm.es/84445/](https://oa.upm.es/84445/) or [https://kbs-lucas.neocities.org/TFG_Gomez_Velayos_Lucas.pdf](https://kbs-lucas.neocities.org/TFG_Gomez_Velayos_Lucas.pdf)

Solo de este repositorio:

L. Gómez, “Trabajo-Fin-de-Grado,” *GitHub*, Accessed Sep. 09, 2024. [Online]. Available: [https://github.com/Pigamer37/Trabajo-Fin-de-Grado](https://github.com/Pigamer37/Trabajo-Fin-de-Grado)

## Índice de archivos y explicación de requerimientos

### Notas:
Los nodos de ROS se desarrollaron en el entorno de docker provisto por ROS para Raspberry PI OS, y se comprobó el funcionamiento de los paquetes necesarios para leer mensajes de tipo vector de enteros también en la Jetson Orin, cuyo SO es una versión de Ubuntu. No se puede garantizar que estos funcionen en otros SO, aunque debido a la naturaleza de ROS 2, Linux y el propio C++, deberían ser portables. Todo el código externo usado permite su reproducción y uso libre personal y comercial. El código externo o preliminar a ROS también debería ser portable al depender en su mayoría de OpenCV y C++, que son multiplataforma. El código que depende de libcamera se sospecha que sólo funcionará en sistemas UNIX, se ha desarrollado específicamente y sólo se ha testeado en estos.

Se usó __CMake__ para compilar código C++. Los archivos CMakeLists.txt se pueden encontrar en la carpeta del mismo nombre.

### Requisitos:
En este apartado se incluyen las librerías o dependencias necesarias para cada parte del código, y las formas de instalación en Ubuntu cuando sea posible. Además se marcará más tarde cada pieza de software con sus dependencias.

- OpenCV 👁️: sudo apt install libopencv-dev python3-opencv (para compilar y ejecutar respectivamente, necesarios en cualquier app que use imágenes).
- libcamera 📷: sudo apt install libcamera (para usar la cámara de la Raspberry, en otros casos, como si se usa una webcam USB, VideoCapture de OpenCV funcionará).
- ROS 2 🤖: depende del sistema operativo (para cualquier nodo de ros2).
- spline.h: Incluída en el repositorio. (para funcionalidad de síntesis/aproximación de líneas).

### Explicación de los archivos/carpetas/piezas de software (LGV):
- OpenCV-test: test preliminar de uso de la librería en Visual Studio, en Windows. 👁️
- Paquetes de ROS 🤖:
  - vec_package: paquete que alberga el mensaje propio Vector, de este paquete dependen todos los que usan este tipo de mensaje.
  - my_package: publisher básico de mensajes de ROS propios, de tipo Vector (`std::vector<int>` en C++).
  - vec_sub: subscriber/listener básico de mensajes de ROS propios, de tipo Vector (`std::vector<int>` en C++).
  - Cam_package 👁️: algoritmo TrackApprox en entorno ROS, capaz de publicar datos internos en un topic. Contiene también ROSPublish.hpp, útil para tener una primera clase básica que permite publicar mensajes relevantes del algoritmo en ROS.

- Aux_OpenCV.cpp(*.hpp) 👁️: intento anticuado de librería custom para facilitar el uso de OpenCV. Sustituida después por OCV_Funcs.cpp(*.hpp).
- OCV_Funcs.cpp(*.hpp) 👁️: librería con variedad de funciones para implementar algoritmos relacionados con OpenCV, compatibilización con spline.h, extracción de datos, creación de los histograma descritos en el trabajo, seguimiento inteligente de líneas con la clase `laneLogic`. 
- DetectTrack.cpp 👁️: Versión anterior de TrackApprox.cpp.
- TrackApprox.cpp 👁️: Implementación de algoritmo en imágenes y vídeos. 
- RaspiTrack.cpp 📷👁️: Implementación de TrackApprox con la cámara de la Raspberry, lo que supone un cambio de estructura.

## Cómo lanzar implementación del código/ tutorial de uso con Docker:
Existe ya en la placa, como se ha mencionado en el trabajo, un contenedor de docker permanente en el que se puede compilar, almacenar y desarrollar código. Para entrar en una sesión de cli en este, se debe lanzar este comando:<br>
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

Si no se hace `--network host`, la herramienta apt-get funcionará. Todos los paquetes necesarios se pueden instalar con:<br>
`apt-get update`<br>
`apt-get install libopencv-dev`<br>
`apt-get install python3-opencv`<br>
`sudo apt install python3-colcon-common-extensions`<br>
`apt-get -y install \
    firefox \
    libcanberra-gtk-module \
    libcanberra-gtk3-module`

Respectivamente, cada línea: actualiza, instala OpenCV para desarrollo, instala OpenCV para ejecución, instala las herramientas de colcon para compilar en ROS, instala las dependencias para que OpenCV pueda presentarse en pantallas.
