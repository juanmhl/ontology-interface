# Motor de inferencia para el proyecto RACE

La feature inferenceengine encapsula el motor de inferencia para el proyecto RACE de UMA, UMH y UVA, que implementa una Ontología desarrollada en UMA para automatizar la maniobra de sutura laparoscópica, total o parcialmente, mediante un sistema robotizado.

La estructura de archivos es:
- `ros_package/`: paquete Python que encapsula en un servicio el motor de inferencia con su ontología.
- `primitivegenerator/`: paquete Python que encapsula la generación de primitivas, paso de acciones cualitativas del motor de inferencia a acciones cuantitativas de los robots. NO TERMINADO
- `umatransforms/`: paquete de python con funciones basadas en numpy para transformadas homogéneas típicas (basado en las funciones de MATLAB de ISA-UMA). NO UTILIZADO
- `ros_package/ontology.owl`: fichero con la descripción de la ontología desarrollada en UMA por Álvaro Galán Cuenca, Marta Fernández Naranjo y Alfredo Burrieza.
- `Dockerfile`: descripción de la imagen utilizada para el desarrollo de la app. Encontrándose en el directorio raíz del repositorio ejecutar `docker build -t ros-noetic-race .` para construir la imagen usada en el desarrollo del motor de inferencia.
- `launch_container.sh`: dar permisos de ejecución con `chmod +x launch_container.sh` y ejecutar con `./launch_container.sh` para correr el contenedor con el entorno de desarrollo ROS. El entorno mapea `ros_package/` en el host a `/catkin_ws/src/inferenceengine/` en el contenedor.
- `requirements.txt`: archivo obtenido con `pip list --format=freeze > requirements.txt` con todas las versiones de los paquetes de Python usados durante el desarrollo. Se instalan automáticamente en el contenedor.
- `ros_package/scripts`: archivos de testeo para probar la feature inferenceengine. Solo son para pruebas. El cuaderno jupyter no se podrá ejecutar en el Docker pero tiene pruebas que se hicieron anteriormente.
- `ros_package/src/inferenceengine`: feature de motor de inferencia como servicio que encapsula la ontología y otras implementaciones posibles (que no funcionan).


### Notas sobre Docker:

Compilar la imagen con:

`docker build -t ros-noetic-race .`

Correr el contenedor con:

`docker run -it --rm --user root -v $PWD/ros_package:/catkin_ws/src/inferenceengine:rw --network=host --ipc=host ros-noetic-race`

O más fácil, con `./launch_container.sh`

Luego en la terminal compilar el paquete con `catkin_make` en el directorio `/catkin_ws/`. Después de eso utilizar el nodo como siempre:

`rosrun inferenceengine inferenceengine_node.py`

Tener en cuenta que para sacar otra terminal del mismo contenedor se hace con:

`docker exec -it <container_tag> bash`

Cuando defino lo del entrypoint en `Dockerfile` hace que al final de docker run pueda poner el comando que yo quiera, como por ejemplo rosrun turtlesim turtlesim_node. Eso lo haré en condiciones cuando termine de desarrollar para que al correr el contenedor se ejecute el nodo de golpe. O lo mismo hago un docker-compose.

Para usar interfaz gráfica, primero en el host ejecutar `xhost +`. Luego al run añadir `-v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY`. De todas formas esto no me ha estado funcionando.