# Practica final. Simulación en ROS2 y Gazebo
***
**Autor**: *Sergio Cobos Blanco*

**Asignatura**: *Modelado y Simulación de Robots*

**Grado**: *Grado En Ingenieria Robótica Software*

**Universidad**: *Universidad Rey Juan Carlos*
***

### Objetivos de la práctica
- Configurar el modelo de un rover (cuyo nombre es *speedy*), modelado en la practica anterior, para que pueda ser utilizado y simulado por ROS2 y Gazebo:
    * Adaptar y dividir el modelo del formato .urdf a ficheros .urdf.xacro
    * Utilización de *ros2_control* y un [fichero de configuración de controladores](speedy_description/config/speedy_controllers.yaml) para la control de las articulaciones móviles en simulación.
    * Configuración de diferentes propiedades de las ruedas para su correcta simulación.

- Simulación en Gazebo del rover:
    * Configurar los ficheros de los [sensores](speedy_description/urdf/sensors) para que puedan ser simulados en Gazebo. Así como la [configuración](speedy_description/config/speedy_bridge.yaml) el nodo bridge de Gazebo.

    * Utilizar el launcher [robot_gazebo](speedy_description/launch/robot_controllers.launch.py) para lanzar:
        - Gazebo
        - rviz2
        - [Modelo del robot](speedy_moveit_config/config/rsp.launch.py)
        - Gazebo bridge para el sensor IMU
        - Gazebo image bridges para las imágenes de las cámaras
        - *twist_stamper* para enviar los comandos de /cmd a simulación

    * Utilizar el launcher [robot_controllers](speedy_description/launch/robot_controllers.launch.py) para lanzar los controladores de las ruedas, el brazo y el gancho del brazo.

- Control del brazo con [Moveit](https://moveit.picknik.ai/main/index.html)

