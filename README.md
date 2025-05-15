# Practica final. Simulación en ROS2 y Gazebo
***
**Autor**: *Sergio Cobos Blanco*

**Asignatura**: *Modelado y Simulación de Robots*

**Grado**: *Grado En Ingenieria Robótica Software*

**Universidad**: *Universidad Rey Juan Carlos*
***

## Objetivos de la práctica
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

***

## Modelo del robot
El modelo del robot esta descrito en el fichero [speedy.urdf.xacro](speedy_descritpion/robots/speedy.urdf.xacro), teniendo subdivididas sus partes en los ficheros que se encuentran en los directorios de directorio [urdf](speedy_description/urdf) del paquete [speedy_description](speedy_description).

Utilizando `rviz2` podemos visualizar el modelo del robot, sin necesidad de simularlo, y sus links publicadas en el sistema de `tf` gracias al nodo `robot_description_publisher`, el cual también publica el URDF generado a partir del fichero [speedy.urdf.xacro](speedy_descritpion/robots/speedy.urdf.xacro) en el topic `/robot_description`.

Pero para poder visualizar los links unidos a joints moviles, y sus  hijos, es necesario que se publique información relevante sobre estos joints en el topic `/joint_states`. Para ello, se debe utilizar el nodo `joint_state_publisher_gui`, que no solo publicará esta información, si no que también nos brindará una interfaz para controlarlos.

<div align="center">
    <img src="media/rviz_robot_joints.gif" alt="robot_rviz_gif"/>
    <img src="media/rviz_robot_joints_cap.png" alt="robot_rviz_cap">
</div>

En el PDF [tf_tree](media/tf_tree.pdf) podemos encontrar el arbol generado con el nodo `view_frames` de `tf2_tools`.

