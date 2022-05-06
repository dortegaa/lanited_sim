# lanited_ros
Paquetes Lanited para celda robotica
El paquete universal_robots se puede clonar desde 
https://github.com/ros-industrial/universal_robot.git
Para que funcione este espacio de trabajo debe estar enlazado a un ws que contenga los paquetes de universal robots

Para hacer overlaying de los dos paquetes.
1- Crear una carpeta para los Workspaces

2- Crear el ws de UR y el ws de lanited. Considerar que no tengan los directorios build, devel.

3- Primero compliar el paquete UR y hacer source

4- En seguida se compila el ws de lanited para enlazarlo con el ur.

5- Se hace el source a lanited y queda enlazado.

Verificar overlaying con $echo $ROS_PACKAGE_PATH
