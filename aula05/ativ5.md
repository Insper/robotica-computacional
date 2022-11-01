
# Atividade 5 - projeto 

Familiarização com marcadores

Primeiramente atualize os repositórios my_simulation e robot20

Depois dispare o launch file:

    roslaunch my_simulation mundoteste_ar_tracking.launch

Monitore a visão do robô por:

    rqt_image_view


Dirija o robô até que esteja perto dos marcadores

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 

Observe o tópico de visão:

    rostopic echo /ar_pose_marker

Veja no Rviz os marcadores dando **add marker**

    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch



Em seguida dispare o código base em Python para observar:


    rosrun exemplos_python marcador_translacoes.py



## Agora faça:

Quais os ids dos marcadores visíveis na cena?

Escolha um deles, e baseado nas coordenadas de translação, faça o robô se aproximar do creeper a do id que escolheu, freiando o mais próximo possível









