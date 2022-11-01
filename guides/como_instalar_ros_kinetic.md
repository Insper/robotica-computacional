# Instalação do ROS Kinetic no Ubuntu 16.04

Siga [este guia http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu
) usando o Ubuntu 16.04, e instale o ros-kinetic-desktop-full

Depois, instale os seguintes pacotes:




    sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
    ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs

Depois, instale o gstreamer:

    sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-* gstreamer1.0-libav* gstreamer1.0-plugins*


Depois, instale o hping3 (para alguns workarounds da rede):


    sudo apt-get install hping3
    sudo setcap cap_net_raw+ep /usr/sbin/hping3

Configure seu workspace *catkin*:

Para entender o que acontece quando você cria um workspace, veja [este tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).


    source /opt/ros/kinetic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ..
    catkin_make

Edite seu arquivo `~/.bashrc` de modo que ele contenha a seguinte linha:

    source ~/catkin_ws/devel/setup.bash

Num terminal, faça o seguinte:

    cd ~/catkin_ws/src
    git clone https://github.com/ros-teleop/teleop_twist_keyboard.git    
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications.git   
    git clone https://github.com/mirwox/robot17   
    cd ..
    catkin_make .
    catkin_make



Para ver se tudo funcionou, num terminal faça:

    export TURTLEBOT3_MODEL=waffle_pi

Nós vamos usar o Turtlebot Burger com upgrades. No simulador é mais conveniente usarmos o Turtlebot Waffle do que fazermos o upgrade por nossa própria conta

Depois, no mesmo terminal do comando *export*:

    roslaunch turtlebot3_gazebo turtlebot3_world.launch

Num outro terminal, faça:

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Num terceiro terminal, faça:

    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch



Fonte original: [Curso de Robótica na Olin College](https://sites.google.com/site/comprobo17/how-to/setting-up-your-environment)
