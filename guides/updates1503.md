# Atualizações 15/3/2018

## Turtlebot - atualizações

Devido a [uma alteração](https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin/commit/6d874fc7e012bdce41fb8718cdaa96651ab012f7) realizada nos últimos dois dias, precisamos atualizar o simulador de Turtlebot.


	cd ~
	cd catkin_ws/src
	git clone https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin
	cd turtlebot3
	git pull
	cd ../turtlebot3_msgs
	git pull
	cd ../turtlebot3_simulations
	git pull
	cd ../robot18
	git pull
	cd ~/catkin_ws
	catkin_make


Lembrando que para ver os efeitos você deve fazer:

Num terminal:

    export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_gazebo turtlebot3_world.launch


Em outro terminal:



   export TURTLEBOT3_MODEL=waffle_pi
    roslaunch turtlebot3_gazebo turtlebot3_world.launch

Ainda em outro terminal:


	export TURTLEBOT3_MODEL=waffle_pi
	roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch 

Em mais outro terminal:


    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


Dica: use seu poder de dedução e descubra como evitar ficar digitando 	`export TURTLEBOT3_MODEL=waffle_pi`  toda hora.

## Anaconda em conflito com ROS

Evite ter o Anaconda instalado no mesmo usuário em que você trabalha com ROS. Vai gerar diversos conflitos.

Você pode ter Anaconda no Linux, mas crie um usuário separado:

	sudo adduser NOME_DO_USUARIO

Se você já tem o Anaconda e está tendo problemas com o catkin_make, **remova o Anaconda deste login**, depois faça o seguinte:

	cd ~/catkin_ws
	rm -Rf devel
	rm -Rf build
	cd ~/catkin_ws/src
	rm CMakeLists.txt
	cd ~/catkin_ws/src
	catkin_init_workspace
	cd ~/catkin_ws
	catkin_make






