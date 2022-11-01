
Antes de qualquer passo ligue a Raspberry Pi. O endereço IP deve ser visto na tela.

Aguarde a Raspberry Pi tocar uma escala musical ascendente.



## No PC


Certifique-se de que todo o software do Turtlebot e do curso está atualizado:

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


### Em cada terminal

Crie a variável de ambiente para conectar-se à Raspberry Pi, substituindo IPBerry pelo IP do seu robô:

	export IPBerry=192.168.0.51
	export ROS_MASTER_URI="http://"$IPBerry":11311"

Crie a variável de ambiente do PC

	export ROS_IP=`hostname -I`

Indique o tipo de robô que está usando

	export TURTLEBOT3_MODEL=burger

Nota: você pode colocar as três linhas acima no arquivo `~/.bashrc` para evitar ter que digitar em todo terminal



Agora execute:

	roscore

Depois, num outro terminal:

	export IPBerry=192.168.0.51
	export ROS_MASTER_URI="http://"$IPBerry":11311"
	export ROS_IP=`hostname -I`
	export TURTLEBOT3_MODEL=burger

	roslaunch turtlebot3_bringup turtlebot3_remote.launch

Depois, finalmente em mais um terminal:

	export IPBerry=192.168.0.51
	export ROS_MASTER_URI="http://"$IPBerry":11311"
	export ROS_IP=`hostname -I`
	export TURTLEBOT3_MODEL=burger

	rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz


Sempre que a câmera estiver invertida, rode para ajustá-la:

	export IPBerry=192.168.0.51
	export ROS_MASTER_URI="http://"$IPBerry":11311"
	export ROS_IP=`hostname -I`
	export TURTLEBOT3_MODEL=burger

	rosrun rqt_reconfigure rqt_reconfigure


Se quiser teleoperar o robô faça:

	export IPBerry=192.168.0.51
	export ROS_MASTER_URI="http://"$IPBerry":11311"
	export ROS_IP=`hostname -I`
	export TURTLEBOT3_MODEL=burger

	rosrun rqt_image_view rqt_image_view


Para acionar o comando por teclas:

	export IPBerry=192.168.0.51
	export ROS_MASTER_URI="http://"$IPBerry":11311"
	export ROS_IP=`hostname -I`
	export TURTLEBOT3_MODEL=burger

	roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


## No robô novamente, para desligar

Aperte o botão para baixo até o laser para de desligar.




# Em caso de problemas no robô


Conecte-se ao ssh

	ssh pi@$IPBerry

No terminal da Raspberry Pi digite:

	screen -r

Você deverá ver mensagens de erro do ROS, alternando entre telas com a opção  `Crtl` + `A` seguindo de `"` .

Caso haja um erro feche o `screen` e execute:

	cd
	./start_turtle.sh

Ou então simplesmente reinicie o robô



Assim que as luzes da Raspberry pararem de piscar desligue a energia no cabo


Fonte: [Manual do Bringup do Turtlebot3 - documentação oficial](http://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup)


	







