
## No PC
Conectar-se à raspberry:

Você precisa do IPBerry e da senha (pergunte a senha da semana ao professor). O IPBerry deve ser visto na tela do seu robô.

	
	export IPBerry=192.168.0.51

<Font color=red>Este IP é apenas um exemplo!<font>


	ssh pi@$IPBerry

Os comandos da seção a seguir devem ser digitados na Raspberry Pi (via ssh).


## Na Raspberry


Certifique-se de que o cabo da bateria (plug J2 vermelho) está conectado.

Ligue o equipamento na chave de liga-desliga


Rode na Raspberry:

Primeiramente precisamos do gerenciador de sessões, para fazer caber vários terminais num só ssh:

	screen

Para todas as instruções abaixo a separação por vírgula quer dizer que o primeiro atalho deve ser realizado, depois solta-se o teclado antes de proceder ao segundo atalho.

Agora crie 3 sub-sessões dentro da sessão:

`Ctrl A, C`  (Ctrl A, depois solta tudo e digita C)
`Ctr A, C`
`Ctrl A, C`

Agora volte para a segunda sessão:

`Ctrl A, "`

Note que em algumas plataformas para que a tecla de `"` seja considerada apertada é necessário apertar barra de espaço em seguida.


**(1) ** Deve aparecer uma lista de sub-terminais do `screen`. Usando a seta direcional vá para o segundo terminal e digite o seguinte comando para iniciar a leitura e controle dos motores e sensores básicos:


	roslaunch turtlebot3_bringup turtlebot3_core.launch

**(2)** Agora digite `Ctrl A, "` e selecione o segundo terminal. Digite nele:

	roslaunch turtlebot3_bringup turtlebot3_lidar.launch

**(3)**Em seguida inicie os serviço de stream da câmera.

	roslaunch raspicam_node camerav2_640x480_30fps.launch

Note que você pode variar os parâmetros de launch para abrir a câmera com outras resoluções. <font color=red>Outras opções</font>:


	roslaunch raspicam_node camerav2_1280x720.launch
	roslaunch raspicam_node camerav2_410x308_30fps.launch
	roslaunch raspicam_node camerav2_1280x960_10fps.launch 
	roslaunch raspicam_node camerav2_640x480_30fps.launch
	roslaunch raspicam_node camerav2_1280x960.launch  

Ao final, você pode deixar o `ssd` aberto ou encerrar a sessão. Se quiser encerrar, dê

`Ctrl A, D`

Este comando faz o detach da sessão e permite que você encerre o SSH sem matar os programas.

Digite na linha de comando:

	exit

## De volta no PC


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
	cd ../robot18-2
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

Conecte-se ao ssh

Digite:

	sudo shutdown -P now

Assim que as luzes da Raspberry pararem de piscar desligue a energia no cabo


Fonte: [Manual do Bringup do Turtlebot3 - documentação oficial](http://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup)


	







