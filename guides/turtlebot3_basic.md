
# Turtlebot 3 Básico


Estes são os comandos necessários para executar o simulador do Turtlebot

Cada comando precisa ser dado num terminal diferente.

Primeiro precisamos definir qual Turtlebot usar na simulação. A versão *Waffle* é interessante porque já vem com a câmera (este comando deve ser repetido em todo terminal, ou adicionado ao final do ~/.bashrc).

    export TURTLEBOT3_MODEL=waffle_pi

Depois iniciamos o ambiente virtual de simulação:

roslaunch turtlebot3_gazebo turtlebot3_house.launch




Ou você pode também testar num ambiente  de uma arena de robôs:

    roslaunch turtlebot3_gazebo turtlebot3_world.launch


Agora um terminal que nos permita controlar o robô com as teclas:

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Finalmente o RVIZ - painel de instrumentação:


roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch 

