# Pacotes e Comandos da ROS 2

Vamos começar iniciando o simulador novamente. Abra um terminal (`Ctrl` `Alt` `T`) e execute o seguinte comando:

```bash
ros2 launch my_gazebo pista-23B.launch.py
```

Vamos entender o que esse comando faz:
* `ros2` é o comando para executar programas da ROS 2;
* `launch` é o comando para executar um arquivo de configuração de lançamento (launch);
* `my_gazebo` é o nome do pacote que contém o arquivo de configuração de lançamento;
* `pista-23B.launch.py` é o nome do arquivo de configuração de lançamento.

Na ROS, pacotes (packages) são coleções de arquivos e programas que podem ser usados para realizar uma tarefa específica. O pacote `my_gazebo` contém os arquivos necessários para executar e gerar o mundo de simulação do robô.

Agora, rode novamente o comando para controlar o robô:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Novamente, vamos entender o que esse comando faz:
* `run` é o comando para executar um programa;
* `turtlebot3_teleop` é o nome do pacote que contém o programa;
* `teleop_keyboard` é o nome do programa.

A diferença entre um **programa** e um **arquivo de configuração de lançamento** é que o ultimo, é um arquivo que contém instruções para executar um ou mais programas, enquanto que um programa é um arquivo que contém instruções para realizar uma tarefa específica.

No caso do arquivo de configuração de lançamento `pista-23B.launch.py`, ele contém instruções para executar o simulador, gerar o mundo de simulação e carregar o robô. Já o programa `teleop_keyboard` contém instruções para controlar o robô.
