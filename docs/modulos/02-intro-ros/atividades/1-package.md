# Pacotes e Comandos da ROS 2

Vamos começar iniciando o simulador novamente. Abra um novo terminal (`Ctrl+Alt+T`) e execute o seguinte comando:

```bash
ros2 launch my_gazebo pista-23B.launch.py
```

Vamos entender o que esse comando faz:

* `ros2` é a interface de linha de comando da ROS 2.
* `launch` invoca o sistema de lançamento (launch) da ROS 2 para executar um arquivo `.launch.py`.
* `my_gazebo` é o nome do **pacote** que contém o arquivo de lançamento.
* `pista-23B.launch.py` é o **arquivo de lançamento** dentro do pacote (geralmente localizado em `launch/`).

Na ROS 2, **pacotes (packages)** são coleções organizadas de nós/programas, bibliotecas, configurações e arquivos de lançamento usados para realizar tarefas específicas. O pacote `my_gazebo` reúne os arquivos necessários para iniciar o simulador, gerar o mundo e carregar o robô.

Agora, abra um novo terminal (`Ctrl+Alt+T`) e rode novamente o comando para controlar o robô:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Novamente, vamos entender o que esse comando faz:

* `run` executa um **programa** (também chamado de nó).
* `turtlebot3_teleop` é o nome do **pacote** que contém o programa.
* `teleop_keyboard` é o **executável** que lê o teclado e publica comandos de velocidade para o robô. (Mantenha o terminal em foco; para sair, use `Ctrl+C`.)

A diferença entre um **programa** e um **arquivo de lançamento** é que o **arquivo de lançamento** descreve como iniciar **um ou mais programas** (nós), com parâmetros, remapeamentos e dependências; já um **programa** é um executável único que realiza uma tarefa específica.

No caso do arquivo de lançamento `pista-23B.launch.py`, ele contém instruções para iniciar o simulador, gerar o mundo e carregar o robô. Já o programa `teleop_keyboard` contém as instruções para iniciar o nó para controlar o robô pelo teclado.
