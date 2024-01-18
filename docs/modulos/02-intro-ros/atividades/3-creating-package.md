# Criando um novo Pacote na ROS 2

Agora, vamos aprender a criar um novo pacote na ROS 2. Abra um terminal (`Ctrl` `Alt` `T`) e execute o seguinte comando:

```bash
cd ~/colcon_ws/src
ros2 pkg create --build-type ament_python my_package --dependencies rclpy std_msgs geometry_msgs
```

Aqui enviamos dois comandos no terminal, o primeiro muda o diretório atual para a pasta `src` do workspace `colcon_ws`, é nessa pasta que **todos** os pacotes da ROS 2 são criados.

O segundo comando é para criar um novo pacote chamado `my_package` dentro da pasta `src`. Vamos entender o que esse comando faz:

* `ros2` é o comando para executar programas da ROS 2;

* `pkg create` é o comando para criar um novo pacote;

* `--build-type ament_python` é o tipo de pacote que estamos criando, nesse caso, um pacote em **Python**;

* `my_package` é o nome do pacote que estamos criando;

* `--dependencies rclpy std_msgs geometry_msgs` são as dependências do pacote que estamos criando, nesse caso, o pacote `rclpy` e os pacotes `std_msgs` e `geometry_msgs`.

Na ROS, depois de criar um pacote, é necessário compilar o pacote para que ele possa ser executado. Para compilar os pacotes da ROS 2, execute os seguintes comandos:

```bash
cd ~/colcon_ws
colcon build
```

Muito bem, agora que o pacote foi criado e compilado, siga para a próxima atividade para aprender a criar um nó na ROS 2.
