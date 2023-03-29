# Explorando tópicos e mensagens

Primeiramente vamos inciar com algumas definições:
* Nodes (Nós): Um nó na ROS é um processo que executa algum tipo de processo. No caso da disciplina, um nó seria um script do python que vamos chamar diretamente.
* Topics (Tópicos): Os tópicos na ROS são barramentos onde a informação é trocada entre nós. Através de tópicos, nós podemos publicar e se inscrever para enviar e receber mensagens.
Quando um nó publica uma mensagem ela é enviada para todos os nós que estão inscritos nesta mensagem.
Tópicos são identificados por strings únicas.

Se o simulador da última atividade foi fechado, abra novamente, executando cada comando a seguir em seu terminal.

```bash 
roslaunch my_simulation pista_s2.launch
rqt_image_view
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

## Como descobrir a lista de tópicos do ROS
Você pode listar os tópicos existentes com o seguinte o comando `rostopic list`.

Exemplo:

```bash 
borg@ubuntu:~$ rostopic list
/bumper_states
/camera/camera_info
/camera/image
/camera/image/compressed
/camera/image/compressed/parameter_descriptions
/camera/image/compressed/parameter_updates
/camera/image/compressedDepth
/camera/image/compressedDepth/parameter_descriptions
/camera/image/compressedDepth/parameter_updates
/camera/image/theora
/camera/image/theora/parameter_descriptions
/camera/image/theora/parameter_updates
/camera/parameter_descriptions
/camera/parameter_updates
/clock
/cmd_vel
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo_ros_control/pid_gains/joint3/parameter_descriptions
/gazebo_ros_control/pid_gains/joint3/parameter_updates
/gazebo_ros_control/pid_gains/joint3/state
/imu
/joint_states
/odom
/rosout
/rosout_agg
/rqt_gui_cpp_node_22867/compressed/parameter_descriptions
/rqt_gui_cpp_node_22867/compressed/parameter_updates
/scan
/statistics
/tf
```

## Para inspecionar um tópico
O comando `rostopic echo TOPICO` permite ver os dados que estão circulando em algum dos tópicos. Por exemplo, vamos olhar a saída do sensor laser sobre o robô:

```bash 
borg@ubuntu:~$ rostopic echo /scan
header: 
  seq: 7
  stamp: 
    secs: 808
    nsecs: 154000000
  frame_id: "base_scan"
angle_min: 0.0
angle_max: 6.28318977355957
angle_increment: 0.017501922324299812
time_increment: 0.0
scan_time: 0.0
range_min: 0.11999999731779099
range_max: 3.5
ranges: [inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.064145803451538, 2.0749526023864746, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 3.017547845840454, 3.0342345237731934, 3.0564873218536377, 3.0211501121520996, 3.0298988819122314, 3.0324952602386475, 3.03559947013855, 3.048578977584839, 3.0339245796203613, 3.0593883991241455, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf,
...
```
A mensagem no exemplo acima foi cortada por ser muito grande.

### Descobrindo o tipo de dados de um tópico

Precisamos descobrir o **tipo** de dado de um certo tópico.

Um tópico é carrega uma mensagem no formato de um **objeto** e o **tipo** é uma *Classe*.

Para descobrir o tipo de dado de um tópico, usamos o comando `rostopic info TOPICO`.

Por exemplo, em um novo terminal execute:

```bash
borg@ubuntu:~$ rostopic info /scan
Type: sensor_msgs/LaserScan

Publishers: 
 * /gazebo (http://borg:42221/)

Subscribers: 
 * /rostopic_49645_1677515258406 (http://borg:37311/)
```

Para descobrir quais as variáveis internas do dado do tópico, usamos o comando `rosmsg`:

```bash
borg@ubuntu:~$ rosmsg info sensor_msgs/LaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

O que podemos ver é que, cada mensagem do laser do robô que é publicada no tópico `/scan` tem a estrutura `sensor_msgs/LaserScan`. Cada mensagem tem um header, que contém o identificador de quando a mensagem foi gerada. Cada leitura é feita de 0 a 6.28318977355957 (2*pi) em incrementos de 0.017501922324299812 (1 deg). O valor mínimo de uma leitura é de 0 m e máximo de 3.5m, uma lista com todas as leituras está presente no atributo `ranges`, que, portanto, é uma lista de `float32` com tamanho 360. 

**Pergunta para pensar:** Se o valor máximo é 3.5m, o que significa o valor `inf`?

## Para escrever em um tópico no terminal

Existem tópicos de leitura e tópicos de escrita. Por exemplo, é comum que o `cmd_vel` seja a velocidade desejada de um robô no ROS.

Podemos escrever a velocidade desejada usando o `rostopic pub` para o `cmd_vel`

O comando abaixo manda o robô ir para a frente com uma taxa de 10 vezes por segundo ( `-r 10` ) este comando sera util quando estiver trabalhando com o robo e deve ser utilizado com um comando de EMERGENCIA.

    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# Exercicio Topico /odom

!!! exercise long 
    Mostre uma mensagem do tópico `odom`

    !!! answer
        A mensagem deve estar no formato XXXX 

!!! exercise long 
    Qual o tipo de mensagem que o topico `odom` transporta?

    !!! answer
        A mensagem deve estar no formato XXXX 

!!! exercise long 
    Qual a estrutura da mensagem que o topico `odom` transporta?

    !!! answer
        A mensagem deve estar no formato XXXX 

!!! exercise long 
    No simulador, mova o robô, como a saída do tópico `odom` muda?

    !!! answer
        A mensagem deve estar no formato XXXX 