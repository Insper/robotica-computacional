# Explorando Tópicos e Mensagens

Vamos começar com algumas definições:

* `Nodes` (Nós): Um nó é um processo que executa uma tarefa especifica na ROS. No nosso caso, um nó é um script python que vamos chamar diretamente.

* `Topics` (Tópicos): Os tópicos na ROS são barramentos onde a informação é trocada entre nós. Através de tópicos, nós podemos publicar e se inscrever para enviar e receber mensagens.
Quando um nó publica uma mensagem ela é enviada para todos os nós que estão inscritos nesta mensagem.
Tópicos são identificados por strings únicas.

* `Messages` (Mensagens): As mensagens são estruturas de dados que carregam informações. Elas são podem ser compostas por tipos primitivos, como inteiros, floats, strings, etc. ou por outras mensagens. As mensagens são usadas para publicar e receber informações nos tópicos.

Se o simulador da última atividade foi fechado, abra novamente, executando cada comando em um terminal diferente:

```bash 
ros2 launch my_gazebo pista-23B.launch.py
ros2 run turtlebot3_teleop teleop_keyboard
```

## Navegando nos Tópicos da ROS

Você pode listar os tópicos disponíveis com o seguinte o comando:

```bash 
ros2 topic list
```

O comando `list` retorna a lista de tópicos disponíveis na ROS no momento. Cada tópico transporta uma mensagem de um tipo específico. Para saber qual o tipo de mensagem que um tópico transporta, execute o seguinte comando:

```bash
ros2 topic info {nome_do_topico}
```

O comando `info` informa o tipo de mensagem que o tópico transporta. Vamos testar o `/cmd_vel`, execute o seguinte comando:

```bash
ros2 topic info /cmd_vel
```

Vamos entender a saída do comando acima:

* Type: geometry_msgs/msg/Twist - Esse é o tipo de mensagem que o tópico transporta, no caso, uma mensagem do tipo `Twist` que existe dentro do pacote `geometry_msgs`.

* Publisher count: 1 - Quantidade de nós que estão publicando mensagens neste tópico.

* Subscription count: 1 - Quantidade de nós que estão inscritos neste tópico.

**Dica:** Se a saída do comando acima for `Publisher count: 0`, inicie o programa `teleop_keyboard` novamente.

## Visualizando as Mensagens dos Tópicos

No comando anterior, vimos que o tópico `/cmd_vel` transporta mensagens do tipo `Twist` e que existe um nó publicando mensagens neste tópico. Podemos visualizar as mensagens que estão sendo publicadas em um tópico com o seguinte comando:

```bash
ros2 topic echo {nome_do_topico}
```

Então, para visualizar as mensagens que estão sendo publicadas no tópico `/scan`, execute o seguinte comando:

```bash
ros2 topic echo /cmd_vel
```

Mude o foco para o terminal rodando o programa `teleop_keyboard` e mova o robô. Você deve ver alterações nas mensagens sendo publicadas no tópico `/cmd_vel`.

## Visualizando a Estrutura das Mensagens

Uma mensagem do tipo `Twist` não explica muito sobre o que ela representa. Para entender melhor o que uma mensagem representa, podemos visualizar a estrutura da mensagem com o seguinte comando:

```bash
ros2 interface show {nome_do_pacote}/msg/{nome_da_mensagem}
```

Então, para visualizar a estrutura da mensagem `Twist`, execute o seguinte comando:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

A saída do comando acima mostra a estrutura da mensagem `Twist`, que é composta por duas mensagens do tipo `Vector3`, `linear` e `angular`. Cada mensagem do tipo `Vector3` é composta por três valores do tipo `float64`, `x`, `y` e `z`. `linear` representa a velocidade linear do robô e `angular` representa a velocidade angular do robô.

# Pratica Topico `/odom`

!!! exercise long 
    Qual o commando para visualizar as mensagens do tópico `odom`

    !!! answer
        ros2 topic echo /odom

!!! exercise long 
    Qual o tipo de mensagem que o tópico `odom` transporta?

    !!! answer
        nav_msgs/msg/Odometry

!!! exercise long 
    Qual o comando para visualizar a estrutura da mensagem que o tópico `odom` transporta?

    !!! answer
        ros2 interface show geometry_msgs/msg/Odometry

!!! exercise long 
    No simulador, mova o robô, como a saída do tópico `odom` muda?

    !!! answer
        Se o robô estiver se deslocando os valores do pose.pose.position vão variar de acordo com a direção do deslocamento. Se o robô estiver girando, os valores do pose.pose.orientation vão variar, no caso a orientação está sendo representada no formato de `quaternion`, isso será explorado melhor no futuro.

