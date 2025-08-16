# Explorando Tópicos e Mensagens

Vamos começar com algumas definições:

* `Nodes` (Nós): Um nó é um processo que executa uma tarefa **específica** na ROS 2. No nosso caso, um nó é um script **Python** que vamos executar diretamente.

* `Topics` (Tópicos): Tópicos são "barramentos" de comunicação onde a informação é trocada entre nós. Por meio deles, nós publicamos e **nos inscrevemos** (assinamos) para enviar e receber mensagens. Quando um nó publica uma mensagem, ela é entregue a todos os nós inscritos nesse tópico. Tópicos são identificados por **strings** únicas.

* `Messages` (Mensagens): Mensagens são estruturas de dados que carregam informações. Podem ser compostas por tipos primitivos (inteiros, `float`, `string`, etc.) ou por outras mensagens. São usadas para publicar e receber informações nos tópicos.

Se o simulador da última atividade foi fechado, abra novamente, **executando cada comando em um terminal diferente**:

```bash
ros2 launch my_gazebo pista-23B.launch.py
ros2 run turtlebot3_teleop teleop_keyboard
```

## Navegando nos Tópicos da ROS 2

Você pode listar os tópicos disponíveis com o seguinte comando:

```bash
ros2 topic list
```

O subcomando `list` retorna os tópicos disponíveis **no momento**. Cada tópico transporta mensagens de um **tipo específico**. Para descobrir o tipo de mensagem de um tópico, execute:

```bash
ros2 topic info {nome_do_topico}
```

Vamos testar com o tópico `/cmd_vel`:

```bash
ros2 topic info /cmd_vel
```

Entendendo a saída:

* **Type**: `geometry_msgs/msg/Twist` - tipo de mensagem transportada (a mensagem `Twist`, do pacote `geometry_msgs`).
* **Publisher count**: `1` - quantidade de nós publicando neste tópico (pode variar ao longo do tempo).
* **Subscription count**: `1` - quantidade de nós inscritos neste tópico (também pode variar).

!!! dica
    **Dica:** Se aparecer `Publisher count: 0`, garanta que o `teleop_keyboard` esteja em execução e com o terminal em foco. Para encerrar, use `Ctrl+C`.

## Visualizando as Mensagens dos Tópicos

Sabemos que `/cmd_vel` transporta mensagens `Twist` e que há um nó publicando. Para **ver** as mensagens sendo publicadas em um tópico, use:

```bash
ros2 topic echo {nome_do_topico}
```

Então, para visualizar as mensagens do tópico `/cmd_vel`, execute:

```bash
ros2 topic echo /cmd_vel
```

Alterne para o terminal do `teleop_keyboard` e mova o robô. Você deverá ver mudanças nas mensagens publicadas em `/cmd_vel`. (Para interromper o `echo`, use `Ctrl+C`.)

## Visualizando a Estrutura das Mensagens

O nome `Twist` por si só não explica a estrutura interna. Para ver o **formato** de uma mensagem, use:

```bash
ros2 interface show {nome_do_pacote}/msg/{nome_da_mensagem}
```

Para a mensagem `Twist`:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

A saída mostra que `Twist` possui dois campos do tipo `Vector3`: `linear` e `angular`. Cada `Vector3` contém três `float64`: `x`, `y` e `z`. `linear` representa a velocidade linear do robô e `angular` representa a velocidade angular do robô.

# Prática Tópico `/odom`

!!! exercise long
    Qual o **comando** para visualizar as mensagens do tópico `odom`?

        !!! answer
            ```bash
            ros2 topic echo /odom
            ```
            
            Você também pode usar o argumento `--once` para exibir apenas **uma** mensagem:
            
            ```bash
            ros2 topic echo /odom --once
            ```

!!! exercise long
    Qual o **tipo de mensagem** que o tópico `odom` transporta?

        !!! answer
            ```
            `nav_msgs/msg/Odometry`
            ```

!!! exercise long
    Qual o **comando** para visualizar a **estrutura** da mensagem transportada em `odom`?

        !!! answer
            ```bash
            ros2 interface show nav_msgs/msg/Odometry
            ```

!!! exercise long
    O que você imagina que acontece com os valores publicados em `odom` enquanto o robô se move?

    !!! answer
        Enquanto o robô se desloca, os valores de `pose.pose.position` **variarão** conforme a direção do movimento. Ao girar, os valores de `pose.pose.orientation` **mudam**. A orientação é representada como um **quaternion**; exploraremos esse tema com mais detalhes adiante.
