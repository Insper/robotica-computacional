# Entendendo a Odometria

Nesta atividade vamos explorar o tópico de odometria, `odom`, que é um dos tópicos mais importantes para navegação de robôs móveis.

Odometria é a estimativa da posição e orientação de um robô móvel utilizando dados de sensores.
Estes atributos são chamados de Pose, que é a posição e orientação do robô no espaço. 

No caso do robô utilizado neste curso, a odometria é estimada utilizando os dados dos encoders dos motores.

!!! Importante
    Antes de começar, crie um novo pacote chamado `robcomp_util` com as seguintes dependências:
    ```bash
    ros2 pkg create --build-type ament_python robcomp_util --dependencies rclpy std_msgs geometry_msgs sensor_msgs
    ```

# Componentes da Pose

## Posição

A posição é composta por três coordenadas, x, y e z. Existem dois sistemas de coordenadas que podem ser utilizados para representar a posição do robô:

* Coordenadas globais: O sistema de coordenadas globais é fixo no mundo. A origem deste sistema de coordenadas é o ponto (0, 0, 0) do mundo.

* Coordenadas locais: O sistema de coordenadas locais é fixo no robô. A origem deste sistema de coordenadas é o centro do robô.

Isso está ilustrado na figura abaixo:

![Sistemas de coordenadas](figs/coordenadas.png)

## Orientação

### Orientação - Euler Angles

A orientação de um objeto pode ser descrita através de ângulos de Euler. Trata-se de três ângulos que especificam a rotação do objeto em torno dos eixos XX, YY, e ZZ. Geralmente, rotações nos eixos XX, YY, e ZZ são chamadas de roll, pitch, e yaw, respectivamente, como mostrado na imagem abaixo.

![Euler Angles](figs/euler.jpg)

Este método é intuitivo, mas pode sofrer de "gimbal lock". Gimbal lock é um problema que ocorre quando dois dos eixos de rotação estão alinhados. Neste caso, a rotação em torno de um eixo é perdida, gerando ambiguidade na orientação do objeto.

### Orientação - Quaternion

Uma alternativa aos ângulos de Euler é o uso de `quaternions`. Um quaternion é uma estrutura matemática que evita o problema de gimbal lock e é computacionalmente mais eficiente para algumas operações. Ele é representado como $q=w+xi+yj+zkq=w+xi+yj+zk$.

Assista o vídeo abaixo para entender como funciona a representação de orientação usando quaternions.

[Quaternions and 3d rotation, explained interactively](https://www.youtube.com/watch?v=zjMuIxRvygQ&t=233s)

# Tópico de Odometria

Agora que passamos pelos conceitos básicos, vamos ver como a odometria é representada na ROS2.

Vamos começar abrindo o simulador e o teleop através dos comandos, um em cada terminal:

- abrir o simulador:
```bash
ros2 launch my_gazebo pista-23B.launch.py
```
- Teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Antes de se increver no tópico `odom`, vamos olhar o tipo de mensagem e o conteúdo da mensagem. Para isso, abra um novo terminal e digite:

```bash
ros2 topic info /odom
```

O tipo da mensagem é `nav_msgs/msg/Odometry`, que é um tipo de mensagem padrão para odometria. Para ver o conteúdo da mensagem, digite:

```bash
ros2 interface show nav_msgs/msg/Odometry
```

A mensagem é composta por:

* `pose.pose.position`: A posição do robô no espaço. No caso do simulador, a origem do sistema de coordenadas é o centro da pista (encontro das linhas RGB - XYZ). No caso do robô real, a origem do sistema de coordenadas é estabelecida quando ele começa a se mover - portanto manualmente mover o robô, significa mover a origem do sistema de coordenadas.

* `pose.pose.orientation`: A orientação do robô no espaço.

* `twist.twist.linear`: A velocidade linear do robô no espaço. Aqui podemos ver um exemplo entre local e global. Por limitações de hardware, a velocidade linear local do robô só tem uma direção, para frente (eixo-X). No entanto, a velocidade linear global pode ter qualquer direção, no plano XY.

* `twist.twist.angular`: A velocidade angular do robô no espaço. Neste caso, a velocidade angular local e global são iguais, pois o robô só pode girar em torno do eixo Z.

Por fim, rode o `second_node` utilizando comando abaixo para ver o conteúdo da mensagem:

```bash
ros2 run my_package second_node
```

agora, ande com o robô utilizando o teleop, para ver como a odometria é atualizada.

!!! tip
    Para ajudar na compreenção o sistemas de coordenadas, primeiramente ande com o robô em uma direção por vez, primeiro para frente, pare, gire 90 graus, ande novamente e monitore a atualização da odometria.

## Módulo de Odometria - APS 3

Agora vamos encapsular a odometria em uma classe que pode ser facilmente importado em qualquer nó na ROS 2.

Dentro do pacote `robcomp_util`, crie um arquivo denominado `odom.py` e uma classe chamada `Odom` **sem nenhuma herança**, ou seja, não herde da classe `Node`. Essa classe deve:

!!! info
    Estamos removendo a herança para que você possa reutilizar a classe em qualquer nó, o que não seria possível se `Odom` herda-se de `Node`.

* Não inicie um nó nesse arquivo.

* Definir um subscriver para o tópico `odom` que chama a função `odom_callback` quando uma mensagem é recebida.

* Definir uma função `odom_callback` que recebe uma mensagem do tipo `nav_msgs/msg/Odometry` e armazena os seguintes parâmetros:

    * `self.x`: posição X do robô no espaço global.

    * `self.y`: posição Y do robô no espaço global.

    * `self.yaw`: orientação do robô no espaço global. Valor em radianos e no intervalo de `-pi` a `pi`.

Para auxiliar, enviamos uma função que faz conversão de quaternion para ângulos de Euler, que deve ser utilizada na função `odom_callback`:

```python
    def euler_from_quaternion(self, quaternion):
            """
            Converts quaternion (w in last place) to euler roll, pitch, yaw
            quaternion = [x, y, z, w]
            Below should be replaced when porting for ROS2 Python tf_conversions is done.
            """
            x = quaternion[0]
            y = quaternion[1]
            z = quaternion[2]
            w = quaternion[3]

            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (w * y - z * x)
            pitch = np.arcsin(sinp)

            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            return roll, pitch, yaw
```

### Testando

Para testar, baseado-se no arquivo `base.py` crie um arquivo chamado `test_odom.py`, dentro do pacote `robcomp_util`. Este arquivo deve conter um nó chamado `test_odom_node` que importa a classe `Odom` do arquivo `odom.py` e imprime a posição e orientação do robô no espaço global a cada 1 segundo.

Lembre-se:

* Importe a classe `Odom` da seguinte forma:
```python
from robcomp_util.odom import Odom
```

* Faça a herança da classe `Odom` no `test_odom_node`.

* Adicione o nó no arquivo `setup.py` e então compile o pacote.

* Rode o nó `test_odom_node` utilizando o comando `ros2 run robcomp_util test_odom` e mova o robô utilizando o teleop, para ver como a odometria é atualizada.
