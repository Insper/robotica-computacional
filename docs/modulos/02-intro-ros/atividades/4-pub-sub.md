# Criando um Publisher e um Subscriber na ROS 2

## Publisher e Subscriber

Agora que você já sabe como criar um novo pacote, vamos aprender como **publicar** e **assinar** tópicos para enviar/receber mensagens.

Primeiro, algumas definições:

* **Subscriber:** nó que **recebe** mensagens de um tópico específico (assina no tópico).
* **Publisher:** nó que **envia** mensagens para um tópico específico (publica no tópico).

Na ROS 2, quando um nó publica uma mensagem em um tópico, **todos** os nós assinantes nesse tópico a recebem. O gif abaixo ilustra esse processo:

![ros-pub-sub](imgs/ros-pub-sub.gif)

## Criando um Publisher

Abra um novo terminal (`Ctrl+Alt+T`) e inicie o VS Code no diretório do pacote `my_package`:

```bash
cd ~/colcon_ws/src/my_package
code .
```

Crie um arquivo chamado `first_node.py` dentro da pasta **`my_package/`** (essa pasta contém os arquivos Python executados como nós ROS):

```bash
cd my_package
touch first_node.py
chmod +x *.py
```

Os comandos acima fazem o seguinte:

* `cd my_package`: entra na pasta do módulo Python do pacote `my_package`.
* `touch first_node.py`: cria o arquivo do nó.
* `chmod +x *.py`: concede permissão de execução aos arquivos Python (necessário para executá‑los como nós). **Lembre-se sempre de usar este comando após criar um novo arquivo Python que será executado como nó ROS.**

Abra `first_node.py` e cole o código abaixo:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class FirstNode(Node):
    def __init__(self):
        super().__init__('first_node')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.25, self.control)

    def control(self):
        msg = Twist()
        msg.linear.x = 0.2  # m/s
        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Explicação do Código

* `super().__init__('first_node')`: cria um nó chamado `first_node`.
* `self.create_publisher(Twist, '/cmd_vel', 10)`: recebe três argumentos:

    * `Twist`: tipo da mensagem que será publicada.

    * `'/cmd_vel'`: nome do tópico que será publicado.

    * `10`: tamanho da fila de mensagens. Este argumento é opcional e o valor padrão é 10.


* `self.create_timer(0.25, self.control)`: recebe dois argumentos:

    * `0.25`: tempo entre execuções da função `self.control`.

    * `self.control`: função que será executada a cada 0.25 segundos.

A função `self.control` será chamada a cada 0.25 segundos, e em cada chamada, ela cria uma mensagem do tipo `Twist`, altera o valor da velocidade linear para 0.2 e publica a mensagem no tópico `cmd_vel`.

Por fim, a função `main` faz o seguinte:

* `rclpy.init(args=args)`: inicializa o módulo `rclpy` (ROS Client Library for Python).

* `first_node = FirstNode()`: cria uma instância da classe `FirstNode`.

* `rclpy.spin(first_node)`: mantém o nó em execução até que ele seja finalizado.

* `first_node.destroy_node()`: finaliza o nó.

* `rclpy.shutdown()`: finaliza o módulo `rclpy`.

Para executar o nó, precisamos fazer duas coisas:

* Criar uma entrada em `setup.py` para expor o executável.
* Compilar o pacote e atualizar o ambiente.

## Configurando o Arquivo `setup.py`

Abra `setup.py` e garanta que a entrada a seguir exista dentro de `entry_points -> console_scripts`:

```python
    entry_points={
        'console_scripts': [
            'first_node = my_package.first_node:main', # Adiciona o nó first_node
        ],
    },
```

Essa linha cria o comando `first_node` que executa a função `main` em `my_package/first_node.py`.

### Rodando o Nó

!!! atenção
    Para evitar conflito, **não** rode o `teleop_keyboard` ao mesmo tempo que este publisher no mesmo tópico. Caso rode ambos, as mensagens podem competir.

Compile e rode apenas o pacote `my_package`:

```bash
cd ~/colcon_ws
colcon build --packages-select my_package
source ~/colcon_ws/install/setup.bash
```

Dessa vez, enviamos o argumento `--packages-select` para o comando `colcon build`. Este argumento faz com que o comando compile apenas o pacote `my_package`.

Rode o nó com o comando abaixo:
**IMPORTANTE:** Certifique-se de que o simulador está rodando antes de executar o comando abaixo!

```bash
ros2 run my_package first_node
```

## Criando um Subscriber

Com o `first_node` em execução, o robô publicará comandos no `/cmd_vel`. Agora criaremos um nó que **assina** o tópico `/odom` e imprime a posição recebida.

Crie `second_node.py` em `my_package/` e torne o arquivo executável (como feito para o `first_node.py`).

Antes, confirme o tipo do tópico `odom`:

```bash
ros2 topic info /odom
```

O tópico `/odom` usa mensagens do tipo `nav_msgs/msg/Odometry`. Abra `second_node.py` e adicione o código:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

class SecondNode(Node):
    def __init__(self):
        super().__init__('second_node')
        self.x = 0.0
        self.y = 0.0

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.timer = self.create_timer(0.25, self.control)

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def control(self):
        print(f'Posição x: {self.x:.3f}')
        print(f'Posição y: {self.y:.3f}\n')


def main(args=None):
    rclpy.init(args=args)
    node = SecondNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Explicação do Código

* `create_subscription(...)`: assina o tópico `/odom` recebendo mensagens `Odometry`.

  * **Tipo**: `Odometry` - obtido com `ros2 topic info /odom`.
  * **Tópico**: `'/odom'` - nome do tópico assinado.
  * **Callback**: `self.odom_callback` - executada a cada mensagem recebida.
  * **QoS**: `QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)` - perfil de qualidade de serviço (opcional).
* `odom_callback`: atualiza `x` e `y` com a posição do robô.
* `control`: imprime periodicamente as posições.

!!! dica
    Você pode substituir `print(...)` por `self.get_logger().info(...)` para usar o sistema de **logs** do ROS 2.

### Configurando o Arquivo `setup.py`

Agora precisamos configurar o arquivo `setup.py` para que a ROS consiga encontrar o nó e então compilar o pacote novamente. Para isso, abra o arquivo `setup.py` e adicione a seguinte linha dentro do dicionário `entry_points` na chave `console_scripts`:

```python
'second_node = my_package.second_node:main',
```

### Rodando o Nó

Uma vez que o arquivo `setup.py` foi configurado, **compile** o pacote novamente com o mesmo comando que usamos anteriormente e rode o nó `first_node.py` com o comando mencionado anteriormente:

```bash
cd ~/colcon_ws
colcon build --packages-select my_package
source ~/colcon_ws/install/setup.bash

ros2 run my_package first_node
```

E agora, em um novo terminal, execute o comando abaixo para rodar o nó `second_node.py`:

```bash
ros2 run my_package second_node
```

Você deverá ver as posições **x** e **y** do robô sendo impressas a cada 0,25 segundos. Mova o robô e observe as mudanças nos valores.