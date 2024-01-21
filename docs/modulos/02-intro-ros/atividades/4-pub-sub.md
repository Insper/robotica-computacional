# Criando um Pubshisher e um Subscriber na ROS

## Pubshisher e Subscriber
Agora que você já sabe como criar um novo pacote, vamos aprender como se inscrever em um tópico para receber mensagens. Primeiro, vamos para algumas definições:

* **Subscriber:** Um subscriber é um nó que recebe mensagens de um tópico específico.

* **Publisher:** Um publisher é um nó que envia mensagens para um tópico específico.

Na ROS, quando um nó enviar uma mensagem para um tópico, todos os nós que estiverem inscritos nesse tópico receberão a mensagem. A figura abaixo ilustra esse processo:

![ros-pub-sub](imgs/ros-pub-sub.png)

## Criando um Publisher

Vamos começar abrindo um terminal e executando o comando abaixo para abrir o VSCode no diretório do pacote `my_package`:

```bash
cd ~/colcon_ws/src/my_package
code .
```

Agora vamos criar um novo arquivo chamado `first_node.py` dentro da pasta `my_package`. Esta pasta contém todos os arquivos Python que serão executados como nós ROS. 

No mesmo terminal rode os comandos abaixo para criar o arquivo:

```bash
cd my_package
touch first_node.py
chmod +x *.py
```

Os comandos acima fazem o seguinte:

* `cd my_package`: entra na pasta `my_package`.

* `touch first_node.py`: cria o arquivo `first_node.py`.

* `chmod +x *.py`: dá permissão de execução para todos os arquivos Python da pasta. Este comando é necessário para que o ROS consiga executar o arquivo Python como um nó ROS.

Agora vamos abrir o arquivo `first_node.py` e adicionar o seguinte código:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

"""
ros2 launch my_package first_node.launch.py
"""

class FirstNode(Node):

    def __init__(self):
        super().__init__('first_node')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.25, self.control)
        
    def control(self):
        self.twist = Twist()
        self.twist.linear.x = -0.2

        self.vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    first_node = FirstNode()

    rclpy.spin(first_node)
    
    first_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


### Explicação do Código
Vamos entender o que cada linha do código acima faz:

* `super().__init__('first_node')`: cria um nó com o nome `first_node`.

* `self.create_publisher(Twist, 'cmd_vel', 10)`: recebe três argumentos:

    * `Twist`: tipo da mensagem que será publicada.

    * `'cmd_vel'`: nome do tópico que será publicado.

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

Para rodar o nó precisamos fazer duas coisas:

* Criar um arquivo launch.

* Configurar o arquivo `setup.py` para que a ROS consiga encontrar o nó.

## Criando um Arquivo Launch

Vamos criar um arquivo launch chamado `first_node.launch.py` dentro da pasta `my_package`. Este arquivo será responsável por iniciar o nó `first_node.py`.

No mesmo terminal rode os comandos abaixo para criar o arquivo:

```bash
cd ~/colcon_ws/src/my_package
mkdir launch
cd launch
touch first_node.launch.py
```

Agora vamos abrir o arquivo `first_node.launch.py` e adicionar o seguinte código:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='first_node',
            output='screen'),
    ],)
```

* `Node` recebe três argumentos:

    * `package='my_package'`: nome do pacote que contém o nó.

    * `executable='first_node'`: nome do nó que será executado.

    * `output='screen'`: imprime a saída do nó no terminal.

## Configurando o Arquivo setup.py

Agora precisamos configurar o arquivo `setup.py` para que a ROS consiga encontrar o nó. Para isso, abra o arquivo `setup.py` e adicione o seguinte código:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='somebody very awesome',
    maintainer_email='user@user.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_node = my_package.first_node:main',
        ],
    },
)
```

O que foi alterado no arquivo `setup.py` foi o seguinte:

* Adicionamos dois novos imports:

* Dentro da lista `data_files` adicionamos a seguinte linha:

    * `(os.path.join('share', package_name), glob('launch/*.launch.py'))`: adiciona todos os arquivos launch da pasta `launch` ao pacote.

* Dentro do diciário `entry_points` na chave `console_scripts` adicionamos a seguinte linha:

    * `'first_node = my_package.first_node:main'`: cria um comando chamado `first_node` que executa a função `main` do arquivo `first_node.py`.

### Rodando o Nó

Antes de rodar o nó, precisamos compilar o pacote novamente. Para isso, abra um novo terminal e rode os comandos abaixo:

```bash
cd ~/colcon_ws
colcon build --packages-select my_package
```

Dessa vez, enviamos o argumento `--packages-select` para o comando `colcon build`. Este argumento faz com que o comando compile apenas o pacote `my_package`.

Rode o nó com o comando abaixo:
**IMPORTANTE:** Certifique-se de que o simulador está rodando antes de executar o comando abaixo.

```bash
ros2 launch my_package first_node.launch.py
```

## Criando um Subscriber

rodar com ROS RUN


tralla