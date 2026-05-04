# ROS2 SLAM

Referência: [ROS2 SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node)


SLAM (Simultaneous Localization and Mapping) ou Mapeamento e Localização Simultâneos é uma técnica usada por robôs e veículos autônomos para construir um mapa do ambiente e, ao mesmo tempo, localizar sua posição nesse ambiente. A ROS2 oferece uma série de pacotes que permitem a execução de algoritmos de SLAM no Turtlebot3.

Veja o vídeo abaixo de demonstração do SLAM no Turtlebot3:

[Turtlebot3 SLAM](https://www.youtube.com/watch?v=pJNSxDodhDk)

## Instalando os pacotes necessários

Para instalar os pacotes necessários para o SLAM, execute os comandos abaixo:

```bash
cd /home/borg/colcon_ws/src/my_simulation/atualiza_infra
git stash
git pull
./atualiza_infra_2024.sh
```

## Executando o Mapeamento

Para iniciar o mapeamento do ambiente, devemos executar o pacote denominado `Cartographer`. Para isso, execute os comandos abaixo:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

Agora, abra um novo terminal com o `teleop` para controlar o Turtlebot3:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Com o `teleop` aberto, movimente o Turtlebot3 pelo ambiente para que o mapeamento seja realizado. Após finalizar o mapeamento, execute o comando abaixo para salvar o mapa:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

O mapa será salvo no diretório `~/map` com os arquivos `map.pgm` e `map.yaml`.

## Visualizando o Mapa

O mapa será salvo como um arquivo de imagem no formato `pgm`. Como no exemplo abaixo:

![Mapa](../util/map.png)

O mapa ilustra o ambiente mapeado, onde os pixels pretos representam obstáculos, os pixels brancos representam áreas livres e os pixels cinzas representam áreas desconhecidas.

## Navegando no Mapa

Agora para navegar no ambiente, primeiro **feche todos os terminais abertos** e execute o pacote `Navigation`, através do comando abaixo:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

Uma vez que o pacote de navegação esteja em execução, vamos primeiro definir a posição inicial do Turtlebot3 no mapa, para isso, clique no botão `2D Pose Estimate` no Rviz e clique e segure o botão esquerdo do mouse no local onde o Turtlebot3 está localizado no mapa. Em seguida, arraste o mouse para a direção que o Turtlebot3 está apontando e solte o botão esquerdo do mouse, conforme ilustrado na imagem abaixo:

![2D Pose Estimate](../util/2d_pose_estimate.png)


## Definindo o Ponto de Destino da Navegação

Agora, para definir o ponto de destino da navegação, clique no botão `2D Nav Goal` no Rviz e clique e segure o botão esquerdo do mouse no local onde deseja que o Turtlebot3 vá. Em seguida, arraste o mouse para a direção que deseja que o Turtlebot3 esteja apontando e solte o botão esquerdo do mouse.

## Combinando Mapeamento e Navegação (SLAM)

Agora, vamos aprender a combinar o mapeamento e a navegação, executando o pacote `Navigation` e o pacote `Cartographer` juntos. Para isso, execute o comando abaixo (cada um em um terminal diferente):

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True \
map:='topic://map'
```

### Extraindo a posição do robô no mapa

Quando usamos o sistema de navegação, o tópico `/odom` sozinho não é suficiente para obter a posição do robô no mapa. Isso acontece porque `/odom` representa a pose do robô em um sistema de coordenadas local, associado ao frame `odom`.

Para obter a posição global do robô no mapa, precisamos combinar a odometria com as transformações publicadas no tópico `/tf`. Em especial, usamos a transformação entre os frames `map` e `odom`, junto com a pose do robô em relação ao frame `odom`.

De forma simplificada, a composição é:

```text
map -> odom -> base_link
```

Assim, conseguimos estimar a pose do robô no frame `map`, ou seja:

```text
x, y, yaw no mapa
```

Essa lógica já foi implementada no arquivo [`amcl.py`](https://insper.github.io/robotica-computacional/modulos/08-slam/util/amcl.py). Diferente da classe `Odom`, a classe `AMCL` deve ser executada como um nó ROS independente, publicando a pose global estimada no tópico `/amcl_pose`.

O tópico publicado segue o formato:

```text
/amcl_pose
```

com o tipo:

```text
geometry_msgs/msg/PoseWithCovarianceStamped
```

Nesse tópico:

- `pose.pose.position.x` representa a posição do robô no eixo X do mapa (metros);
- `pose.pose.position.y` representa a posição do robô no eixo Y do mapa (metros);
- `pose.pose.orientation` representa a orientação do robô no mapa.