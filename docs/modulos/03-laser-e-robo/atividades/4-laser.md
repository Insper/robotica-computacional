# Entendendo o Sensor Laser

Nesta atividade vamos explorar o tópico do sensor laser, `scan`. Este sensor se encontra no topo do robô e é utilizado para detectar obstáculos no ambiente.

Para entender como o sensor funciona, vamos primeiro ver o tipo de mensagem que é enviado no tópico `scan`. Para isso, abra um novo terminal e digite (Certifique-se que o simulador rodando antes):

```bash
ros2 topic info /scan
```

O tipo da mensagem é `sensor_msgs/msg/LaserScan`, que é um tipo de mensagem padrão para sensores laser. Para ver o conteúdo da mensagem, digite:

```bash
ros2 interface show sensor_msgs/msg/LaserScan
```

Agora utilize o comando `echo` para ver o conteúdo do tópico `scan`:
Apenas uma vez
```bash
ros2 topic echo /scan 
```

Um exemplo de mensagem é mostrado abaixo:

```bash
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
ranges: [inf, inf, inf, ...]
intensities: [...]
```
A mensagem no exemplo acima foi cortada por ser muito grande.

A mensagem do sensor laser é composta pelos seguintes campos:

* `header`: Cabeçalho da mensagem, que contém informações como o tempo de envio da mensagem e o frame de referência.

* `angle_min: 0.0`: Ângulo inicial do sensor. O valor `0.0` corresponde a leiura do sensor diretamente para frente do robô.

* `angle_max: 6.28...`: Ângulo final do sensor. O valor `6.28...` equivale a uma volta completa (360 graus).

* `angle_increment: 0.017...`: Incremento angular entre cada leitura do sensor. O valor `0.017...` equivale a um ângulo de 1 grau.

* `scan_time & time_increment: 0.0`: Tempo de varredura do sensor e tempo entre cada leitura. O valor `0.0` indica que o sensor está configurado para enviar as leituras o mais rápido possível.

* `range_min: 0.119...` [m]: Distância mínima que o sensor consegue detectar. O valor `0.119...` equivale a 11.9 cm.

* `range_max: 3.5` [m]: Distância máxima que o sensor consegue detectar. O valor `3.5` equivale a 3.5 m.

* `ranges: [inf, inf, inf, ...]`: Vetor com as leituras do sensor. O tamanho do vetor é igual a `angle_max/angle_increment`, ou seja, a lista de leituras é composta por 360 elementos que representam as leituras do sensor a cada 1 grau. O valor `inf` indica que o sensor não detectou nada naquela direção.

* `intensities: [...]`: Vetor com as intensidades das leituras do sensor. Nosso sensor não possui essa informação, portanto, pode desconsiderar esse campo.

Portanto, no valor `ranges`, o sensor retonar um vetor de 360 elementos, que representam as leituras da distância do sensor a cada 1 grau. As medições são no sentido anti-horário, sendo 0 graus na parte de frente do robô. Na **simulação**, valor `inf` indica que o sensor não detectou nada naquela direção, no **robô real**, o valor `0` indica que o sensor não conseguiu fazer a leitura.

**Pergunta:** Qual o indice do vetor `ranges` que representa a leitura do sensor diretamente para frente do robô? E da esquerda? E da direita? E para trás?

## Módulo do Laser - APS 3

Vamos criar encapsular a leitura do laser em uma classe que pode ser facilmente importado em qualquer nó na ROS 2.

Dentro do pacote `robcomp_util/robcomp_util`, crie um arquivo denominado `laser.py` e uma classe chamada `Laser` e siga os seguintes passos:

1. Remova a herança de `Node` da classe `Odom` e a inicialização do nó, `super().__init__('second_node')`.

!!! info
    Estamos removendo a herança para que você possa reutilizar a classe em qualquer nó, criando um módulo, o que não seria possível se `Odom` herda-se de `Node`.

2. Remova a função `control()` da classe `Odom` e o timer que chama essa função.

3. Remova a função `main()` e a condição `if __name__ == '__main__':`.

4. Inicialize uma variável `self.opening` no construtor `__init__`, que será utilizada para armazenar a abertura do sensor laser, ao mudar esse valor, podemos modificar o que consideramos como "frente" do robô, por exemplo.

5. Defina um subscriber para o tópico `/scan` que chama a função `laser_callback` quando uma mensagem é recebida. O subscriber terá o seguinte formato, a diferença da `Odometry` é que para o robô real é necessário alterar `reliability` para `BEST_EFFORT`, por limitações de hardware.

```python
self.laser_sub = self.create_subscription(
    LaserScan,
    '/scan',
    self.laser_callback,
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
```

5. Definir uma função `laser_callback` que recebe uma mensagem do tipo `sensor_msgs/msg/LaserScan` e armazena os seguintes parâmetros:

    5.1. Utilize o seguinte comando para converter a lista em um array numpy:

    ```python
    self.laser_msg = np.array(msg.ranges).round(decimals=2)
    ```

    5.2. Utilize o seguinte comando jogar os valores `0` para `inf`, removendo ambiguidades, entre o que o robo real e o robo simulado entende por "fora de alcance".

    ```python
    self.laser_msg[self.laser_msg == 0] = np.inf
    ```
    
    5.3. Converta `self.laser_msg` para uma lista novamente.

    5.4. Faça um fatiamento na lista `self.laser_msg` com os +- `self.opening` valores na frente do robô e armazene na variável `self.front`.

    5.5. Faça um fatiamento na lista `self.laser_msg` com os +- `self.opening` valores na esquerda do robô e armazene na variável `self.left`.

    5.6. Faça um fatiamento na lista `self.laser_msg` com os +- `self.opening` valores na direita do robô e armazene na variável `self.right`.

    5.7. Faça um fatiamento na lista `self.laser_msg` com os +- `self.opening` valores atrás do robô e armazene na variável `self.back`.

### Testando

Para testar, baseado-se no arquivo `base.py` crie um arquivo chamado `test_laser.py`, dentro do pacote `robcomp_util`. Este arquivo deve conter um nó chamado `test_laser_node` que importa a classe `Laser` do arquivo `laser.py` e imprime as leituras do laser a cada **1 segundo**.

Lembre-se:

* Importe a classe `Laser` da seguinte forma:
```python
from robcomp_util.laser import Laser
```

* Faça a herança da classe `Laser` no `test_laser_node`.

* Adicione o nó no arquivo `setup.py` e então compile o pacote.

* Rode o nó `test_laser_node` utilizando o comando `ros2 run robcomp_util test_laser` e mova o robô utilizando o teleop, para ver como o laser é atualizado.