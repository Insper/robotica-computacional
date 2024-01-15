# Q3 - Robô quadrado (Dead reckoning)
Usando o simulador, modifique o arquivo `quadrado.py` para criar um nó da ROS que faça o robô se mova em uma trajetória que se ***aproxima*** de um quadrado.

**DICA 1** - Para fazer o robô se mover, publique uma mensagem para o tópico `cmd_vel`, verifique o tipo de mensagem que este tópico recebe utilizando o comando `rostopic type cmd_vel`.

**DICA 2** - Você pode esperar `n` segundos usando `rospy.sleep(n)`. Dessa forma, assumindo que o robô está se deslocando/rotacionando com velocidade constante, é possível prever sua posição final. Este tipo de controle se chama "dead reckoning". É uma forma simples de se controlar o robô, mas tem a desvantagem de "travar" o código, deixando o roubo menos reativo.

??? details "Resposta"
    [Resposta](../modulo4/scripts_resp/quadrado.py){ .ah-button }

# Q4 - Robô Quase Indeciso
Usando o simulador e o Laser simulado, modifique o arquivo `indeciso.py`, faça com que o robô se afaste da parede quando o obstáculo à sua frente estiver a menos de `0.95m` e se aproximar quando estiver a mais de `1.05m`, caso contrário, o robô deve ficar parado. Portanto o robô deve parar eventualmente.

**DICA** - Recorte a mensagem do laser para observar um limiar de &plusmn;5&deg; e avalie com base no **menor valor desse limiar**.

Deixamos no arquivo a função que recebe os dados do Laser.
```python
def laser_callback(self, msg: LaserScan) -> None:
    self.laser_msg = np.array(msg.ranges).round(decimals=2)
    self.laser_msg[self.laser_msg == 0] = np.inf
```
Esta função será chamada sempre que uma mensagem for publicada no tópico `/scan`. O conteúdo da mensagem será convertido para np.array e o valor é arredondado para 2 casas decimais. 

Como no robô simulado um objeto muito longe tem o valor `np.inf` e no robô real tem o valor `0`. a segunda linha do `laser_callback` serve para fazer a padronização dos valores.

??? details "Resposta"
    [Resposta](../modulo4/scripts_resp/indeciso.py){ .ah-button }
