# Exercícios Extra ROS

## Antes de começar
Adicione o arquivo [goto.py](../modulo4/scripts/goto.py) para o diretório `~/catkin_ws/src/modulo4/scripts` e depois execute o comando:

```bash
roscd modulo4
cd scripts
chmod +x *.py
```
## Q1 - Máquina de Estados - Robô Quase Indeciso
Modifique o arquivo `indeciso.py` para uma implementação de uma máquina de estados.

* O estado `aproxima` faz o robô se aproximar da parede quando o obstáculo à sua frente estiver a mais de `1.05m`. Quando o obstáculo estiver a menos de `0.95m`, o estado muda para `afasta`.

* O estado `afasta` faz o robô se afastar da parede quando o obstáculo à sua frente estiver a menos de `0.95m`.

* Se o obstáculo estiver entre `0.95m` e `1.05m`, o robô deve entrar no estado `parado`, que para o robô.

## Q2 - Go To
Começando do arquivo `goto.py` complete as partes do código com ??? para que o código funcione sem erros. O nó deve mover o robô para o ponto `self.point`, enviado como parâmetro da classe GoTo, utilizando controle proporcional. O robô deve parar quando estiver a um raio de 0.1m do ponto desejado.

Primeiramente entenda o a função `odom_callback`, ela recebe a mensagem do tópico `odom` e salva a posição atual do robô em `self.x`, `self.y` e `self.yaw`, que está no intervalo de -pi a pi.

* A função `get_angular_error` calcula a distância e o erro angular entre o robô e o ponto desejado.

* O estado `center` rotaciona o robô para o ponto desejado. Quando o erro angular for menor que 5 graus, o estado muda para `goto`. Atualize o erro chamando a função `get_angular_error` a cada iteração.

* O estado `goto` move o robô para o ponto desejado. Quando a distância for menor que 0.1m, o estado muda para `stop`. Atualize o erro chamando a função `get_angular_error` a cada iteração.

* O estado `stop` para o robô.

**DICA** - Para calcular o erro angular entre o robô e o ponto desejado, utilize a função `atan2` do python, que retorna o ângulo entre dois pontos em radianos. Depois normalize o ângulo para o intervalo de -pi a pi utilizando `atan2` novamente, da seguinte forma:

```python
    self.err = theta - self.yaw
    self.err = np.arctan2(np.sin(self.err), np.cos(self.err))  # Normaliza o erro para o intervalo [-π, π]
```

## Q3 - Robô quadrado (Controle Proporcional)
Modifique o arquivo `quadrado.py` para criar um nó da ROS que faça o robô se mova em um quadrado utilizando controle proporcional.

* Depois teste seu código no robô real.

**DICA** - Em um estado, chame a classe `GoTo` para mover o robô para um ponto desejado. Quando o robô chegar no ponto desejado, mude o estado para o próximo ponto.