# Exercícios Extra ROS

## Antes de começar
Adicione o arquivo [goto.py](../modulo4/scripts/goto.py) para o diretorio `~/catkin_ws/src/modulo4/scripts` e depois execute o comando:

```bash
roscd modulo4
cd scripts
chmod +x *.py
```

## Go To
Começando do arquivo `goto.py` complete as partes do código com ??? para que o código funcione sem erros. O nó deve mover o robô para o ponto `self.point`, enviado como parâmetro da classe GoTo, utilizando controle proporcional. O robô deve parar quando estiver a um raio de 0.1m do ponto desejado.

Execute o codigo no simulador e então grave um video mostrando o robo real realizando o movimento.

Primeiramente entenda o a função `odom_callback`, ela recebe a mensagem do tópico `odom` e salva a posição atual do robô em `self.x`, `self.y` e `self.yaw`, que está no intervalo de -pi a pi.

* A função `get_angular_error` calcula a distancia e o erro angular entre o robô e o ponto desejado.

* O estado `center` rotaciona o robô para o ponto desejado. Quando o erro angular for menor que 5 graus, o estado muda para `goto`. Atualize o erro chamando a função `get_angular_error` a cada iteração.

* O estado `goto` move o robô para o ponto desejado. Quando a distancia for menor que 0.1m, o estado muda para `stop`. Atualize o erro chamando a função `get_angular_error` a cada iteração.

* O estado `stop` para o robô.

**DICA** - Para calcular o erro angular entre o robô e o ponto desejado, utilize a função `atan2` do python, que retorna o ângulo entre dois pontos em radianos. Depois normalize o ângulo para o intervalo de -pi a pi utilizando `atan2` novamente, da seguinte forma:

```python
    self.err = theta - self.yaw
    self.err = np.arctan2(np.sin(self.err), np.cos(self.err))  # Normaliza o erro para o intervalo [-π, π]












??? details "Resposta"
    [Resposta](../modulo4/scripts_resp/image_publisher.py){ .ah-button }
