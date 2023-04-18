# Exercícios ROS
No arquivo `base.py` fornecemos uma estrutura básica de um script da ROS usando classe no python.

A função `main.py` cria um nó com o nome `'Controler'` e construir um a classe `Control()`, depois até que o `core` da ROS seja interrompido vai executar o conteúdo da função `Control().control` em loop.

```python
def main():
	rospy.init_node('Controler')
	control = Control()
	rospy.sleep(1) # Espera 1 segundo para que os publishers e subscribers sejam criados

	while not rospy.is_shutdown():
		control.control()
```

Para fazer com que o nó se inscreva em um tópico deve-se primeiro pegar o tipo de mensagem e então comando com a seguinte estrutura de comando pode se inscrever no tópico:
```python
self.sub = rospy.Subscriber({topico},{tipo de mensagem},self.callback)
```

Sempre que uma nova mensagem seja enviada ao tópico “topico”, a função `self.callback` será executada com esta nova mensagem.

Para fazer com que um nó publique em um tópico, utilize a seguinte estrutura de comando: 

```python
self.pub = rospy.Publisher({topico},{tipo de mensagem},queue_size=10)
```

O argumento `queue_size` indica o tamanho máximo da fila de mensagens. Em condições normais a fila não ultrapassa o tamanho 1, este argumento só é relevante quando o conteúdo da mensagem é muito extenso, como uma imagem.

## Antes de começar
Após extrair os arquivos e rodar `catkin_make`, para rodar os exercícios, execute os seguintes comandos no terminal:

```bash
roscd modulo4
cd scripts
chmod +x *.py
```

## Q1 - Publisher
Começando do arquivo `publisher.py` complete as partes do código com ??? para que o código funcione sem erros. O nó deve publicar uma mensagem no tópico `publisher` do tipo `std_msgs/String` contendo o horário atual e um o número da mensagem enviada, **separadas por um espaço**. Também deve imprimir no terminal uma alerta utilizando o comando `rospy.loginfo` com a seguinte estrutura:

```bash
[INFO] [1677878366.175759]: Ola, são 1677878366175707817 e estou publicando pela 117 vez
```

Utilize o comando `rostopic echo publisher` para verificar se o exercício está correto.

**DICA 1** - Para pegar o horário atual
```python 
tempo = rospy.Time.now()
tempo_sec = rospy.Time.now().to_sec()
```
??? details "Resposta"
    [Resposta](modulo4/scripts_resp/publisher.py){ .ah-button }

# Q2 - Subscriber
Agora vamos trabalhar em um nó que se inscreve no tópico que criamos no exercício anterior. A função `callback`, deve separar o tempo do contador no conteúdo da mensagem,lembre-se de checar a estrutura da mensagem. A função `control` deve calcular o tempo que passou e utilizar o comando `rospy.loginfo` para mostrar o número da mensagem e o delay dela no terminal, como no exemplo a seguir,

```bash
[INFO] [1677878948.424955]: Ola, estou recebendo a mensagem: 217 e se passaram 0.005347013 segundos
```

**DICA 1** - Pode carregar um valor float referente a um tempo da seguinte forma:
```python 
time = rospy.Time( float( rospy.Time.now().to_sec() ) )
```

!!! exercise long 
    Qual a estrutura da mensagem do tipo `String`?

    !!! answer
        `string data`. O conteúdo da mensagem é armazenado na variável `data`. Então para acessar o conteúdo, deve-se utilizar `msg.data`. Depois pode separar o tempo do contador utilizando o comando `msg.data.split()`.

??? details "Resposta"
    [Resposta](modulo4/scripts_resp/subscriber.py){ .ah-button }

# Q3 - Robô quadrado (Deadlock)
Usando o simulador, modifique o arquivo `quadrado.py` para criar um nó da ROS que faça o robô se mova em uma trajetória que se ***aproxima*** de um quadrado.

**DICA 1** - Para fazer o robô se mover, publique uma mensagem para o tópico `cmd_vel`, verifique o tipo de mensagem que este tópico recebe utilizando o comando `rostopic type cmd_vel`.

**DICA 2** - Você pode esperar `n` segundos usando `rospy.sleep(n)`. Dessa forma, assumindo que o robô está se deslocando/rotacionando com velocidade constante, é possível prever sua posição final. Este tipo de controle se chama "Deadlock". É uma forma simples de se controlar o robô, mas tem a desvantagem de "travar" o código, deixando o roubo menos reativo.

# Q4 Robô Quase Indeciso
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

# Q5 Image Subscriber
Em projetos mais complexos da ROS, separar o código em módulos é uma boa prática para manter o projeto organizado e bem estruturado. Um módulo muito utilizado seria o módulo da visão, onde todos os processamentos de imagem são executados por um nó, que publica apenas as informações relevantes da imagem, como por exemplo, coordenadas do alvo.
Neste exercício vamos trabalhar no arquivo `image_publisher`. O objetivo é criar um nó da ROS que:

1. Se inscreva no tópico `/camera/image/compressed`.
2. Utilizando métodos de visão computacional, modifique a função `color_segmentation` para encontrar o centro do "creeper" azul.
3. Publique uma imagem com um *crosshair* no centro do “creeper” para o tópico `image_publisher`
4. Publique os valores de `x` e `y` do centro do “creeper” em um tópico `center_publisher` do tipo `geometry_msgs/Point`
5. No caso de não haver um "creeper" azul no frame, deve publicar `x=-1` e `y=-1`.
6. Na função `control` utilize o comando `rospy.loginfo` para mostrar no terminal as coordenadas do centro do “creeper”, ou alertar que não existem “creepers” na imagem.

**DICA** - Utilize o `rqt_image_view` para capturar uma imagem do “creeper” e descubra os limiares do filtro HSV para esta imagem.

**SUGESTÃO** - Durante o curso vamos utilizar também os “creepers” de outras cores, presentes neste cenário, pode ser vantajoso preparar a segmentação para eles também.

Deixamos no arquivo a função que recebe os dados da imagem, faltando apenas o tipo da imagem.

```python linenums="1"
def laser_callback(self, msg: LaserScan) -> None:
    """
    Callback function for the image topic
    """
    try:
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    self.color_segmentation(cv_image) # Processamento da imagem

    self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image))
```

Primeiramente na função `__init__` definimos a variável `self.bridge`. Os tópicos da ROS trabalham com imagens codificadas, então na linha `6`, utilizamos essa variável para converter a imagem para a estrutura do OpenCV. Por fim, depois da imagem ser processada, na linha `12`, ela é codificada novamente e publicada em outro tópico.