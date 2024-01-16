# Exercícios ROS

# Q5 - Image Subscriber
Em projetos mais complexos da ROS, separar o código em módulos é uma boa prática para manter o projeto organizado e bem estruturado. Um módulo muito utilizado seria o módulo da visão, onde todos os processamentos de imagem são executados por um nó, que publica apenas as informações relevantes da imagem, como por exemplo, coordenadas do alvo.
Neste exercício vamos trabalhar no arquivo `image_publisher.py`. O objetivo é criar um nó da ROS que:

1. Se inscreva no tópico `/camera/image/compressed`.
2. Utilizando métodos de visão computacional, modifique a função `color_segmentation` para encontrar o centro do "creeper" azul.
3. Publique uma imagem com um *crosshair* no centro do “creeper” para o tópico `image_publisher` do tipo `sensor_msgs/Image`
4. Publique os valores de `x` e `y` do centro do “creeper” em um tópico `center_publisher` do tipo `geometry_msgs/Point`. Publique também a largura / 2 da imagem no valor `z`.
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

    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
```

Primeiramente na função `__init__` definimos a variável `self.bridge`. Os tópicos da ROS trabalham com imagens codificadas, então na linha `6`, utilizamos essa variável para converter a imagem para a estrutura do OpenCV. Por fim, depois da imagem ser processada, na linha `12`, ela é codificada novamente e publicada em outro tópico.

??? details "Resposta"
    [Resposta](../modulo4/scripts_resp/image_publisher.py){ .ah-button }
