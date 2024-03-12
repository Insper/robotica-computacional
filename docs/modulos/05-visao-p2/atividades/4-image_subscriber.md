# Processando Imagens na ROS

Nesta atividade, vamos aprender a processar imagens na ROS. Para isso, vamos utilizar a biblioteca OpenCV, que é uma das mais utilizadas para processamento de imagens.

No arquivo ![base_visao.py](util/image_subscriber.py), você encontrará um código que se inscreve nó tópico de imagem da câmera do robô. Utilize este código como base para realizar as atividades do curso.

## Explicação do Código
Essencialmente o código é o mesmo que já vimos em aulas anteriores, no código abaixo,

```python
        self.image_sub = self.create_subscription(
            Image, # or CompressedImage
            '/camera/image_raw', # or '/camera/image_raw/compressed'
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
```
 principais pontos a serem observados são:

* Podemos escolher entre `Image` e `CompressedImage` para receber a imagem. A diferença entre eles é que `Image` é uma mensagem que contém a imagem em formato de matriz de pixels, enquanto `CompressedImage` é uma mensagem que contém a imagem comprimida.

* Se você escolher `Image`, o tópico que você deve se inscrever é `/camera/image_raw`. 

* Se você escolher `CompressedImage`, o tópico que você deve se inscrever é `/camera/image_raw/compressed`.

No método `image_callback`, vamos processar imagem recebida.

```python
    def image_callback(self, msg):
        if self.runnable:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # if Image
            # cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage
            
            # Faça aqui o processamento da imagem
            # ou chame uma classe ou função que faça isso
        else:
            print('Image processing is paused')
```

Neste método, a imagem recebida é convertida para um objeto do tipo `numpy.ndarray` utilizando a função `imgmsg_to_cv2` ou `compressed_imgmsg_to_cv2` da biblioteca `cv_bridge`. No `__init__` do código, definimos a variável `self.bridge` como `CvBridge()`, essa biblioteca é uma ponte (bridge) entre a biblioteca OpenCV e a ROS, que trabalham com formatos de imagem diferentes.

Outro ponto importante é a variável `self.runnable`, obtida do tópico `/vision/image_flag`. Enquanto o valor dessa variável for `True`, o processamento da imagem é feito, então, um outro nó pode publicar nesse tópico para pausar ou retomar o processamento da imagem. Isso é útil para economizar recursos do robô quando o processamento da imagem não é necessário, será útil para quando estivermos processando imagens usando Redes Neurais ou Aruco.

Por ultimo, ainda no método `image_callback`, a variável `cv_image` contém a imagem recebida. Neste ponto, você pode fazer o processamento da imagem ou chamar uma função ou classe que faça isso para manter o código organizado.
