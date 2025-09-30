# Detecção de Objetos Complexos com Redes Neurais

Nesta semana vamos trabalhar com um assunto extremamente atual: reconhecimento de objetos e rastreamento de objetos utilizando redes neurais. 

Para isso, vamos utilizar a família YOLO (You Only Look Once), em particular, o YOLOv8, que oferece boa precisão com baixa latência.

A arquitetura da **YOLO** é otimizada para consumir pouca memória e processamento, sendo ideal para dispositivos com poucos recursos computacionais, como smartphones, tablets, Raspberry Pi, etc.

## Por que YOLOv8?

Rápido e leve: projetado para rodar bem em CPU e GPU, inclusive em dispositivos modestos (Raspberry Pi, notebooks comuns).

Modelos em tamanhos diferentes: do nano (yolov8n) ao xlarge (yolov8x). Quanto maior, mais preciso (em geral), porém mais pesado.

Pronto para usar: já vem com pesos pré-treinados em classes comuns (COCO), então você consegue resultados rápidos sem treinar nada.

## Utilizando o YOLOv8 no Robô

Para reduzir a carga do no seu computador, a YOLOv8 já está instalada dentro do robô, mas se mantêm desligada. Você pode ligar a detecção de objetos no robô, mas lembre-se que isso pode afetar o desempenho do robô, por esse motivo, sugerimos que ligue apenas quando for utilizá-la.
Para ligar a detecção de objetos, basta publicar uma mensagem do tipo `std_msgs/msg/Bool` com o valor true no topico `/poweron_yolo`.

Depois, para visualizar a saída da detecção de objetos, voce pode se inscrever no tópico `/yolo_info`, que publica mensagens do tipo `robcomp_interfaces/YoloArray`.

Agora vamos ver qual é a estrutura da mensagem `robcomp_interfaces/YoloArray`:

```
YoloDetector[] yolos
	string classe
	float32[4] boxes
	float32 score
```

### Como interpretar a saída

Para cada objeto detectado, dentro do parametro `yolos`, você terá uma mensagem do tipo `YoloDetector`, que contém as seguintes informações:

* classe (ex.: person, car)
* boxes: x, y, largura, altura (coordenadas do retângulo)
* score (0–1): confiança, quanto maior, mais confiante o modelo está na detecção.

## Prática 1
Agora vamos praticar o uso da YOLOv8.
Baseado no código do arquivo [image_subscriber.py](/docs/modulos/06-visao-p3/util/image_subscriber.py), crie um arquivo chamado `perigo.py` com uma classe chamada `DetectorPerigo`, com um nó chamado `detector_perigo_node` que se inscreva no tópico `/yolo_info` e:

1. Desenhe os retângulos ao redor dos objetos detectados na imagem.
2. Escreva a classe acima de cada retângulo.
3. Faça o robô girar em velocidade constante sem se deslocar (girar no próprio eixo).
4. Enquanto o robô gira, se o robô detectar uma vaca perto de um lobo, deve escrever em vermelho "Cuidado! Vaca em Perigo!" na imagem.

### Video
Grave um vídeo do seu robô realizando a atividade, de upload no YouTube e adicione o link no arquivo README.md do seu repositório.


