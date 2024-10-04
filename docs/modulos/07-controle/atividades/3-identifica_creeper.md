# Identificando os Creeper
Nesta atividade, vamos criar um nó que identifica os Creeper e publica as detecções sincronizadas. Identificar um Creeper significa combinarmos a detecção de cor, através de segmentação de cores, com a detecção de ID, através da leitura do marcador ArUco.

## Identificação Offline
Para facilitar o desenvolvimento, vamos começar criando uma classe que identifica os Creeper através de uma imagem. Para desenvolver essa classe, complete as intruções nos comentários do código [creeper_detector.py](../util/creeper_detector.py).

Uma vez funcional, mova o arquivo `creeper_detector.py` para o pacote `robcomp_interfaces` e compile novamente o pacote.

## Identificação Online (Publisher)
Agora que temos a classe que identifica os Creeper, vamos criar um nó que publica as detecções sincronizadas. 

Com base no código `image_subscriber.py` do capítulo 5, crie um arquivo chamado `creeper_pub.py` que contenha uma classe chamada `CreeperPublisher` com um nó denominado `creeper_detector_node`. Este nó deve:

- Inscrever-se no tópico de imagens.
- Herdar a classe `CreeperDetector` do pacote `robcomp_interfaces`.
- Publicar, no tópico `/creeper`, utilizando o formato de mensagem `robcomp_interfaces/msg/DetectionArray` onde, para cada detecção, a **classe** deve ser `{cor}-{id}` e **cx** seria o erro do centro do corpo do creeper e o centro da imagem.
- Ouvir o tópico `/vision/creeper_flag` e, ao receber uma mensagem `False`, interromper o processamento de imagens.
