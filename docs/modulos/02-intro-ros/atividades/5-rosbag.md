# Salvando Eventos na ROS 2

Nesta atividade vamos aprender a **gravar** e **reproduzir** acontecimentos na ROS 2 usando o pacote `rosbag2`, a ferramenta oficial para captura e replay de mensagens.

Um **ROS bag** pode armazenar mensagens de **tópicos**, além de informações de serviços, parâmetros, ações e diagnósticos. Gravações são úteis para gerar conjuntos de dados de testes e para **reproduzir** cenários sem precisar do robô/simulador em tempo real.

## Parte 1 - Gravando mensagens

Primeiro, abra a simulação do robô, o `teleop` e o `rqt_image_view` (cada comando em um terminal diferente):

* **Simulador**

```bash
ros2 launch my_gazebo pista-23B.launch.py
```

* **Teleop**

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

* **Visualização de imagem**

```bash
ros2 run rqt_image_view rqt_image_view
```

Para **gravar** mensagens em um bag, use:

```bash
ros2 bag record /topic1 /topic2 -o nome_do_meu_bag
```

Aqui `/topic1` e `/topic2` são os tópicos a gravar e `nome_do_meu_bag` é o diretório de saída do bag.

!!!tip
    Recomendamos que você grave apenas um tópico de imagem por vez, pois a gravação de vários tópicos de imagem pode consumir muita memória e resultar em um arquivo de `ROS BAG` muito grande.

No nosso caso, vamos gravar a câmera e o laser:

```bash
ros2 bag record /camera/image_raw /scan -o my_bag
```

Agora movimente o robô com o `teleop` e acompanhe a câmera no `rqt_image_view`.

Para **parar** a gravação, pressione `Ctrl+C`. Um diretório chamado `my_bag` será criado no local atual. Verifique com:

```bash
ls
```

Você pode inspecionar o conteúdo do bag com:

```bash
ros2 bag info my_bag
```

## Parte 2 - Visualizando mensagens

Com o bag criado, vamos **reproduzi‑lo**.

1. Feche os terminais do **simulador** e do **teleop** (para evitar conflitos com publishers reais).

2. Em um novo terminal, reproduza o bag:

```bash
ros2 bag play my_bag
```

Abra o `rqt_image_view` para visualizar os frames reproduzidos.

**Dicas de execução:**

Com o foco no terminal que está rodando o `bag`, você pode:

* Parar **pausar/continuar** a reprodução, pressione `barra de espaço`.

* **Acelerar** a reprodução pressionando `seta para direita` (rate+0.1).

* **Desacelerar** a reprodução pressionando `seta para esquerda` (rate-0.1).

Com isso você já está pronto para desenvolver a **APS 2**.
