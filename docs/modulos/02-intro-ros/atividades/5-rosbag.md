# Salvando Eventos na ROS 2

Nesta atividade vamos aprender como gravar acontecimentos na ROS 2. Para isso, vamos utilizar o pacote `rosbag2` que é uma ferramenta para gravação e reprodução de mensagens da ROS 2.

Em um `ROS BAG` é possível gravar mensagens de tópicos, serviços, parâmetros, ações e eventos de diagnóstico. A gravação de mensagens é útil para a geração de conjuntos de dados para testes. Quando você grava mensagens em um `ROS BAG`, você pode reproduzi-las como se estivesse recebendo mensagens em tempo real.

## Parte 1 - Gravando mensagens

Primeiro, abra a simulação do robô, o `teleop` e o `rqt_image_view`, utilizando os seguintes comandos:

* Mapa do robô:

```bash
ros2 launch my_gazebo pista-23B.launch.py
```

* Teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

* Rqt_image_view:

```bash
ros2 run rqt_image_view rqt_image_view
```


- Para `gravar` mensagens em um `ROS BAG`, execute o seguinte comando:

```bash
ros2 bag record /topic1 /topic2 -o nome_do_meu_bag
```

Onde `/topic1` e `/topic2` são os tópicos que você deseja gravar e `nome_do_meu_bag` é o nome do arquivo do `ROS BAG`.

!!!tip
    Recomendamos que você grave apenas um tópico de imagem por vez, pois a gravação de vários tópicos de imagem pode consumir muita memória e resultar em um arquivo de `ROS BAG` muito grande.

No nosso caso, vamos gravar o tópico de imagem da câmera e do laser, para isso, execute o seguinte comando:

```bash
ros2 bag record /camera/image_raw /scan -o my_bag
```

- Agora, movimente o robô com o `teleop` um pouco e visualize a camera no `rqt_image_view`.

- Pare a gravação do `ROS BAG` com `Ctrl+C`, isso vai gerar um arquivo chamado `my_bag` no diretório atual.

- Digite `ls` no terminal para verificar a criação do arquivo.

## Parte 2 - Visualizando mensagens

Agora que temos uma RosBag criado, vamos executar e vericar o seu funcionamento. 

- Primeiro feche todos os terminais que estão rodando a simulação do robô. Agora, para visualizar as mensagens gravadas no `ROS BAG`, abra um novo terminal e execute o seguinte comando:

```bash
ros2 bag play my_bag
```

Esse comando irá reproduzir as mensagens gravadas no `ROS BAG` e pausar a reprodução. Para visualizar corretamente abra o `rqt_image_view`.

Com o foco no terminal que está rodando o `bag`, você pode:

* Parar **pausar/continuar** a reprodução, pressione `barra de espaço`.

* **Acelerar** a reprodução pressionando `seta para direita` (rate+0.1).

* **Desacelerar** a reprodução pressionando `seta para esquerda` (rate-0.1).

Com isso você já está pronto para desenvolver a APS 2.