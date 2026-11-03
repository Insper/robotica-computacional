# Robótica Computacional - Simulado AF 2

## Atualização dos Pacote (ROS 2)

Execute os comandos abaixo para atualizar os pacotes da `ros2` obrigatórios para a prova:

```bash
cd ~/colcon_ws/src/my_simulation
git stash
git pull
cd ~/colcon_ws/src/insperbot
git pull
cb
```

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** O repositório é informado no Blackboard (veja [Sobre as APS](https://insper.github.io/robotica-computacional/aps/)). Clone-o **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote chamado `simulado_2`.

    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `my_package` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

___________________________

# Exercício 1

Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `q1.py` contendo uma classe denominada `Circuito`. Esta classe deve implementar um **nó** chamado `circuito_node`, responsável por fazer com que o robô **simulado** siga a linha amarela, ao completar uma volta, gire 90 graus e vá até a caixa da cor passada na criação da classe `Circuito`.

Utilize o comando abaixo para iniciar o simulador no mapa da prova:

```bash
ros2 launch my_gazebo circuito.launch.py
```

![circuito](circuito.png)

O nó criado deve: 

1. Dar uma volta completa na pista, parando na posição inicial
2. Girar 90 graus e parar olhando na direção das caixas.
3. Centralizar na caixa da cor passada na criação da classe `Circuito`
4. vai até a caixa da cor selecionada, parando quando estiver a bem próxima
5. volta para posição inicial e para

## Restrições

1. Deve existir o arquivo chamado `q1.py`.
2. O programa deve ser executado sem erros.
3. A classe deve se chamar `Circuito`.
4. A implementação deve seguir a estrutura da classe `Circuito`, conforme exemplo no `base_control.py`.
5. A função `control` deve ser a única a publicar no tópico `/cmd_vel`.
6. A função `control` deve ser idêntica à do arquivo `base_control.py`. Todas as decisões de controle devem ocorrer dentro dos nós, sem alterações na função `control`.
7. Não utilizar loops infinitos ou `sleep` durante o controle do robô.

___________________________

# Exercício 2

Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `q2.py` contendo uma classe denominada `Aleatorio`. Esta classe deve implementar um **nó** chamado `aleatorio_node`, responsável por fazer com que o robô **simulado** gire a uma velocidade de 0.5 rad/s por um tempo aleatório entre 5 e 10 segundos, e então realizar um comportamento diferente dependendo da cor da caixa mais prevalecente no campo de visão do robô.

Utilize o comando abaixo para iniciar o simulador no mapa da prova:

```bash
ros2 launch my_gazebo caixas.launch.py
```

![caixas](caixas.png)

O nó criado deve: 

1. Girar a uma velocidade de 0.5 rad/s por um tempo aleatório entre 5 e 10 segundos
2. Parar de girar.
3. Identificar a cor da caixa mais prevalecente no campo de visão do robô
4. Realizar um comportamento diferente dependendo da cor da caixa mais prevalecente:
    * **Magenta:** Se aproximar da caixa enquanto centraliza a caixa no campo de visão
    * **Amarela:** Se afastar da caixa enquanto centraliza a caixa no campo de visão
5. Voltar para posição inicial e parar

## Restrições

1. Deve existir o arquivo chamado `q2.py`.
2. O programa deve ser executado sem erros.
3. A classe deve se chamar `Aleatorio`.
4. A implementação deve seguir a estrutura da classe `Aleatorio`, conforme exemplo no `base_control.py`.
5. A função `control` deve ser a única a publicar no tópico `/cmd_vel`.
6. A função `control` deve ser idêntica à do arquivo `base_control.py`. Todas as decisões de controle devem ocorrer dentro dos nós, sem alterações na função `control`.
7. Não utilizar loops infinitos ou `sleep` durante o controle do robô.

___

# Extra

O robô deve seguir a linha no sentido da imagem abaixo, e ao completar uma volta, deve mudar de pista até completar uma volta em todas as pistas e parar.

```bash
ros2 launch my_gazebo pista_circulos.launch.py
```

![pistas](pistas.png)
