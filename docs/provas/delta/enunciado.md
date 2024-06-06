# Robótica Computacional 2024.1 - Prova Delta

Observações de avaliações nesta disciplina:

* A prova tem duração de **3 horas**.
* Inicie a prova no Blackboard para a ferramenta de Proctoring iniciar. Só finalize o Blackboard quando enviar a prova via Github classroom.
* Durante a prova vamos registrar as páginas visitadas e acessos online e o teclado.
* Coloque seu nome e email no README.md do seu repositório.
* Você pode consultar a internet ou qualquer material que utilizado no curso, mas não pode se comunicar com pessoas ou colegas a respeito da prova *em qualquer plataforma*.
* `Não é permitido o uso de ferramentas de **IA** como chatGPT, Copilot ou similares durante a prova`.
* Faça commits e pushes frequentes no seu repositório.
* Eventuais avisos importantes serão realizados em sala durante a prova.
* A responsabilidade por *infraestrutura*, *configuração* e *setup* em funcionamento pleno, é de cada estudante.
* Elementos da rúbrica com `(chkp)` podem ser utilizados como comprovação de nota parcial, caso o exercício não esteja completo. Para a nota completa, pode ignorar esses elementos.

## Atualização dos Pacote (ROS 2)

Execute os comandos abaixo para atualizar os pacotes da `ros2` obrigatórios para a prova:

```bash
cd ~/colcon_ws/src/my_simulation
git stash
git pull
cd ~/colcon_ws/src/insperbot
git stash
git pull
cb
```

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.

- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote chamado `avaliacao_ai`.

    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `my_package` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

Escreva a letra B como a resposta da soma no arquivo `README.md` como teste de sua atenção.
___________________________

# Exercício 1 (5)

Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `q1.py` contendo uma classe denominada `SegueComando`. Esta classe deve implementar um **nó** chamado `segue_comando_node`, responsável por fazer com que o robô **simulado** siga a linha amarela, até completar uma volta e parar. O robô deve então publicar no tópico `/animal` a string `completei` e esperar uma resposta no tópico `/animal` contendo um animal. O robô deve então continuar seguindo a linha amarela até chegar na linha da cor correspondente ao animal e seguir a linha parando bem próximo da caixa correspondente ao animal recebido.

Utilize o comando abaixo para iniciar o simulador no mapa da prova:

```bash
ros2 launch my_gazebo retangulos.launch.py
```

![retangulos](retangulos.png)

O nó criado deve: 

1. Se increver no tópico `/animal` (String).
2. Segue a linha amarela completando uma volta e parando.
3. Publica no tópico `/animal` a string `completei`.
4. O robô espera uma resposta no tópico `/animal` contendo um animal.
3. Continua seguindo a linha amarela até chegar na linha da cor correspondente ao animal recebido:
    * **Cachorro:** linha magenta (superior esquerda)
    * **Gato:** linha verde (superior direita)
    * **Cavalo:** linha azul (inferior direita)
4. Assim que passar bem próximo da caixa correspondente ao animal recebido, o robô para.

## Restrições

1. Deve existir o arquivo chamado `q1.py`.
2. O programa deve ser executado sem erros.
3. A classe deve se chamar `SegueComando`.
4. A implementação deve seguir a estrutura da classe `SegueComando`, conforme exemplo no `base_control.py`.
5. A função `control` deve ser a única a publicar no tópico `/cmd_vel`.
6. A função `control` deve ser idêntica à do arquivo `base_control.py`. Todas as decisões de controle devem ocorrer dentro dos nós, sem alterações na função `control`.
7. Não utilizar loops infinitos ou `sleep` durante o controle do robô.

## Rúbrica

1. +1,0 - O robô consegue seguir a linha amarela completando uma volta e parando.
2. +1,0 - O robô publica no tópico `/animal` a string `completei`.
3. +1,0 - O robô se inscreve no tópico `/animal` e recebe o comando contendo um animal.
4. +1,0 - O robô muda de para a pista da cor correspondente.
5. +1,0 - O robô para bem próximo da caixa correspondente ao animal recebido.

## Vídeo

Grave um vídeos, mostrando um terminal echoando o tópico `/animal` e mostrando que o robô é capaz de realizar o comportamento completo ou algum comportamento parcial. Publique os vídeos no YouTube e inclua apenas o `link` no arquivo `README.md` do seu repositório.
___________________________

# Exercício 2 (5)

Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `q2.py` contendo uma classe denominada `MudaPista`. Esta classe deve implementar um **nó** chamado `muda_pista_node`, responsável por fazer com que o robô **simulado** entre na linha vermelha e assim que passar pela primeira linha verde, mude de pista e siga a linha verde. O robô deve então seguir a linha verde até passar pela primeira linha azul, mudar de pista e seguir a linha azul, completando uma volta completa e parando. O robô deve navegar no sentido demarcado na imagem abaixo.

![pistas](pistas.png)

Utilize o comando abaixo para iniciar o simulador no mapa da prova:
```bash
ros2 launch my_gazebo pista_circulos.launch.py
```

O nó criado deve: 

1. Entrar na linha vermelha e seguir no sentido horário.
2. Assim que passar pela primeira linha verde, mudar de pista e seguir a linha verde no sentido horário.
3. Assim que passar pela primeira linha azul, mudar de pista e seguir a linha azul no sentido horário.
4. Dar uma volta `completa` na nova pista azul e parar.

## Restrições

1. Deve existir o arquivo chamado `q2.py`.
2. O programa deve ser executado sem erros.
3. A classe deve se chamar `MudaPista`.
4. A implementação deve seguir a estrutura da classe `MudaPista`, conforme exemplo no `base_control.py`.
5. A função `control` deve ser a única a publicar no tópico `/cmd_vel`.
6. A função `control` deve ser idêntica à do arquivo `base_control.py`. Todas as decisões de controle devem ocorrer dentro dos nós, sem alterações na função `control`.
7. Não utilizar loops infinitos ou `sleep` durante o controle do robô.

## Rúbrica

1. +1,0 - O robô consegue seguir a linha vermelha.
2. +1,0 - O robô consegue mudar de pista e seguir a linha verde.
3. +1,0 - O robô consegue mudar de pista e seguir a linha azul.
4. +2,0 - O robô consegue dar uma volta completa na pista azul e parar.

## Vídeo

Grave um vídeo mostrando que o robô é capaz de realizar o comportamento completo ou algum comportamento parcial. Publique os vídeos no YouTube e inclua apenas o `link` no arquivo `README.md` do seu repositório.
