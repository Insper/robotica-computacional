# Projeto-1 de Robótica Computacional

## Instruções gerais
* O projeto deve ser realizada em duplas.
* Contribuir com outras duplas é aceitável, mas não serão aceitas copias de código entre duplas.
* **Qualquer suspeita de plágio será investigada e, se confirmada, resultará em nota zero para ambas as duplas envolvidas.**

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes da sua dupla no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para este projeto, você vai utilizar o robô simulado.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `projeto_1`.
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `robcomp_util` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.
____________________________________________________________________
# **IMPORTANTE**
Atualize o pacote do `robcomp_interfaces` que existe em seu SSD com os comandos abaixo:
```bash
cd ~/colcon_ws/src/my_simulation/
git stash
git pull
cb
```
____________________________________________________________________

# Exercício 0 - Organização & Qualidade
Este exercício está avaliando a organização e qualidade dos vídeos dos exercícios da APS e do arquivo `README.md`.

## Critérios de Avaliação:
* O pacote foi corretamente configurado.
* As dependências do pacote estão corretas.
* A configuração dos nós foi realizada corretamente.
* Os diretórios e arquivos estão organizados de forma adequada.
* **Vídeo:** Utilize o comando `ros2 run` para executar o nó, mostre o comando sendo executado no terminal.
* **Vídeo:** O vídeo foi gravado na **horizontal**.
* **Vídeo:** O vídeo foi gravado em um ambiente bem iluminado.
* **Vídeo:** O audio está claro e sem ruídos, se desejar, remova o audio e adicione uma música de fundo.
* **Vídeo:** Na descrição do vídeo no Youtube, está descrito o que o robô está fazendo.
* **Vídeo:** Pelo vídeo, é possível entender o que o robô está fazendo.
* **README.md:** O link do vídeo está correto e foi adicionado no campo específico.
* **README.md:** O arquivo README.md tem o nome completo e o email de todos os integrantes do grupo.
____________________________________________________________________

# Objetivo

Neste primeiro projeto vamos trabalhar com exploração e navegação do robô simulado. Seu objetivo é resolver o exercício 1 da [AI-24b](https://insper.github.io/robotica-computacional/simulados/ai_24b/enunciado/), mas utilizando o `GOTO` para navegar entre as paredes.

Utilize o comando abaixo para iniciar o simulador no mapa do projeto:
`ros2 launch my_gazebo tres_paredes.launch.py`


# Parte 1 - Explorar Labirinto (3 pontos)
Utilizando o pacote `Cartographer` e o pacote `Navigation`, explore o mapa do projeto enquanto controla o robô simulado manualmente, explorando cada canto do mapa e então salve o mapa.

Utilizando o pacote `Cartographer` e o pacote `Navigation`, explore o mapa do projeto enquanto controla o robô simulado manualmente, explorando cada canto do mapa. Por fim, salve o mapa do labirinto e adicione o arquivo `map.pgm` e `map.yaml` no seu repositório.

!!! dica
    O `Navigation` atualmente tem um bug que impede o robô de navegar contornando obstáculos, então, envie vários pontos para o robô, para que ele possa navegar pelo labirinto.

## Vídeo

Grave um vídeo do robô explorando o labirinto do laboratório e adicione o link no arquivo `README.md` do seu repositório. No vídeo, mostre a tela do computador com o Rviz e então, mostre o robô explorando o labirinto.

##
____________________________________________________________________

# Parte 2 - Labirinto-GoTo (7 pontos)

Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `teseu.py`, como uma classe `Teseu` e com um nó denominado `teseu_node`, este nó deve conversar com o `Handler` e seguir o caminho das paredes a partir de uma sequência de pontos, obtidas do mapa do labirinto. O nó deve:

* Utilize o script `visualizar_mapa.py` para obter a sequência de pontos do labirinto (Cap. 8)
    * Depois, anote as coordenadas de cada trajeto que o robô deveria fazer e armazene em um dicionário, onde a chave é o nome do trajeto (**cima** ou **baixo**) e o valor é uma lista de coordenadas. Lembrando que a força desconhecida está sempre na mesma posição, entre as duas paredes mais distantes de onde o robô começa.

* Use seu código do `goto.py`.

* Modifique, de alguma forma, a ação `GoTo` para fazer o robô se movimentar entre uma lista de pontos.

* Modifique o `goto.py` para herdar do `AMCL` no lugar do `Odom`

    * Baixe o código do `AMCL` no capítulo 8 da unidade 3, ou pelo [link](https://insper.github.io/robotica-computacional/modulos/08-slam/util/amcl.py)

* Rode o pacote `Navigator` com o mapa que gravou no ex. anterior para iniciar o mapa.

* Seguir as intruções do Handler e caminhar pelas paredes à partir de uma sequência de pontos, obtidas do mapa do labirinto.

* Como no exercício, deve retornar ao encontrar a força desconhecida e retornar ao ponto inicial.

* Ao chegar no ponto inical, o robô deve entrar em um estado `stop` e parar.

!!! dica
    Rode o Navigator pelo comando

    ```bash
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
    ```

    Assumindo que o mapa ainda esta em sua HOME


## Critérios de Avaliação:

1. Execute o pacote `Navigation` para navegar no labirinto.
2. Execute o nó `teseu_node` para fazer o robô escapar do labirinto.
3. Não utiliza nenhuma função de `sleep` para controlar o tempo de execução.
4. Todas as exigências e rúbricas do exercício 1 foram atendidas.
4. **Vídeo:** Mostra o robô executando o comportamento e escapando do labirinto.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.

# Parte 3 - Extra Labirinto-GoTo-Robo-Real (+2 pontos)

Repita o código a parte 2, mas agora utilizando o robô real. O robô deve ser capaz de escapar do labirinto e retornar ao ponto inicial. Não precisa se comunicar com o `Handler`, seguindo apenas os pontos obtidos do mapa. Mude o nome do arquivo para `teseu_real.py` e o nó para `teseu_real_node`.

A entrega deve ser feita em um vídeo, do robô real escapando do labirinto e seu repositório deve conter o mapa do labirinto.