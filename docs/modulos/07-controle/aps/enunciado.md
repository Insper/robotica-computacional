# Entregável 3 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para este entregável, você deve utilizar o robô real, mas você pode testar o código no simulador.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_3`.
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `my_package` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

____________________________________________________________________

# Exercício 1 - GoTo (4 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `goto.py`, com uma classe `GoTo` e com um nó denominado `goto_node`, que, dado uma posição, faça o robô **simulado** `=)` se mova ***precisamente*** para este ponto em qualquer posição. O nó deve:

* A classe `GoTo` deve herdar de `Node` e `Odom`.

* A classe `GoTo` deve ter um método `__init__` que recebe a posição uma variável do tipo `Point` e salva em uma variável `self.point`.

* Ter três estados, `center`, `goto` e `stop`.

* O estado `center` deve ser o estado inicial e faz o robô girar até que ele esteja alinhado com o ponto desejado.

* Quando chegar no ponto desejado, o robô deve entrar no estado `stop`.

* Deve ter um função `get_angular_error` que primeiro calcula o angulo entre a posição atual e o ponto desejado `theta` e depois calcula o erro entre o angulo atual e o angulo desejado.

* `get_angular_error` também deve calcular a distância entre o robô e o ponto desejado.

* O estado `goto` deve fazer o robô se mover até o ponto desejado e parar quando estiver **BEM PERTO** do ponto.

* Utilize duas constante proporcionais, `kp_linear` e `kp_angular` para controlar a velocidade linear e angular do robô.

Quando o nó estiver funcionando corretamente, baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `quadrado_preciso.py`, com uma classe `Quadrado` e com um nó denominado `quadrado_node`. Usando o robô **real**, faça um quadrado ***preciso*** nas arestas de um ladrilho do nosso laboratório. O nó deve:

* Ter dois estados, `segue` e `para`.

* **Chame** (não herde) a classe `GoTo` com as coordenadas do primeiro ponto do quadrado.

* O estado `segue` deve criar um loop que chama a função `control` do `GoTo` até que o robô esteja no ponto desejado - ou seja, `control` da classe `Quadrado` não vai ser chamada porque o código esta preso no loop.

* Quando sair do loop, mude a variável `point` e `robot_state` da classe `GoTo` para, respectivamente, o próximo ponto do quadrado e `center`.

## Critérios de Avaliação:

1. `GoTo` funciona a partir / até qualquer ponto em qualquer quadrante.
2. A classe `Quadrado` chama a classe `GoTo` corretamente.
3. Não utiliza nenhuma função de `sleep` para controlar o tempo de execução.
4. **Vídeo:** Mostra o robô executando o comportamento e "desenhando" e seguindo um ladrilho do laboratório com precisão.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.
____________________________________________________________________

# Exercício 2 - Derruba Creeper (6 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `derrubador.py` com uma classe denominada `Derrubador` e um nó denominado `derrubador_node` que derrube 4 creepers ao redor de uma arena quadrado, em uma ordem pré-definida. O robô sempre inicia no centro do quadrado. O nó deve:

* Recebe corretamente a sequência de creepers que devem ser derrubados.

* Se comunicar com um nó `creepers` que identifica o ID/Cor dos creepers e suas posições.

* Não se increver no tópico de visão, mas sim no tópico de creepers.

* Recebe a sequência de cores/ID dos creepers que devem ser derrubados.

* Se aproximar de cada creeper, em ordem, e parar de forma clara e precisa.

* Depois de parar, o robô deve derrubar o creeper com a garra.

* Siga para o próximo creeper até que todos os creepers tenham sido derrubados.

* Retorne para o centro do quadrado e pare.

## Arena
A arena é um quadrado de 2m x 2m, com o robô iniciando no centro. Em cada canto do quadrado, há um creeper de **verde** ou **azul** com dois possíveis IDs. A ordem dos creepers que deve ser seguinda é informada na hora da execução, como uma lista de strings, por exemplo, `['verde_17', 'verde_21', 'azul_21', 'azul_17']`.

Tanto a ordem dos creepers quanto a posição deles é aleatória, então o robô deve ser capaz de se adaptar a qualquer situação.

## Critérios de Avaliação:

1. Desenvolveu um nó que publica corremente os creepers.
2. O nó `derrubador_node` não se inscreve no tópico de visão.
3. O robô se aproxima de cada creeper e **para** de forma clara e então **derruba** o creeper com a garra.
4. **Vídeo:** Mostra o robô executando o comportamento desejado e derrubando os creepers na ordem correta.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.

## Competição (+2 ponto bônus)

**Para os alunos que completarem o exercício 2 até o dia 29/04/2024** vamos realizar uma competição no estilo "mata-mata". Onde os grupos vão enfrentam-se em pares para ver quem derruba os creepers mais rápido. Os grupos vão disputar na mesma arena e na mesma sequência de creepers. A partir da semifinal, serão adicionados creepers da cor **vermelha**.

O vencedor da competição ganhará 2 pontos bônus na nota da APS e os grupos que chegarem na semifinal e conseguirem completar com os creepers vermelhos, ganharão 1 ponto bônus.
