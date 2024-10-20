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
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_6`.
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

## Quando o nó estiver funcionando corretamente

Quando o nó estiver funcionando corretamente, baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `quadrado_preciso.py`, com uma classe `Quadrado` e com um nó denominado `quadrado_node`. Usando o robô **real**, faça um quadrado ***preciso*** nas arestas de um ladrilho do nosso laboratório. O nó deve:

* Ter dois estados, `segue` e `para`.

* **Chame** (não herde) a classe `GoTo` com as coordenadas do primeiro ponto do quadrado.

* O estado `segue` deve criar um loop que chama a função `control` do `GoTo` como no exemplo [aqui](../util/run_rotate2.py).

* Depois, mude o valor de `point` no `GoTo` para o próximo ponto do quadrado e mude o estado do `GoTo` para `center`.

## Critérios de Avaliação:

1. `GoTo` funciona a partir (até) qualquer ponto em qualquer quadrante.
2. A classe `Quadrado` chama a classe `GoTo` corretamente.
3. Não utiliza nenhuma função de `sleep` para controlar o tempo de execução.
4. **Vídeo:** Mostra o robô executando o comportamento e "desenhando" e seguindo um ladrilho do laboratório com precisão.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.