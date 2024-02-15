# Entregável 4 de Robótica Computacional

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

# Exercício 1 - Robô quadrado (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um nó denominado `quadrado` que faça o robô **real** se mova em uma trajetória que se ***aproxima*** de um quadrado. O nó deve:

* Possui dois estados, `andar` e `girar`

* Utiliza a odometria para `girar` em 90 graus

* Utiliza o metodo `Dead Reckoning` para `andar` os lados do quadrado. Neste metodo, você se desloca em velocidade constante por um tempo fixo, sem receber feedback de quanto, realmente, se deslocou.

* Para os lados, calcule o tempo que passou até chegar no tempo desejado. Exemplo `v=0.5 [m/s] por t=1 [s]` equivale a um quadrado de `lado= 0.5 [m]`

**DICA 2** - Para girar o robo em 90o, some pi/2... TODO....







# Exercício 2 - Robô Quase Indeciso (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um nó denominado `indeciso` que, utilizando o laser, faça com que o robô **real** se afaste da parede quando o obstáculo à sua frente estiver a menos de `0.95m` e se aproximar quando estiver a mais de `1.05m`, caso contrário, o robô deve ficar parado. Portanto o robô deve parar eventualmente. O nó deve:

* Ter três estados, `frente`, `tras` e `parar`.

* Avalie na função `control` para qual estado o robô deve ir.


# Exercício 3 - Robô Limpador (4 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um nó denominado `indeciso` que, utilizando o laser, faça com que o robô **real** tenha o seguinte comportamente:

* Mova-se em frente até encontrar um obstáculo a menos de `0.3m` à sua frente.

* Gire até que o obstáculo, mais proximo, esteja em sua traseira.

* Mova-se em frente e repita o processo.