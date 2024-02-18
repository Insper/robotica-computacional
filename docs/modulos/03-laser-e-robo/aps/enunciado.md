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
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `my_package` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

____________________________________________________________________

# Exercício 1 - Robô quadrado (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `quadrado.py` com um nó denominado `quadrado_node`, que faça o robô **real** se mova em uma trajetória que se ***aproxima*** de um quadrado. O nó deve:

* Possui dois estados, `andar` e `girar`

* Utiliza a odometria para `girar` em 90 graus

* Utiliza o metodo `Dead Reckoning` para `andar` os lados do quadrado. Neste metodo, você se desloca em velocidade constante por um tempo fixo, sem receber feedback de quanto, realmente, se deslocou.

* Não utilize nenhuma função de `sleep`, calcule o tempo decorrido até chegar no tempo desejado. Exemplo `v=0.5 [m/s] por t=1 [s]` equivale a um quadrado de `lado= 0.5 [m]`

**DICA 1** - Para somar `pi/2` ao angulo atual: `self.goal_yaw = (self.yaw_2pi + np.pi / 2) % (2 * np.pi)`. E depois calcule o erro entre o angulo atual e o desejado, até que o erro seja menor que ~`2` graus.

## Critérios de Avaliação:

1. Nó importa corretamente do pacote `my_package`.
2. Desenvolveu o nó `quadrado_node` com os comportamentos corretos.
3. Não utiliza nenhuma função de `sleep` para controlar o tempo de execução.
4. Executa a rotação utilizando feedback da odometria.
5. **Vídeo:** Mostra o robô executando o comportamento e "desenhando" pelos menos dois quadrados no chão.
6. **Vídeo:** O robô não colide com nenhum obstáculo.
7. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

# Exercício 2 - Robô Quase Indeciso (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `indeciso.py` com um nó denominado `indeciso_node` que, utilizando o laser, faça com que o robô **real** se afaste da parede quando o obstáculo à sua frente estiver a menos de `0.95m` e se aproximar quando estiver a mais de `1.05m`, caso contrário, o robô deve ficar parado. Portanto o robô deve parar eventualmente. O nó deve:

* Ter três estados, `forward`, `backward` e `stop`.

* Avalie na função `control` para qual estado o robô deve ir.

**Dica:** Se o robô não parar, tente diminuir a velocidade linear.

## Critérios de Avaliação:

1. Nó importa corretamente do pacote `my_package`.
2. Desenvolveu o nó `indeciso_node` com os comportamentos corretos.
3. **Vídeo:** Mostra o robô executando o comportamento e se aproxima e afasta pelo menos **2** vezes da parede antes de parar. (ajuste a velocidade para que o robô não pare muito rápido).
4. **Vídeo:** O robô não colide com nenhum obstáculo.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.
____________________________________________________________________

# Exercício 3 - Robô Limpador (4 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `limpador.py` com um nó denominado `limpador_node` que, utilizando o laser, faça com que o robô **real** tenha o seguinte comportamente:

* Ter dois estados, `forward`, `turn`.

* Mova-se em frente até encontrar um obstáculo a menos de `0.5m` à sua frente (esse valor pode ser ajustado para melhor desempenho).

* Gire até que o obstáculo mais proximo, esteja na direita inferior (aproximadamente `225` graus).

* Mova-se em frente e repita o processo.

## Critérios de Avaliação:

1. Nó importa corretamente do pacote `my_package`.
2. Desenvolveu o nó `limpador_node` com os comportamentos corretos.
3. **Vídeo:** Mostra o robô executando o comportamento e se aproxima e desvia de pelo menos **10** obstáculos.
4. **Vídeo:** O robô não colide com nenhum obstáculo.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.

## Desafio (+2 ponto)

Juntem-se com pelo menos **4** outros grupos e gravem um vídeo dos **5** robôs limpadores em ação. Cada robô deve ter um comportamento independente, mas todos devem estar no mesmo ambiente e **não podem colidir.**.

Esse vídeo deve ser gravado com um proffessor ou monitor do curso, que irá avaliar a pontuação extra.

Cada grupo deve gravar o vídeo do seu robô e postar no Youtube e incluir o link no README.md com um comentário com o nomes dos outros grupos. Os vídeos devem seguir os critérios de avaliação do exercício 3.


