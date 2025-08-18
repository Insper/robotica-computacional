# Entregável 3 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo com um celular ou camera digital. O vídeo deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para este entregável, você deve utilizar o robô real, mas você pode testar o código no simulador.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_3`.
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `robcomp_util` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

____________________________________________________________________

# Exercício 0 - Organização & Qualidade (1 pontos)
Este exercício está avaliando a organização e qualidade dos vídeos dos exercícios 2, 3 e 4, do desafio e do arquivo `README.md`.

## Critérios de Avaliação:
* **Vídeo:** O vídeo foi gravado na **horizontal**.
* **Vídeo:** O vídeo foi gravado em um ambiente bem iluminado.
* **Vídeo:** O audio está claro e sem ruídos, se desejar, remova o audio e adicione uma música de fundo.
* **Vídeo:** Na descrição do vídeo no Youtube, está descrito o que o robô está fazendo.
* **Vídeo:** Pelo vídeo, é possível entender o que o robô está fazendo.
* **README.md:** O link do vídeo está correto e foi adicionado no campo específico.
* **README.md:** O arquivo README.md tem o nome completo e o email de todos os integrantes do grupo.

____________________________________________________________________
# Exercício 1 - Robô Quase Indeciso (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `indeciso.py` com um nó denominado `indeciso_node` que, utilizando o laser, faça com que o robô **real** se afaste da parede quando o obstáculo à sua frente estiver a menos de `0.95m` e se aproximar quando estiver a mais de `1.05m`, caso contrário, o robô deve ficar parado. 

Portanto o robô deve parar eventualmente.

O nó deve:

* Ter três estados, `forward`, `backward`, `stop` e `done`.

* O nó deve começar no estado `forward`.

* Em cada estado, o robô deve definir a velocidade linear e angular com base na leitura do laser.

* Não modifique a função `control()` do módulo base.



**Dica:** Se o robô não parar, tente diminuir a velocidade linear.

## Critérios de Avaliação:

1. Nó importa corretamente do pacote `my_package`.
2. Desenvolveu o nó `indeciso_node` com os comportamentos corretos.
3. **Vídeo:** Mostra o robô executando o comportamento e se aproxima e afasta pelo menos **2** vezes da parede antes de parar. (ajuste a velocidade para que o robô não pare muito rápido).
4. **Vídeo:** O robô não colide com nenhum obstáculo.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

# Exercício 3 - Robô quadrado (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `quadrado.py` com um nó denominado `quadrado_node`, que faça o robô **real** se mover em uma trajetória que se ***aproxima*** de um quadrado. O nó deve:

* Possui dois estados, `andar` e `girar`

* Utiliza a odometria para `girar` em 90 graus

* Utiliza o metodo `Dead Reckoning` para `andar` os lados do quadrado. Neste metodo, você se desloca em velocidade constante por um tempo fixo, sem receber feedback de quanto, realmente, se deslocou.

* Não utilize nenhuma função de `sleep`, calcule o tempo decorrido até chegar no tempo desejado. Exemplo `v=0.5 [m/s] por t=1 [s]` equivale a um quadrado de `lado= 0.5 [m]`

## Intruções:
### **Estado girar:**
Para rodar o robô, primeiro calcule o ângulo desejado, somando `pi/2` ao angulo atual:

```python
self.goal_yaw = self.yaw + np.pi/2
```

E depois, a cada iteração, calcule o erro entre o angulo atual e o desejado, até que o erro seja menor que ~`2` graus.

```python
erro = self.goal_yaw - self.yaw
erro = np.arctan2(np.sin(erro), np.cos(erro))
```

A função `np.arctan2` é utilizada para normalizar o erro angular entre `-pi` e `pi`. E depois calcule o erro entre o angulo atual e o desejado, até que o erro seja menor que ~`+-2` graus. Dessa forma:

1. Quando o erro for **menor** que 0 o robô deve girar no **sentido horário**;
2. Quando for **maior** que 0, no **sentido anti-horário**.



### **Estado andar:**
Para andar, defina a velocidade linear, calcule o tempo necessário para percorrer a distância desejada e armazene o tempo inicial.

E a cada iteração, calcule o tempo decorrido e compare com o tempo necessário para percorrer a distância desejada.

!!! dica
    **DICA:** A grande questão desse exercício é quando você deve armazenar o tempo inicial e o ângulo desejado. Pense em como você pode fazer isso de forma que o robô não fique atualizando o alvo a cada iteração.



## Critérios de Avaliação:

1. Nó importado corretamente do pacote `my_package`.
2. Desenvolveu o nó `quadrado_node` com os comportamentos corretos.
3. Não utiliza nenhuma função de `sleep` para controlar o tempo de execução.
4. Executa a rotação utilizando feedback da odometria.
5. **Vídeo:** Mostra o robô executando o comportamento e "desenhando" pelos menos dois quadrados no chão.
6. **Vídeo:** O robô não colide com nenhum obstáculo.
7. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

# Exercício 4 - Robô Limpador (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `limpador.py` com um nó denominado `limpador_node` que, utilizando o laser, faça com que o robô **real** tenha o seguinte comportamento:

* Ter dois estados, `forward`, `turn`.

* Mova-se em frente até encontrar um obstáculo a menos de `0.5m` à sua frente (esse valor pode ser ajustado para melhor desempenho).

* Gire `225` graus.

* Mova-se em frente e repita o processo.

## Critérios de Avaliação:

1. Nó importado corretamente do pacote `my_package`.
2. Desenvolveu o nó `limpador_node` com os comportamentos corretos.
3. **Vídeo:** Mostra o robô executando o comportamento e se aproxima e desvia de pelo menos **10** obstáculos.
4. **Vídeo:** O robô não colide com nenhum obstáculo.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

## Desafio (+2 pontos bônus)

Juntem-se com pelo menos **4** outros grupos e gravem um vídeo dos **5** robôs limpadores em ação. Cada robô deve ter um comportamento independente, mas todos devem estar no mesmo ambiente e **não podem colidir.**.

Esse vídeo deve ser gravado com um professor ou monitor do curso, que irá avaliar a pontuação extra.

Cada grupo deve gravar o vídeo do seu robô e postar no Youtube e incluir o link no README.md com um comentário com o nomes dos outros grupos. 

Os vídeos devem seguir os critérios de avaliação do exercício 3 com uma adiação:

* **Vídeo:** Mostra uma ocasião em que um robô se aproxima de outro robô mas para antes de colidir.
