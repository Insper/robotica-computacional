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

O nó deve:

* Ter três estados, `forward`, `backward`, `stop` e `done`.

* O nó deve começar no estado `forward`.

* Em cada estado, o robô deve decidir se permanece no estado atual ou muda para outro estado com base nas leituras do laser.

* Se permanecer no estado atual, o robô deve definir a velocidade linear.

* Não modifique a função `control()` do módulo base.

* Apenas a função `control()` deve publicar o comando de velocidade.

**Dica:** Se o robô não parar, tente diminuir a velocidade linear.

## Critérios de Avaliação:

1. Nó importa corretamente do pacote `my_package`.
2. Desenvolveu o nó `indeciso_node` com os comportamentos corretos.
3. **Vídeo:** Mostra o robô executando o comportamento e se aproxima e afasta pelo menos **2** vezes da parede antes de parar. (ajuste a velocidade para que o robô não pare muito rápido).
4. **Vídeo:** O robô não colide com nenhum obstáculo.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

# Exercício 2 - Robô quadrado (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `quadrado.py` com um nó denominado `quadrado_node`, que faça o robô **real** se mover em uma trajetória que se ***aproxima*** de um quadrado.

## Ação Cliente (Principal)

O nó principal deve:

* Instanciar duas ações, `andar` e `girar`

* Ter três estados, `andar`, `girar`, `done`.

* Os estados `andar` e `girar` devem **iniciar** e **aguardar** a conclusão de suas respectivas ações antes de mudar para o próximo estado.

* O nó deve começar no estado `andar`.

* O nó deve alternar entre os estados `andar` e `girar`, até que o robô tenha **completado UM quadrado completo**, ou seja, 4 execuções de `andar` e 4 execuções de `girar`.

* **Não utilize nenhuma função de `sleep` em nenhum lugar do código!**

## Ação de Andar

Na atividade [Estrutura Básica](../atividades/2-estrutura-basica.md) você implementou a ação de andar uma distância `d`. Teste a ação no simulador para verificar se o robô anda a distância correta.

## Ação de Girar
Baseando-se no código `base_action.py` do módulo 3, crie um arquivo chamado `girar.py` com uma classe denominada de `Girar` e um nó denominado `girar_node`, que faça o robô **real** gire `rotacao`, onde a variavel `rotacao` é a quantidade de graus que o robô deve girar e será fornecida pelo cliente da ação, na função `reset()`.

Crie a seguinte função auxiliar para ajustar o ângulo no limite de `[-pi, pi]`, isso é importante para evitar problemas de ângulo quando o robô gira mais de uma volta ou número de voltas negativas:
A função `np.arctan2` é utilizada para fazer a normalização do ângulo entre `-pi` e `pi`.

```python
    def ajuste_angulo(self, angulo):
        """
        Ajusta o ângulo para o intervalo [-pi, pi].
        """
        return np.arctan2(np.sin(angulo), np.cos(angulo))
```

Para rodar o robô, primeiro calcule o ângulo desejado, somando o valor da rotação **em radianos** ao ângulo atual:

```python
self.goal_yaw = self.ajuste_angulo(self.yaw + rotacao)
```

Este será nosso objetivo angular, em seguida, a cada iteração, calcule o erro entre o angulo atual e o desejado, até que o erro seja menor que ~`2` graus, lembre-se que os ângulos estão em radianos.

```python
erro = self.ajuste_angulo(self.goal_yaw - self.yaw)
```

Quando o erro for menor que ~`+-2` graus podemos finalizar a ação mudando o estado para `stop`. Caso o erro seja maior do que o limiar de `+-2` graus, faça o seguinte:

1. Quando o erro for **menor** que 0 o robô deve girar no **sentido horário**;
2. Quando for **maior** que 0, o robô deve girar no **sentido anti-horário**.


## Critérios de Avaliação:

1. Nó importado corretamente do pacote `robcomp_util`.
2. Desenvolver a ação `Andar` que funciona idependentemente do nó principal.
3. Desenvolver a ação `Girar` que funciona independentemente do nó principal.
4. Executa a rotação utilizando feedback da odometria.
2. Desenvolveu o nó `quadrado_node` que alterna entre as açoes `andar` e `girar`, até completar um quadrado.
3. Não utiliza nenhuma função de `sleep` para controlar o tempo de execução.
5. **Vídeo: Desenhe o quadrado na referência do laboratório**
5. **Vídeo:** Executa apenas o nó `quadrado_node`.
5. **Vídeo:** Mostra o robô executando o comportamento e "desenhando" um quadrado no chão.
6. **Vídeo:** O robô não colide com nenhum obstáculo.
7. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

# Exercício 3 - Robô Limpador (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `limpador.py` com uma classe denominada `Limpador` e um nó denominado `limpador_node` que, utilizando o laser, faça com que o robô **real** tenha o comportamento de um robô limpador equipado com um laser 2D (então, sem colisões).

## Ação Cliente (Principal)

O nó principal deve:

* Instanciar a ação de `girar`.
* Ter quatro estados, `procurar`, `limpar`, `esperar` e `girar`.
* O estado `girar` deve **iniciar** e **aguardar** a conclusão de suas ação antes de mudar para o próximo estado.
* O estado `procurar` deve fazer com que o robô gire em trajetória elíptica até encontrar um obstáculo à frente ou à direita.
* O estado `limpar` deve fazer com que o robô se mova para frente, limpando a área à sua frente até que o sensor detecte um obstáculo em sua trajetória (frente).
* O estado `esperar` deve fazer com que o robô espere até encontrar uma direção (direita, esquerda ou traseira) onde não exista obstáculos.
* A ação de girar deve levar o robô a girar até a direção livre escolhida.

Mais detalhes sobre as ações e transições estão no handout na atividade [1-maquina-de-estados](../atividades/1-maquina-de-estados.md).

## Critérios de Avaliação:

1. Nó importado corretamente do pacote `robcomp_util`.
2. Desenvolveu o nó `limpador_node` com os comportamentos corretos.
3. Executa a rotação utilizando feedback da odometria.
4. **Vídeo:** Mostra o robô executando o comportamento e se aproxima e desvia de pelo menos **10** obstáculos.
5. **Vídeo:** Mostra que robô o robô fica parado eternamente se não estirem direções livres (demonstre colocando objetos em volta do robô).
6. **Vídeo:** O robô não colide com nenhum obstáculo.
7. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

## Desafio (+2 pontos bônus)

Uma vez finalizado os outros exercicios da APS, juntem-se com pelo menos **4** outros grupos e gravem um vídeo dos **5** robôs limpadores em ação. Cada robô deve ter um comportamento independente, mas todos devem estar no mesmo ambiente e **não podem colidir.**.

O vídeo deve  ter duração minima de 2 minutos e ser gravado com um professor ou monitor do curso, que irá avaliar a pontuação extra.

Cada grupo deve gravar o vídeo do seu robô e postar no Youtube e incluir o link no README.md com um comentário com o nomes dos outros grupos. 

Os vídeos devem seguir os critérios de avaliação do exercício 3 com uma adiação:

* **Vídeo:** Mostra uma ocasião em que um robô se aproxima de outro robô mas para antes de colidir.
* **Vídeo:** Mostra pelo menos um robô esperando outros saírem do caminho antes de começar a se mover.

!!! importante
    Está atividade é uma atividade em grupo de grupos, neste ponto, os grupos devem se organizar e colaborar para garantir que TODOS os robôs funcionem corretamente e não colidam entre si.
