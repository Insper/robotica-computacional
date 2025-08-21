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

---

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

---

# Exercício 1 - Robô Quase Indeciso (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `indeciso.py` com um nó denominado `indeciso_node` que, utilizando o laser, faça com que o robô **real** se afaste da parede quando o obstáculo à sua frente estiver a menos de `0.95m` e se aproximar quando estiver a mais de `1.05m`, caso contrário, o robô deve ficar parado.

O nó deve:

* Ter **três** estados: `forward`, `backward`, `stop`.
* O nó deve começar no estado `forward`.
* Em cada estado, o robô deve decidir se permanece no estado atual ou muda para outro estado com base nas leituras do laser.
* Se permanecer no estado atual, o robô deve definir a velocidade linear.
* **Não** modificar a função `control()` do módulo base.
* Apenas a função `control()` deve **publicar** o comando de velocidade.

**Dica:** Se o robô não parar, **reduza a velocidade linear**.

## Critérios de Avaliação

1. Importa corretamente do pacote `robcomp_util`.
2. Implementa o nó `indeciso_node` com os comportamentos corretos.
3. **Vídeo:** Mostra o robô aproximando‑se e afastando‑se da parede pelo menos **2 vezes** antes de parar (ajuste a velocidade para não parar rápido demais).
4. **Vídeo:** O robô **não colide** com obstáculos.
5. **Vídeo:** Link do vídeo no YouTube.

---

# Exercício 2 - Robô Quadrado (3 pontos)

Com base em `base_control.py` (Módulo 3), crie `quadrado.py` com um nó `quadrado_node` que faça o robô **real** movimentar‑se em uma trajetória que **se aproxima** de um quadrado.

## Ação Cliente (Principal)

O nó principal deve:

* Instanciar duas **ações**: `andar` e `girar`.
* Ter três estados: `andar`, `girar`, `done`.
* Em `andar` e `girar`, **iniciar** e **aguardar** a conclusão das **respectivas ações** antes da transição.
* O nó deve começar no estado `andar`.
* O nó deve alternar entre os estados `andar` e `girar`, até que o robô tenha **completado UM quadrado completo** (4× `andar` e 4× `girar`).
* **Não** utilizar funções de `sleep` em nenhuma parte do código.

## Ação de Andar

Na atividade [Estrutura Básica](../atividades/2-estrutura-basica.md) você implementou a ação de andar uma distância `d`. **Teste** no simulador para validar a ação.

## Ação de Girar
Baseando-se no código `base_action.py` do módulo 3, crie um arquivo chamado `girar.py` com uma classe denominada de `Girar` e um nó denominado `girar_node`, que faça o robô **real** gire `rotacao`, onde a variavel `rotacao` é a quantidade de graus que o robô deve girar e será fornecida pelo cliente da ação, na função `reset()`.

Crie a seguinte função auxiliar para ajustar o ângulo no limite de `[-pi, pi]`, isso é importante para evitar problemas de ângulo quando o robô gira mais de uma volta ou número de voltas negativas:
A função `np.arctan2` é utilizada para fazer a normalização do ângulo entre `-pi` e `pi`.

```python
def ajuste_angulo(self, angulo):
    """Ajusta o ângulo para o intervalo [-pi, pi]."""
    return np.arctan2(np.sin(angulo), np.cos(angulo))
```

### Alvo angular
Para girar o robô, primeiro calcule o ângulo desejado, somando o valor da rotação **em radianos** ao ângulo atual:

```python
self.goal_yaw = self.ajuste_angulo(self.yaw + rotacao)
```

### Erro angular a cada iteração
Este será nosso objetivo angular, em seguida, a cada iteração, calcule o erro entre o angulo atual e o desejado, até que o erro seja menor que ~`2` graus, lembre-se que os ângulos estão em radianos.

```python
erro = self.ajuste_angulo(self.goal_yaw - self.yaw)
```

### Condição de parada

Quando o erro for menor que ~`+-2` graus podemos finalizar a ação mudando o estado para `stop`. Caso o erro seja maior do que o limiar de `+-2` graus, faça o seguinte:

1. Quando o erro for **menor** que 0 o robô deve girar no **sentido horário**;
2. Quando for **maior** que 0, o robô deve girar no **sentido anti-horário**.

## Critérios de Avaliação

1. Nó importado corretamente do pacote `robcomp_util`.
2. Implementa a ação **`Andar`** que funciona **independentemente** do nó principal.
3. Implementa a ação **`Girar`** que funciona **independentemente** do nó principal.
4. Executa a rotação usando **feedback da odometria**.
5. Implementa o nó `quadrado_node` que alterna entre as ações até completar um quadrado.
6. **Não** usa `sleep` para controlar tempo de execução.
7. **Vídeo:** Desenha o quadrado na **referência do laboratório**.
8. **Vídeo:** Executa **apenas** o nó `quadrado_node`.
9. **Vídeo:** Mostra o robô "desenhando" um quadrado no chão.
10. **Vídeo:** O robô **não colide** com obstáculos.
11. **Vídeo:** Link do vídeo no YouTube.

---

# Exercício 3 - Robô Limpador (3 pontos)

Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `limpador.py` com uma classe denominada `Limpador` e um nó denominado `limpador_node` que, utilizando o laser, faça com que o robô **real** tenha o comportamento de um robô limpador equipado com um laser 2D (então, sem colisões).

## Ação Cliente (Principal)

O nó principal deve:

* Instanciar a ação de **`girar`**.
* Ter quatro estados: `procurar`, `limpar`, `esperar`, `girar`.
* Em `girar`, **iniciar** e **aguardar** a conclusão da ação antes da transição.
* O estado `procurar` deve fazer com que o robô gire em trajetória elíptica até encontrar um obstáculo à **frente** ou à **direita**.
* O estado `limpar` deve fazer com que o robô avançar, limpando a área à sua frente até que o sensor detecte um obstáculo obstáculo **frontal**.
* O estado `esperar` deve fazer com que o robô espere até encontrar uma direção (**direita**, **esquerda** ou **traseira**) onde não exista obstáculos.
* A ação de girar deve levar o robô a girar até a direção livre escolhida.

Mais detalhes sobre ações e transições no handout [1-maquina-de-estados](../atividades/1-maquina-de-estados.md).

## Critérios de Avaliação:

1. Nó importado corretamente do pacote `robcomp_util`.
2. Implementa o nó `limpador_node` com os comportamentos corretos.
3. Executa a rotação utilizando **feedback da odometria**.
4. **Vídeo:** Mostra o robô aproximando‑se e desviando de pelo menos **10** obstáculos.
5. **Vídeo:** Demonstra que o robô permanece parado indefinidamente quando **não existem direções livres** (ex.: circunde o robô com objetos).
6. **Vídeo:** O robô **não colide** com nenhum obstáculo.
7. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

## Desafio (+2 pontos bônus)

Após finalizar os demais exercícios, reúnam‑se com **pelo menos 4** outros grupos e gravem um vídeo com 5 robôs limpadores em ação. Cada robô deve ter comportamento independente, todos no mesmo ambiente, e não podem colidir.

O vídeo deve ter duração **mínima de 2 minutos** e ser gravado com um professor, que avaliará a pontuação extra.

Cada grupo deve postar o vídeo no YouTube e incluir o link no README.md, citando nos comentários os nomes dos outros grupos participantes.

Os vídeos devem seguir os critérios do Exercício 3, com as seguintes **adições**:

* **Vídeo:** Mostra uma situação em que um robô se aproxima de outro, mas **para antes** de colidir.
* **Vídeo:** Mostra pelo menos um robô **aguardando** outros saírem do caminho antes de voltar a se mover.

!!! importante
    Está atividade é uma atividade de **colaboração entre grupos**, neste ponto, os grupos devem se organizar e **colaborar** para garantir que **todos** os robôs funcionem corretamente e **não colidam entre si**.

