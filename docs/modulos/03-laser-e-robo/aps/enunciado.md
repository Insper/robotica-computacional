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

# Exercício 2 - Robô quadrado (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `quadrado.py` com um nó denominado `quadrado_node`, que faça o robô **real** se mover em uma trajetória que se ***aproxima*** de um quadrado.

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
Baseando-se no código `base_action.py` do módulo 3, crie um arquivo chamado `girar.py` com uma classe denominada de `Girar` e um nó denominado `girar_node`, que faça o robô **real** gire 90 graus no sentido anti-horário.

Crie a seguinte função auxiliar para ajustar o ângulo no limite de `[-pi, pi]`, isso é importante para evitar problemas de ângulo quando o robô gira mais de uma volta ou voltas negativas:
A função `np.arctan2` é utilizada para normalizar o erro angular entre `-pi` e `pi`.

```python
    def ajuste_angulo(self, angulo):
        """
        Ajusta o ângulo para o intervalo [-pi, pi].
        """
        return np.arctan2(np.sin(angulo), np.cos(angulo))
```

Para rodar o robô, primeiro calcule o ângulo desejado, somando `pi/2` ao angulo atual:

```python
self.goal_yaw = self.ajuste_angulo(self.yaw + np.pi/2)
```

E depois, a cada iteração, calcule o erro entre o angulo atual e o desejado, até que o erro seja menor que ~`2` graus.

```python
erro = self.ajuste_angulo(self.goal_yaw - self.yaw)
```

E depois calcule o erro entre o angulo atual e o desejado, até que o erro seja menor que ~`+-2` graus, finalizando a ação ao entrar no estado `stop`. Caso o erro seja maior do que o limiar, faça o seguinte:

1. Quando o erro for **menor** que 0 o robô deve girar no **sentido horário**;
2. Quando for **maior** que 0, no **sentido anti-horário**.


## Critérios de Avaliação:

1. Nó importado corretamente do pacote `robcomp_util`.
2. Desenvolver a ação `Andar` que funciona idependentemente do nó principal.
3. Desenvolver a ação `Girar` que funciona independentemente do nó principal.
4. Executa a rotação utilizando feedback da odometria.
2. Desenvolveu o nó `quadrado_node` que alterna entre as açoes `andar` e `girar`, até completar um quadrado.
3. Não utiliza nenhuma função de `sleep` para controlar o tempo de execução.
5. **Vídeo:** Mostra o robô executando o comportamento e "desenhando" pelos menos dois quadrados no chão.
6. **Vídeo:** O robô não colide com nenhum obstáculo.
7. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

# Exercício 3 - Robô Limpador (3 pontos)
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
