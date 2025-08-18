# Máquina de Estados em Robótica

Nesta atividade, vamos revisar o conceito de **máquina de estados finitos (FSM)** e como implementá‑la para controlar um robô.

Imagine o robô seguidor de linha abaixo:

![Robô seguidor de linha](figs/linha.png)

Uma forma de implementar o comportamento desse robô é utilizar uma **máquina de estados**: um modelo que descreve o comportamento do sistema a partir de **estados** e **transições**. Cada estado representa uma condição do sistema; cada transição, uma mudança de estado disparada por entradas/sensores.

No caso do robô seguidor de linha, podemos definir os seguintes estados:

* **Andar para frente:** o robô avança acionando ambos os motores em velocidade desejada.
* **Virar para a direita:** o robô gira para a direita acionando o motor esquerdo mais forte que o direito.
* **Virar para a esquerda:** o robô gira para a esquerda acionando o motor direito mais forte que o esquerdo.
* **Parar:** o robô desliga ambos os motores.

Agora, definimos as **transições** entre estados (de acordo com a leitura dos sensores de linha):

* **Andar para frente:** quando **ambos** os sensores detectam a linha.
* **Virar para a direita:** quando **apenas o sensor direito** detecta a linha.
* **Virar para a esquerda:** quando **apenas o sensor esquerdo** detecta a linha.
* **Parar:** quando **nenhum** sensor detecta a linha.

Em Python, podemos implementar a máquina de estados com uma variável que guarda o **estado atual** (string) e uma **função por estado**, responsável pelas ações daquele estado. A função principal executa a função do estado atual e decide se o estado deve mudar. 

Em vez de um grande bloco `if/elif`,essa é uma abordagem mais elegante que usa um **dicionário** que mapeia `estado → função`.

Então, a estrutura do código ficaria assim:

```python
class SeguidorDeLinha:
    def __init__(self):
        # Estado inicial
        self.robot_state = 'frente'
        
        # Tabela de despacho: estado → método
        self.state_machine = {
            'frente': self.frente,
            'direita': self.direita,
            'esquerda': self.esquerda,
            'parar': self.parar,
        }

    # ===== Comportamentos por estado =====
    def frente(self):
        # Código para andar para frente
        pass

    def direita(self):
        # Código para virar para a direita
        pass

    def esquerda(self):
        # Código para virar para a esquerda
        pass

    def parar(self):
        # Código para parar
        pass

    # ===== Laço de controle =====
    def control(self):
        # Executa a ação do estado atual
        self.state_machine[self.robot_state]()

        # Verifica leituras dos sensores e avalia transições
        # if sensores == ...:
        #     self.robot_state = 'direita'  # exemplo
```

No código acima, definimos o estado inicial como `frente` e criamos o dicionário `state_machine` que associa cada estado a uma função. A função `control` chama a função do **estado atual** e, em seguida, avalia as condições de **transição** para possivelmente atualizar `self.robot_state`.

Outro ponto importante em máquinas de estados é a **definição do estado inicial**. No seguidor de linha, utilizar `frente` como estado inicial costuma ser adequado: se o robô começar em `parar` ou já `virando`, ele pode **não reencontrar a linha** com facilidade.

## Robô Quase Indeciso

Agora vamos aplicar o conceito de máquina de estado para o seguinte comportamento:

Um robô que se afasta da parede quando o obstáculo à sua frente estiver a menos de `0.95m` e se aproxima quando estiver a mais de `1.05m`, caso contrário, o robô deve ficar parado.

Quais seriam os estados e transições para esse comportamento?
