# Estrutura Básica de um Nó

Para facilitar o desenvolvimento de um nó, a fornecemos uma estrutura básica de um nó da ROS 2 em python. Também fornecemos uma estrutura básica de um nó que controla o robô, incluindo a definição da maquina de estados, a função de controle e o publisher do `cmd_vel`.

* Nó base: [base.py](../util/base.py)

* Nó de ação base [base_action.py](../util/base_action.py)

* Nó base de controle: [main.py](../util/base_control.py)

Baixe os arquivos e coloque-os em uma pasta de fácil acesso. 

Agora vamos entender o que cada parte do código faz.

## Nó Base

Este script, não existe novidade em relação ao que já vimos anteriormente. Quando executado, ele chama a função `control()` a cada 0,25 segundos. Este script é útil para criar um nó que se inscreve em um tópico e executa uma função a cada nova mensagem recebida, publicando uma mensagem em outro tópico.

!!! tip
    Nó Base é útil para desenvolver módulos que se inscrevem em um tópico e publica o resultado em outro tópico, como por exemplo, um nó de visão computacional.

## Nó Base de Ação

Esse script é mais complexo, pois define uma estrutura básica para desenvolver ações que o robô deve executar.
Uma ação deve ter um início, meio e fim. Nesta estrutura básica, a ação começa quando a função `reset()` é chamada. A ação termina quando o estado do robô é alterado para 'done' (feito).

Vamos entender cada parte do código:

```python
class Acao(Node,): # Mude o nome da classe

    def __init__(self):
        super().__init__('acao_node') # Mude o nome do nó
        self.timer = None

        self.robot_state = 'done' # Comece em 'done' - reset iniciará a ação
        self.state_machine = { # Adicione quantos estados forem necessários
            'acao': self.acao,
            'stop': self.stop,
            'done': self.stop
        }

        # Inicialização de variáveis
        # ...

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        # ...
```

Nesta parte, definimos o nome do nó, definimos a máquina de estados do robô (no dicionário `self.state_machine`), o estado inicial (no atributo `self.robot_state`) e criamos os publishers e subscribers necessários, por padrão, o publisher do `cmd_vel` já está criado.

```python
    def reset(self):
        self.twist = Twist()
        self.robot_state = 'acao' # Inicie a ação
        if self.timer is None:
            self.timer = self.create_timer(0.25, self.control) # Timer para o controle
        ### Iniciar variaveis da ação
```

Nesta parte, definimos a função `reset()`, essa função deve ser chamada para iniciar a ação. Ela inicializa 
1. a variável `self.twist`,
2. define o estado do robô para o estado inicial da ação (pode mudar para a sua ação)
3. inicia o timer, `self.timer` que chama a função `control()` a cada 0,25 segundos.
4. Por fim, você deve inicializar as variáveis necessárias para a ação.

A partir desse ponto, vamos seguir uma máquina de estado padrão, até o fim da ação.

```python
    def stop(self):
        self.twist = Twist() # Zera a velocidade
        print("Parando o robô.")
        self.timer.cancel() # Finaliza o timer
        self.timer = None # Reseta a variável do timer
        self.robot_state = 'done' # Ação finalizada
```

Ao final da ação, o estado do robô deve ser alterado para `stop`, o estado `stop` deve parar o robô e cancelar o timer. E então, o estado do robô deve ser alterado para `done`, indicando que a ação foi concluída.

```python
    def control(self): # Controla a máquina de estados - eh chamado pelo timer
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]() # Chama o método do estado atual 
        self.cmd_vel_pub.publish(self.twist) # Publica a velocidade
```

Outra função importante é a `control()`, que é chamada pelo timer a cada 0,25 segundos. Ela imprime o estado atual do robô e chama o método correspondente ao estado atual do robô na máquina de estados (`self.state_machine[self.robot_state]()`). Por fim, ela publica a velocidade do robô no tópico `cmd_vel`.
**A função control deve ser a única função que publica no tópico `cmd_vel`**. Isso é importante para garantir que o robô não receba comandos conflitantes.


## Nó Base de Controle

Neste script, definimos a máquina de estados do robô (no dicionário `self.state_machine`), o estado inicial (no atributo `self.robot_state`) e a função de controle (no método `self.control()`), que a cada 0,25 segundos:

1. Executa a função `self.state_machine[self.robot_state]()`, que executa a função correspondente ao estado atual do robô.
2. Publica a ação de controle no tópico `cmd_vel` (`self.cmd_vel_pub.publish(self.twist)`).

!!! importante
    A função de controle **é a única função que publica no tópico `cmd_vel`**. Isso é importante para garantir que o robô não receba comandos conflitantes.
    
    Portanto, a função de controle não precisa ser alterada.