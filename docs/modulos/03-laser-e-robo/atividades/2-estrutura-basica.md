# Estrutura Básica de um Nó

Para facilitar o desenvolvimento, fornecemos uma **estrutura base** de um nó da ROS 2, uma estrutura base para **nós de ação** e uma estrutura base para **nós de controle** ou clientes da ação.

* Nó base: [base.py](../util/base.py)
* Nó de ação base: [base_action.py](../util/base_action.py)
* Nó "cliente" da ação: [base_control.py](../util/base_control.py)

Baixe os arquivos e coloque-os em uma pasta de fácil acesso.

Agora, vamos entender o papel de cada parte.

## Nó Base

Este script não traz novidades em relação ao que já vimos, com **duas exceções** importantes:

1. `rclpy.spin_once(self)` - processa **uma vez** a fila de callbacks (mensagens recebidas, timers, etc.), atualizando variáveis intermediárias antes do próximo passo de controle.
2. `self.create_timer(0.25, self.control)` - cria um **timer** que chama `control()` a cada **0,25 s**.

!!! dica
    O **Nó Base** é útil para módulos que **assinam** um tópico, processam os dados e **publicam** o resultado em outro tópico (ex.: um nó de visão computacional).

## Nó Base de Ação

Este script define uma estrutura para ações com **início, meio e fim**. A ação inicia quando `reset()` é chamada e termina quando o estado chega a `'done'`.

```python
class Acao(Node): # Mude o nome da classe

    def __init__(self, node = 'node_name_here'): # Mude o nome do nó
        super().__init__(node)
        self.timer = None

        self.robot_state = 'done' # Comece em 'done' - reset iniciará a ação
        self.state_machine = { # Adicione quantos estados forem necessários
            'acao': self.acao,
            'stop': self.stop,
            'done': self.stop,
        }

        # Inicialização de variáveis
        # ...

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        # ...
```

Nesta parte, passamos para o construtor o **nome do nó** e iniciamos a **máquina de estados** (`self.state_machine`), o **estado inicial** (`self.robot_state`) e os **publishers/subscribers** necessários, por padrão, o publisher do `cmd_vel` já está definido.

**Quando mais de um nó forem utilizar a mesma ação, será necessário mudar o nome do nó para algo mais específico.**

```python
    def reset(self):
        self.twist = Twist()
        self.robot_state = 'acao' # Inicie a ação
        if self.timer is None:
            self.timer = self.create_timer(0.25, self.control) # Timer para o controle
        ### Iniciar variaveis da ação
```

Nesta parte, definimos a função `reset()`, essa função deve ser chamada para iniciar a ação. A função `reset()` faz o seguinte:

1. Inicializa a variável `self.twist`,
2. Define o estado do robô para o estado inicial da ação (mude para o estado inicial da sua ação)
3. Inicia o timer, `self.timer` que chama a função `control()` a cada 0,25 segundos.
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

Ao final da ação, o estado do robô deve ser alterado para `stop`. Este estado deve:

1. Parar o robô;
2. Cancelar o timer;
3. Em seguida, mudar para o estado `'done'` (finalização).

```python
    def control(self): # Controla a máquina de estados - eh chamado pelo timer
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]() # Chama o método do estado atual 
        self.cmd_vel_pub.publish(self.twist) # Publica a velocidade
```

Outra função importante é a `control()`, que é chamada pelo timer a cada 0,25 segundos. Ela imprime o estado atual do robô e chama o método correspondente ao estado atual do robô na máquina de estados (`self.state_machine[self.robot_state]()`). Por fim, ela publica a velocidade do robô no tópico `cmd_vel`.

!!! importante
    **A função control deve ser a única função que publica no tópico `cmd_vel`**. Isso é importante para garantir que o robô não receba comandos conflitantes.

## Nó "Cliente" da Ação

Este script é um **gerenciador** que pode ou não chamar a ação definida no nó de ação. Normalmente, ele é o **nó principal** do pacote que decide quando iniciar/parar uma ação.

Vamos estudar as partes relevantes:

```python
class BaseControlNode(Node): # Mude o nome da classe
    def __init__(self):
        super().__init__('node_name_here') # Mude o nome do nó
        # Outra Herança que você queira fazer
        self.acao_node = Acao() # Cria o nó da Acao

        self.robot_state = 'acao'
        self.state_machine = {
            'acao': self.acao, # Estado para GERENCIAR a ação
            'done': self.done
        }

        self.estados_clientes = ['acao'] # Coloque aqui os estados que são "cliente de ação".

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

        ## Por fim, inicialize o timer
        self.timer = self.create_timer(0.1, self.control)
```

Diferenças chave em relação ao nó de ação:

1. **Não cancelamos** o timer do gerenciador (ele roda o tempo todo).
2. Instanciamos e guardamos os nós auxiliares (ex.: `self.acao_node`).
3. Armazenamos os estados que são "cliente de ação" em `self.estados_clientes`, nestes estados, este nó não publicará no `cmd_vel`, para não conflitar com o nó de ação.

Função que **aciona** a ação:

```python
    def acao(self):
        if self.acao_node.robot_state == 'done': # Se a ação NÂO FOI INICIADA
            print("\nIniciando [ACAO]...")
            rclpy.spin_once(self.acao_node) # Processa as callbacks uma vez
            self.acao_node.reset() # Reseta o nó para iniciar a ação

        rclpy.spin_once(self.acao_node) # Processa os callbacks e o timer

        if self.acao_node.robot_state == 'done': # Se a ação FOI FINALIZADA
            self.acao_node.control() # Garante que o robo é parado antes de finalizar a ação
            print("[ACAO] Finalizada.")
            self.robot_state = 'done' # Muda para o próximo estado da máquina de estados
```
Essa função:

1. primeiro verifica se a ação já foi iniciada, caso contrário, inicia a ação chamando `reset()` do nó de ação.
2. Em seguida, processa os callbacks do nó de ação (incluindo o timer) chamando `rclpy.spin_once(self.acao_node)`.
3. Por fim, verifica se a ação foi finalizada internamente, caso afirmativo, garante que o robô é parado chamando `self.acao_node.control()` e muda o estado para o próximo estado da máquina de estados, nesse caso, `'done'`.

No estado anterior, note a ação só será iniciada uma vez e só passará para o próximo estado quando a ação for finalizada internamente, ou seja, quando o nó de ação mudar seu estado para `'done'`. Assim, o controle da ação é delegado para o nó de ação, enquanto o nó "cliente" apenas gerencia quando iniciar/parar a ação.

Agora, no estado `'control'` temos uma alteração:

```python
    def control(self): # Controla a máquina de estados - eh chamado pelo timer
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]() # Chama o método do estado atual 
        if self.robot_state not in self.estados_clientes: # Se o estado atual não é um estado "cliente de ação"
            self.cmd_vel_pub.publish(self.twist) # Publica a velocidade
```

Aqui, o nó "cliente" só publicará no `cmd_vel` se o estado atual **não for um estado cliente de ação**. Isso é importante para evitar conflitos de publicação entre o nó "cliente" e o nó de ação, garantindo que o nó de ação tenha controle total sobre o `cmd_vel` durante a execução da ação.