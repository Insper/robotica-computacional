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
    O **Nó Base** é útil para módulos que **assinam** um tópico, processam os dados e **publicam** o resultado em outro (ex.: um nó de visão computacional).

## Nó Base de Ação

Este script define uma estrutura para ações com **início, meio e fim**. A ação inicia quando `reset()` é chamada e termina quando o estado chega a `'done'`.

Trecho (simplificado):

```python
class Acao(Node,): # Mude o nome da classe

    def __init__(self):
        super().__init__('acao_node') # Mude o nome do nó
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

Nesta parte, definimos o **nome do nó**, a **máquina de estados** (`self.state_machine`), o **estado inicial** (`self.robot_state`) e os **publishers/subscribers** necessários, por padrão, o publisher do `cmd_vel` já está definido.

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
        super().__init__('base_control_node') # Mude o nome do nó
        # Outra Herança que você queira fazer
        rclpy.spin_once(self) # Roda pelo menos uma vez para pegar os valores
        self.acao_node = Acao() # Cria o nó da Acao

        self.robot_state = 'stop'
        self.state_machine = {
            'acao': self.acao, # Estado para GERENCIAR a ação
            'stop': self.stop
        }

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

Função que **aciona** a ação:

```python
    def acao(self):
        print("\nIniciando movimento de ação...")
        rclpy.spin_once(self.acao_node) # Processa as callbacks uma vez
        self.acao_node.reset() # Reseta o nó para iniciar a ação

        while not self.acao_node.robot_state == 'done': # Enquanto a ação não estiver finalizada
            rclpy.spin_once(self.acao_node) # Processa os callbacks e o timer

        # Quando a ação estiver finalizada, o 
        #   estado do robô deve ser alterado para o próximo estado ou finalizar mudando para 'stop'
        self.robot_state = 'stop'
```
Essa função:
1. inicia a ação, chamando o método `reset()` do nó de ação
2. Processa os callbacks da ação uma única vez
3. Aguarda até que a ação esteja concluída.
4. Quando a ação é concluída, o estado do robô é alterado dependendo da lógica do sistema, nesse caso vamos simplesmente mudar para `stop`, finalizando a execução do robô.

# Ação de Mover

Vamos criar a primeira ação: **andar uma distância** `d`.

Como estudamos em `Explorando Tópicos e Mensagens`, a velocidade linear do robo pode ser controlada publicando uma mensagem do tipo `Twist` em `cmd_vel`. Porém, muitas vezes queremos percorrer uma **distância conhecida**, mas como fazer isso apenas com a velocidade?

Em robotica uma estratégia comum é o método de **Dead Reckoning** que consiste em deslocar-se com **velocidade constante** por um **tempo fixo**, sem receber feedback de quanto, realmente, se deslocou.

Para desenvolver essa ação, vamos começar do codigo [base de ação](../util/base_action.py), que podera ser utilizada como uma etapa inicial para açoes.

Nesta atividade, faça o seguinte:
1. Mude o nome da classe para `Andar` e o nome do nó para `andar_node`.
2. Mude também a chamada da classe na função `main()` para `Andar()`.
3. Mude o nome do estado de ação para `andar`.
4. Na função `reset()`, 
    * mude o estado do robô para `andar` e inicie a variável `self.velocidade` com um valor de 0.2 m/s.
    * Inclua a variável `self.tempo_inicial` como o seguinte:
    ```python
        self.tempo_inicial = self.get_clock().now().to_msg()
        self.tempo_inicial = float(self.tempo_inicial.sec)
    ```
5. Na função `andar`(antiga função `acao`), 
    * defina a velocidade linear do robô na variável `twist` como `self.twist.linear.x = self.velocidade`.
    * Recolha o tempo atual e armazene em `self.tempo_atual`.
    * Calcule o delta de tempo entre o tempo atual e o tempo inicial, e imprima esse valor.
    * Se o delta for maior ou igual a 4 segundos, pare o robô (defina `self.twist.linear.x = 0.0`) e mude o estado do robô para `stop`.

A ideia do metodo `Dead Reckoning` é que você se desloca em velocidade constante por um tempo fixo, dessa forma, voce se deslocou uma distancia `d = v * t`, onde `v` é a velocidade e `t` é o tempo. Com isso em mente,

6. Modifique o codigo do andar para andar `2m`.
7. Teste esta ação, executando o nó no simuldor e verificando se o robô se move a distância correta.
