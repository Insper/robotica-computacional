# Estrutura Básica de um Nó

Para facilitar o desenvolvimento de um nó, a fornecemos uma estrutura básica de um nó da ROS 2 em python. Também fornecemos uma estrutura básica de um nó que controla o robô, incluindo a definição da maquina de estados, a função de controle e o publisher do `cmd_vel`.

* Nó base: [base.py](../util/base.py)

* Nó de ação base [base_action.py](../util/base_action.py)

* Nó "cliente" da ação: [main.py](../util/main.py)

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


## Nó "Cliente" da Ação

Neste script, apresentamos uma estrutura básica para um nó que chama a ação definida no nó base de ação. 
Geralmente, esse nó é o nó principal do seu pacote, que inicia a ação e decide quando a ação deve ser iniciada ou parada.

Vamos estudar as partes relevantes:

```python
class BaseControlNode(Node): # Mude o nome da classe

    def __init__(self):
        super().__init__('base_control_node') # Mude o nome do nó
        self.timer = self.create_timer(0.1, self.control)

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
```
Essa parte é bem similar ao nó base de ação, mas com algumas diferenças importantes:
1. Não vamos cancelar o timer, portanto já iniciamos o timer no construtor do nó.
2. Vamos criar um todos os nós necessários para a ação, e atribuí-los a variáveis de instância, como `self.acao_node`.

Em seguida, vamos definir a função cliente da ação :
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
Essa função é responsável por
1. iniciar a ação, chamando o método `reset()` do nó de ação 
2. Processar os callbacks da ação uma unica vez
3. Aguardar até que a ação esteja concluída.
4. Quando a ação é concluída, o estado do robô é alterado dependendo da lógica do sistema, nesse caso vamos simplesmente mudar para 'stop', finalizando a execução do robô.

# Ação de Mover

Nesta atividade vamos aprender nossa primeira ação, a ação de andar uma distancia `d`.
Como estudamos em `Explorando Tópicos e Mensagens`, a velocidade linear do robo pode ser controlada se publicarmos uma mensagem de um certo tipo para um tpoico, mas, em robotica, muitas vezes queremos mover uma distacia conhecidada, mas como fazer isso apenas com a velocidade?

Em robotica uma forma de controlar o robo para se deslocar uma distancia `d` é utilizando um metodo conechecido como `Dead Reckoning`, Neste metodo, você se desloca em **velocidade constante** por um **tempo fixo**, sem receber feedback de quanto, realmente, se deslocou.

Para desenvolver essa ação, vamos começar do codigo base de ação, que podera ser utilizada como uma etapa inicial para açoes.

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
