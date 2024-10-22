
# Action Server

## O que é um Servidor de Ação (Action Server)?

Um **Servidor de Ação** em ROS2 é uma estrutura que permite a execução de tarefas longas ou que precisam ser monitoradas e controladas, como o movimento de um robô para um ponto distante. Ele se comunica com um **Cliente de Ação**, que envia um objetivo (goal) ao servidor. O servidor de ação pode:
- Aceitar ou rejeitar um objetivo.
- Enviar feedbacks periódicos ao cliente sobre o progresso do objetivo.
- Permitir o cancelamento de uma tarefa em andamento.
- Concluir uma tarefa e enviar o resultado de volta ao cliente.

Os servidores de ação são usados quando há necessidade de feedback contínuo ou controle de cancelamento durante a execução de uma tarefa, diferentemente dos serviços ROS, que lidam com operações mais simples e imediatas.

## Explicação das Funções no Código de GoToActionServer

Este arquivo contém uma explicação de cada função no código para o `GoToActionServer`, utilizado em um curso de ROS2. O objetivo é mover o robô até um ponto específico usando o conceito de **Action Server**.

## Classe `GoToActionServer`

### `__init__(self)`
- **Função**: Inicializa o Action Server e seus parâmetros de controle.
- **Detalhes**: 
    - Configura o Action Server com callbacks para lidar com pedidos de objetivos, cancelamento e execução.
    - Define parâmetros de controle para o movimento do robô, como a velocidade máxima e os ganhos proporcionais para controle linear e angular.
    - Cria um publicador para enviar comandos de velocidade ao robô.

### `goal_callback(self, goal_request)`
- **Função**: Lida com a recepção de um novo objetivo.
- **Detalhes**: 
    - Aceita o objetivo recebido e imprime uma mensagem no logger indicando que o objetivo foi recebido.

### `handle_accepted_callback(self, goal_handle)`
- **Função**: Inicia a execução de um objetivo aceito.
- **Detalhes**: 
    - Associa o objetivo recebido ao `goal_handle` e inicia a execução.

### `cancel_callback(self, goal_handle)`
- **Função**: Lida com pedidos de cancelamento de objetivos.
- **Detalhes**: 
    - Cancela a execução do objetivo, interrompe o movimento do robô e publica um comando de velocidade zero.

### `execute_callback(self, goal_handle)`
- **Função**: Executa o movimento do robô até o ponto especificado no objetivo.
- **Detalhes**: 
    - Atualiza o estado do robô enquanto ele está se movendo em direção ao objetivo.
    - Finaliza quando o estado da máquina de estados é `'stop'` ou quando o ROS2 é interrompido.
    - Ao final, retorna o sucesso da operação.

### `get_angular_error(self)`
- **Função**: Calcula o erro angular entre a posição atual do robô e o ponto de destino.
- **Detalhes**: 
    - Calcula a diferença entre o ângulo atual do robô e o ângulo para o ponto objetivo, ajustando a velocidade angular do robô de acordo com esse erro.

### `center(self)`
- **Função**: Alinha o robô com a direção do ponto objetivo.
- **Detalhes**: 
    - Quando o erro angular é pequeno o suficiente, muda o estado do robô para `'goto'`, permitindo que ele avance em direção ao ponto.

### `goto(self)`
- **Função**: Move o robô em direção ao ponto objetivo.
- **Detalhes**: 
    - Controla a velocidade linear do robô para levá-lo até o ponto.
    - Se o robô estiver suficientemente perto do ponto objetivo, muda o estado para `'stop'`.

### `stop(self)`
- **Função**: Para o robô.
- **Detalhes**: 
    - Publica um comando de velocidade zero, garantindo que o robô pare.

### `control(self)`
- **Função**: Controla o estado atual do robô e publica os comandos de velocidade apropriados.
- **Detalhes**: 
    - Baseia-se na máquina de estados atual (center, goto, stop) para determinar o comportamento do robô.
    - Publica continuamente comandos de velocidade baseados no estado atual.

## Função `main(args=None)`
- **Função**: Inicializa o nó ROS2 e mantém o servidor de ações em execução.
- **Detalhes**: 
    - Inicializa o nó ROS2 e o servidor de ações, e mantém o nó em execução até que seja encerrado.
