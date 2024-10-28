
# ROS2 Action Server

## Introdução ao Action Server

Um **Action Server** no ROS2 é uma abstração que permite a execução de tarefas de longa duração que precisam de monitoramento e controle. Ele é utilizado quando o tempo de execução de uma tarefa pode variar, permitindo o envio de feedback contínuo ao cliente que solicitou a tarefa. Um exemplo clássico é mover um robô para um ponto específico, o que pode levar algum tempo e, durante esse período, é útil fornecer atualizações ao cliente ou até mesmo permitir o cancelamento da tarefa.

Os Action Servers são particularmente úteis em três cenários:

1. **Tarefas Longas**: A tarefa pode demorar mais do que uma simples chamada de serviço, como mover um robô para um destino ou realizar uma varredura de sensores.
2. **Feedback**: O cliente que requisita a ação pode precisar de informações contínuas sobre o progresso da tarefa.
3. **Cancelamento**: A tarefa pode ser cancelada antes de sua conclusão, e o sistema precisa tratar o cancelamento adequadamente.

### Componentes de um Action Server

O Action Server no ROS2 tem quatro partes principais:

1. **Objetivo (Goal)**: O cliente envia um objetivo (goal) para o Action Server, que representa a tarefa a ser realizada.
2. **Feedback**: O Action Server pode enviar feedback ao cliente sobre o progresso do objetivo.
3. **Resultado**: Quando o objetivo é alcançado (ou falha), o Action Server envia o resultado para o cliente.
4. **Cancelamento**: Um objetivo pode ser cancelado pelo cliente a qualquer momento, e o Action Server deve lidar com essa solicitação.

## Action Server Base
Para auxiliar no desenvolvimento de Action Servers, o arquivo (action_base.py )[../util/action_base.py] fornece uma classe base que simplifica a criação de Action Servers. A classe `BaseActionServer` é uma subclasse de `rclpy.node.Node` e fornece métodos para lidar com a comunicação entre o Action Server e o cliente.

### Métodos da Classe `BaseActionServer`
A classe `BaseActionServer` fornece os seguintes métodos para lidar com a comunicação entre o Action Server e o cliente:
- O construtor da classe recebe as seguintes informações:

    - `node`: O nó do Action Server.
    - `action_type`: O tipo da ação que será registrada no sistema ROS2.
    - `action_name`: Nome da ação que será registrada no sistema ROS2, semelhante ao nome de um tópico.

- O construtor da classe inicializa o Action Server `ActionServer()` que recebe:

    - `self`: O nó do Action Server.
    - `action_type`: O tipo da ação que será registrada no sistema ROS2.
    - `action_name`: Nome da ação que será registrada no sistema ROS2, semelhante ao nome de um tópico.
    - `execute_callback`: Função de callback que será chamada quando um objetivo é recebido. Essa função é responsável por executar a tarefa solicitada pelo cliente. Ela terá que ser implementada pelo usuário.
    - `goal_callback`: Função de callback que será chamada quando um objetivo é recebido. Essa função é responsável por processar o objetivo recebido.
    - `handle_accepted_callback`: Função de callback que será chamada quando um objetivo é aceito. Essa função é responsável por aceitar o objetivo recebido.
    - `cancel_callback`: Função de callback que será chamada quando um objetivo é cancelado. Essa função é responsável por cancelar a tarefa em execução.

## Action Segue Linha
Para exemplificar o uso de um Action Server, fornecemos uma versão simplificada do nó `segue_linha` que utiliza um Action Server para seguir uma linha amarela no chão, no arquivo [action_segue_linha.py](../util/action_segue_linha.py). O Action Server `SegueLinhaActionServer` recebe um objetivo booleano que indica se o robô deve seguir a linha ou não.

Note como a função `control` e o `image_callback` só rodam quando o objetivo é aceito e a váriavel `self._goal_handle` é válida. Isso permite que o robô pare de seguir a linha quando o objetivo é cancelado.

Observe também a função `execute_callback`
```python
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.robot_state == 'stop':
                break

        self._result_msg.success = True
        goal_handle.succeed()

        return self._result_msg
```

Essa função foi reescrita do `action_base.py`. Ela é responsável por executar a tarefa solicitada pelo cliente. No caso do `SegueLinhaActionServer`, a tarefa é seguir a linha amarela. A função roda até que o robô seja parado ou o objetivo seja cancelado. Quando o objetivo é concluído, a função retorna o resultado da tarefa, mas nesse caso, o robô não vai parar de seguir a linha até que o objetivo seja cancelado pelo cliente.