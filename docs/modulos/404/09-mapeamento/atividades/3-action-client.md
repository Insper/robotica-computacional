
# ROS2 Action Server

## Introdução ao Action Client
Um **Action Client** no ROS2 é uma abstração que permite a execução de tarefas de longa duração que precisam de monitoramento e controle. Ele é utilizado para enviar objetivos (goals) para um Action Server e receber feedback e resultados. Além disso, o Action Client permite o cancelamento de objetivos em execução.


## Action Client Base
Para auxiliar no desenvolvimento de Action Client, o arquivo (client_base.py )[../util/client_base.py] fornece uma classe base que simplifica a criação de Action Client. A classe `BaseActionClientNode` é uma subclasse de `rclpy.node.Node` e fornece métodos para lidar com a comunicação entre o Action Server e o cliente.

### Métodos da Classe `BaseActionClientNode`
A classe `BaseActionClientNode` fornece os seguintes métodos para lidar com a comunicação entre o Action Server e o cliente:
- O construtor da classe recebe as seguintes informações:

    - `node`: O nó do Action Server.
    - `action_type`: O tipo da ação que será registrada no sistema ROS2.
    - `action_name`: Nome da ação que será registrada no sistema ROS2, semelhante ao nome de um tópico.

- O construtor da classe inicializa o Action Server `ActionClient()` que recebe:

    - `self`: O nó do Action Server.
    - `action_type`: O tipo da ação que será registrada no sistema ROS2.
    - `action_name`: Nome da ação que será registrada no sistema ROS2, semelhante ao nome de um tópico.

## Client Segue Linha
Para exemplificar o uso de um Action Client, fornecemos uma versão simplificada do nó `segue_linha` que utiliza um Action Client para seguir uma linha amarela no chão, no arquivo [client_segue_linha.py](../util/client_segue_linha.py). O Action Client `SegueLinhaActionClient` recebe um objetivo booleano que indica se o robô deve seguir a linha ou não.

Note como a função `control` e não publica comandos para o robô quando o Action server está em execução. Isso evita que o robô receba comandos conflitantes.

Observe também a função `feedback_callback` que foi implementada para receber feedback do Action Server.
```python
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback received: {feedback_msg.feedback}')
```
Ela poderia tomar alguma ação com base no feedback recebido, mas nesse caso, ela apenas imprime o feedback recebido e é o estado `waiting_for_result` que verifica se a volta foi concluída e então cancela o objetivo.

```python
    ...
    elif self.hora_de_parar is True and dist < 0.5:
        self.get_logger().info('Cancelando objetivo, o robô completou uma volta.')
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()  # Envia o comando para cancelar o objetivo
        self.robot_state = 'stop'
```

# Atividade

Agora, baseado-se nas atividades anteriores, crie um Action Server e um Action Client para ação de navegar até um ponto específico. O Action Server deve criar um ação do tipo `GoToPoint` (`robcomp_interfaces`) com o nome `goto_point`. O Action Client deve enviar um objetivo para o Action Server com as coordenadas do ponto que o robô deve navegar.

!!! dica
    Verifique a estrutura de uma ação do tipo `GoToPoint` no robcomp_interfaces