# Estrutura Básica de um Nó

Para facilitar o desenvolvimento de um nó, a fornecemos uma estrutura básica de um nó da ROS 2 em python. Também fornecemos uma estrutura básica de um nó que controla o robô, incluindo a definição da maquina de estados, a função de controle e o publisher do `cmd_vel`.

* Nó base: [base.py](../util/base.py)

* Nó base de controle: [main.py](../util/base_control.py)

Baixe os arquivos e coloque-os em uma pasta de fácil acesso. 

Agora vamos entender o que cada parte do código faz.

## Nó Base

Este script, não existe novidade em relação ao que já vimos anteriormente. Quando executado, ele chama a função `control()` a cada 0,25 segundos. Este script é útil para criar um nó que se inscreve em um tópico e executa uma função a cada nova mensagem recebida, publicando uma mensagem em outro tópico.

!!! tip
    Nó Base é útil para desenvolver módulos que se inscrevem em um tópico e publica o resultado em outro tópico, como por exemplo, um nó de visão computacional.

## Nó Base de Controle

Neste script, definimos a máquina de estados do robô (no dicionário `self.state_machine`), o estado inicial (no atributo `self.robot_state`) e a função de controle (no método `self.control()`), que a cada 0,25 segundos:

1. Executa a função `self.check_danger()`, que verifica se o robô está em alguma forma de perigo, como colisão ou proximidade de obstáculos e atualiza o estado do robô de acordo.
2. Executa a função `self.state_machine[self.robot_state]()`, que executa a função correspondente ao estado atual do robô.
3. Publica a ação de controle no tópico `cmd_vel` (`self.cmd_vel_pub.publish(self.twist)`).

!!! importante
    A função de controle **é a única função que publica no tópico `cmd_vel`**. Isso é importante para garantir que o robô não receba comandos conflitantes.
    
    Portanto, a função de controle não precisa ser alterada.