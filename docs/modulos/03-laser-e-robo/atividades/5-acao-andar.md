# Ação de Andar

Neste handout vamos criar nossa primeira ação: A ação de **andar uma distância** `d`.

Como estudamos em [Explorando Tópicos e Mensagens](https://insper.github.io/robotica-computacional/modulos/02-intro-ros/atividades/2-topicos/), a velocidade linear do robo pode ser controlada publicando uma mensagem do tipo `Twist` em `cmd_vel`. Porém, muitas vezes queremos percorrer uma **distância conhecida**, mas como fazer isso apenas com a velocidade?

Em robotica uma estratégia comum é o método de **Dead Reckoning** que consiste em deslocar-se com **velocidade constante** por um **tempo fixo**, sem receber feedback de quanto, realmente, se deslocou.

Dentro do pacote `robcomp_util/robcomp_util`, baseando-se no codigo [Nó Base de Ação](../util/base_action.py), crie um arquivo denominado `andar.py` e uma classe chamada `Andar` e siga os seguintes passos:

1. Mude o nome da classe para `Andar` e o nome do nó para `andar_node`.
2. Mude também a chamada da classe na função `main()` para `Andar()`.
3. Mude o nome do estado de ``acao`` para `andar`.
4. No construtor `__init__()`, adicione e a variável `self.velocidade` com o valor de `0.2 m/s`.
4. Na função `reset()`, 
    * Modifique a definição da função `reset()` para receber um parâmetro `distancia` que define a distância a ser percorrida.
    * Calcule o tempo necessário, `self.threshold` para percorrer a distância com uma velocidade constante.
    * mude o estado do robô para `andar`.
    * Inicie a variável `self.tempo_inicial` como o seguinte:
    ```python
        self.tempo_inicial = self.get_clock().now().to_msg()
        self.tempo_inicial = float(self.tempo_inicial.sec)
    ```
5. Na função `andar` (antiga função `acao`), 
    * Adicione a velocidade linear do robô no pacote da mensagem `twist` da seguinte forma `self.twist.linear.x = self.velocidade`.
    * Calcule o tempo decorrido `self.dt` entre o tempo atual e o tempo inicial, e imprima esse valor.
    * Se o tempo decorrido for maior ou igual a `self.threshold` segundos, pare o robô, zere a velocidade na mensagem `self.twist.linear.x = 0.0`, e mude o estado do robô para `stop`.

A ideia do método `Dead Reckoning` é que você se desloca em velocidade constante por um tempo fixo, dessa forma, voce se deslocou uma distancia `d = v * t`, onde `v` é a velocidade e `t` é o tempo. Com isso em mente,

6. Adicione o nó no arquivo `setup.py` e então compile o pacote.
7. Teste esta ação, executando o nó no simuldor e verificando se o robô se move a distância correta.
