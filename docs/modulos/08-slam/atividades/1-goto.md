# GoTo (vai até coordenadas)

Quando o robô está em um ambiente conhecido, é possível fazer com que ele vá até uma coordenada específica. Para isso, é necessário que o robô saiba a sua posição e a posição desejada. Neste atividade, você irá criar um nó que faz o robô ir até uma coordenada específica.

## Geometria

Considere que o robô está em um plano cartesiano e que a posição do robô é dada por `(x, y)` e a posição desejada é dada por `(x2, y2)`.

![goto](goto.png)

Para fazer o robô ir até a posição desejada, devemos "mirar" o robô em direção ao ponto desejado e então fazer o robô se mover até o ponto. Para isso, devemos calcular o erro angular entre o `yaw` atual do robô e o ângulo `theta` formado entre o robô e o ponto desejado.

A distância entre o robô e o ponto desejado pode ser utilizado para controlar a velocidade do robô. Quanto mais perto do ponto, menor a velocidade.

## Comportamento

Para otimizar a trajetória do robô, é interessante que o robô gire até que ele esteja alinhado com o ponto desejado e então inicie o movimento. Para isso, o nó deve ter três estados, `center`, `goto` e `stop`.

* O estado `center` deve ser o estado inicial e faz o robô girar até que ele esteja alinhado com o ponto desejado.
* O estado `goto` deve utilizar controladores proporcionais para mover o robô até o ponto desejado (`kp_linear`) e ajustar a sua orientação (`kp_angular`).
* Quando chegar no ponto desejado, o robô deve entrar no estado `stop`.

## Prática 1: Ação GoTo

Baseando-se no codigo [Nó Base de Ação](https://insper.github.io/robotica-computacional/modulos/03-laser-e-robo/util/base_action.py) do módulo 3, crie uma ação em um arquivo chamado `goto.py`, com uma classe `GoTo` e com um nó denominado `goto_node`, que, dado uma posição, faça o robô **simulado** `=)` se mova ***precisamente*** para este ponto em qualquer posição. O nó deve:

1. Mude o nome da classe para `GoTo` e o nome do nó para `goto_node`.
2. Mude também a chamada da classe na função `main()` para `GoTo()`.
3. A classe `GoTo` deve herdar de `Node` e `Odom`.
4. A ação deve ter três estados, `center`, `goto` e `stop`.
5. No construtor `__init__()`, adicione as variáveis necessárias.
6. Na função `reset()`,
    * Deve receber uma variável `point` do tipo `Point`.
    * Salve o ponto em uma variável `self.point`.
    * Mude o estado do robô para `center`.
7. O estado `center` deve ser o estado inicial e faz o robô girar até que ele esteja alinhado com o ponto desejado.
 Quando chegar no ponto desejado, o robô deve entrar no estado `stop`.
8. Deve ter um função `get_angular_error` que primeiro calcula o angulo entre a posição atual e o ponto desejado `theta` e depois calcula o erro entre o angulo atual e o angulo desejado, ajustando o erro para o intervalo `[-pi, pi]`.
9. `get_angular_error` também deve calcular a distância entre o robô e o ponto desejado.
10. O estado `goto` deve fazer o robô se mover até o ponto desejado e parar quando estiver **BEM PERTO** do ponto.
11. Utilize duas constantes proporcionais, `self.kp_linear` e `self.kp_angular`, para controlar a velocidade linear e angular do robô.