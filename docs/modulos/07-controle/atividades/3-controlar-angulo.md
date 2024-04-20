# Controlar Orientação com Controle Proporcional

Em muitas aplicações, é necessário controlar a orientação de um robô. Nesta atividade, vamos aprender a fazer este controle de forma suave e precisa utilizando o controle proporcional.

## Conceito

Relembrandos o conceito de controle proporcional, temos um sistema de controle que ajusta a saída proporcionalmente ao erro. No caso de controlar a orientação de um robô, o erro é a diferença entre a orientação desejada e a orientação atual do robô.

```python
rot = K_p * erro
```

Onde a variável `rot` é a velocidade angular do robô, `K_p` é a constante proporcional e `erro` é a diferença entre a orientação desejada e a orientação atual do robô.

## Prática

Com isso em mente, crie um novo nó que:

1. Tenha uma função chamada `get_goal_from_target` que receba um ângulo alvo, no intervalo de 0:2pi, converta-o para o intervalo de -pi:pi e o salve em uma variável `self.goal_yaw` - isso pode ser feito utilizando o código abaixo:

```python
self.goal_yaw = (ang + np.pi) % (2 * np.pi) - np.pi
```
0. Crie uma classe `RotateTo` que herda de `Node` e `Odom`.

1. O método `__init__` deve receber um ângulo alvo e chamar a função `get_goal_from_target` com esse ângulo.

2. O nó agora deve ter apenas dois estados: `gira` e `para`.

3. No estado `gira`, o robô calcular o erro entre a orientação atual e a orientação desejada.

4. Como o erro pode saturar, você tem duas opções - uma dessas opções garante que o robô sempre gire no sentido mais curto. Escolha uma das opções abaixo:
    1. Utilizar o operador `%` para limitar o erro entre 0 e 2pi.
    2. Utilizar a função `np.arctan2` para calcular o erro entre -pi e pi.

5. Utilize a fórmula do controle proporcional para calcular a velocidade angular do robô.

6. Mude para o estado `para` quando o erro for menor que um valor de tolerância.

7. **Desafio:** No código do quadrado, chame essa função e substitua o estado `girar` por um loop que chama a função `control`, do `RotateTo`, até que o estado `para` seja atingido.
