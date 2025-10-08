# Segue Linha

Agora que aprendemos o conceito de controle proporcional, vamos implementar a **Ação Segue Linha** para que o robô siga a linha amarela do controle de forma suave e precisa.

## Conceito

Relembrando o controle proporcional: a saída é ajustada **proporcionalmente ao erro**.

```python
rot = K_p * erro
```

* `rot` é a velocidade angular do robô (ex.: `Twist.angular.z`),
* `K_p` é a constante proporcional,
* `erro` erro da medição.

### Como calcular o erro para seguir a linha:
Para andar em direção a um alvo, o alvo deve estar **centralizado** na horizontal na imagem, ou seja, o **x do centróide do alvo** deve ser igual ao **x do centro da imagem**. Com isso em mente o erro é a distância horizontal (x) entre o centro da imagem e o centróide do segmento amarelo.

### Como calcular na prática:
No `image_callback`, após detectar o centróide do segmento amarelo:

* Guarde `self.cx_linha` (x do centróide) e **`self.w` = metade da largura da imagem**.
* Se não houver linha, defina `self.erro = None`.
* Se houver linha, calcule o erro:

  * `self.erro = ???` (em pixels)
  * Normalize o erro ≈ `[-1, 1]` dividindo por `self.w` 

Então no estado `segue`, use o controle proporcional para definir a velocidade angular. Defina o ganho `self.kp` testando valores e considerando o limite da velocidade angular (+- 0.5 rad/s, por exemplo).

Cuidado com o sinal do erro! Pense quando o robô deve girar para a esquerda ou para a direita.

## Prática 1

Baseando-se no codigo [Nó Base de Ação](https://insper.github.io/robotica-computacional/modulos/03-laser-e-robo/util/base_action.py) e [Nó Base de Visão](https://insper.github.io/robotica-computacional/modulos/06-visao-p3/util/image_subscriber.py), implemente a **Ação Segue Linha** com o seguindo os passos:

1. A Ação Segue Linha não deve ter fim (ou seja, o robô deve seguir a linha até o Cliente finalizar).
1. Mude o nome da classe para `SegueLinha` e o nome do nó para `seguelinha_node`.
2. Mude também a chamada da classe na função `main()` para `SegueLinha()`.
3. Mude o nome do estado de `acao` para `seguir`.
4. No construtor `__init__()`, adicione as variáveis necessárias.
4. Na função `reset()`, 
    * Mude o estado do robô para `seguir`.

5. Na função `seguir` (antiga função `acao`), 
    * Calcule a velocidade angular com controle proporcional: `self.twist.angular.z = self.kp * self.erro`.
    * Defina uma velocidade linear constante.

6. No. `image_callback`:
   * Siga os passos definidos no inicio do handout para calcular `self.erro`.


---
