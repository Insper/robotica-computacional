# OLD
# Exercício 2 - Segue Linha (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `segue_linha.py` com um nó denominado `seguidor_node`, que faça com que robô **real** siga a linha amarela do chão. O nó deve:

* O nó deve ter estados, `centraliza` e `segue` e `para`.

* Adicione um subscriber, que se inscreve no tópico de imagem **comprimida** e direciona para a função `image_callback`.

* A função `image_callback` deve filtrar a faixa amarela na pista e armazenar o centro da linha mais próximo nas variáveis `self.cx`, `self.cy`, e a metade da largura da imagem na variável `self.w`.

* A função `image_callback` deve ser executada apenas se a variável `self.running` for `True`.

* Calcule também a distância do centro da linha ao centro da imagem.

<!-- # Calcular erro no callback -->

* Caso o robô não veja nenhum contorno, defina o centro como `(-1,-1)`, ou seja,`self.cx = -1`, `self.cy = -1`.

* o estado `centraliza` deve centralizar o robô no segmento de linha amarelo mais relevante.

* o estado `segue` deve fazer o robô seguir a linha amarela, se movendo para frente.

* O estado `para` deve ser chamado depois de completar uma volta na pista, e o robô deve parar.

**Dica:** Ao iniciar a execução do nó, armazene em uma variável a posição inicial do robô e compare com a posição atual para saber se o robô completou uma volta.

## Critérios de Avaliação:

1. Nó filtra corretamente a imagem da câmera para encontrar a linha amarela.
2. Desenvolveu o nó `seguidor_node` com os comportamentos corretos.
3. Não utiliza nenhuma função de `sleep` e `while` no código. Com exceção do `sleep` para "dar boot" no robô.
5. Navega corretamente pela pista.
5. **Vídeo:** Mostra o robô executando o comportamento e navegando por uma volta completa na pista e parando.
6. **Vídeo:** O robô não colide com nenhum obstáculo.
7. **Vídeo:** Link do vídeo do robô em ação no Youtube.

# Segue Linha

Agora que aprendemos o conceito de controle proporcional, vamos modificar o código do nosso seguidor de linha para que ele controle o ângulo do robô de acordo com a posição do centro de massa do segmendo de linha detectado.

Dessa forma o robô terá um comportamento mais suave e preciso ao seguir a linha.

## Material Necessário

Primeiramente, vamos começar do seu código do seguir de linha ou você pode baixar o gabarito ao lado <--.

## Conceito

Relembrando o conceito de controle proporcional, temos um sistema de controle que ajusta a saída proporcionalmente ao erro. No caso do seguidor de linha, o erro é a diferença entre a posição do centro de massa do segmento de linha detectado e o centro da imagem.

```python
rot = K_p * erro
```

Onde a variável `rot` é a velocidade angular do robô, `K_p` é a constante proporcional e `erro` é a diferença entre a posição do centro de massa do segmento de linha detectado e o centro da imagem.

## Prática

Com isso em mente, faça as seguintes modificações no código do seguidor de linha:

1. Adicione uma variável `self.kp` no método `__init__` para armazenar a constante proporcional - lembre-se de inicializá-la com um valor adequado (considere a largura da imagem e as limitações do robô) e remova o estado `centraliza`.

2. No estado `segue`, se `self.cx == np.inf`, o robô deve apenas girar para encontrar a linha.

3. No estado `segue`, defina uma velocidade linear constante e calcule o erro como a diferença entre a posição do centro de massa do segmento de linha detectado e o centro da imagem, por fim, utilize a fórmula do controle proporcional para calcular a velocidade angular do robô.

4. Agora teste o robô e ajuste o valor de `self.kp` para que o robô siga a linha de forma suave e precisa.

5. **DESAFIO:** Ajuste os valores de `self.kp` e o delay da chamada da função `self.control` para que o robô siga a linha o mais rápido possível sem perder a precisão.

6. Adicione um estado `stop` que pare o robô. Modifique o callback da câmera para não rodar quando o estado for `stop`.

