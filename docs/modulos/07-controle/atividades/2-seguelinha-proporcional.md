# Segue Linha com Controle Proporcional

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

1. Adicione uma variável `self.kp` no método `__init__` para armazenar a constante proporcional - lembre-se de inicializá-la com um valor adequado (considere a largura da imagem e as limitações do robô).

2. No estado `segue`, se `self.x == np.inf`, o robô deve apenas girar para encontrar a linha.

3. No estado `segue`, defina uma velocidade linear constante e calcule o erro como a diferença entre a posição do centro de massa do segmento de linha detectado e o centro da imagem, por fim, utilize a fórmula do controle proporcional para calcular a velocidade angular do robô.

4. Agora teste o robô e ajuste o valor de `self.kp` para que o robô siga a linha de forma suave e precisa.

5. **DESAFIO:** Ajuste os valores de `self.kp` e o delay da chamada da função `self.control` para que o robô siga a linha o mais rápido possível sem perder a precisão.

6. Adicione um estado `stop` que pare o robô. Modifique o callback da câmera para não rodar quando o estado for `stop`.

