# Resolução de Exercício - Visão Computacional

Nesta atividade, vamos resolver uma questão da prova de visão de 2023-B.

## Questão 1 (3,5)

Essa questão consiste em identificar bandeiras de vários países em imagens. Um processo anterior já removeu o fundo, então só nos resta dizer qual é qual. Iremos analisar os seguintes países. Veja na pasta `img/q1` exemplos de todas essas bandeiras nas imagens de teste.

1. Mônaco
2. Peru
3. Singapura
4. Irlanda
5. Itália

Você deverá editar a classe `IdentificadorBandeiras` do arquivo [q1.py](/docs/modulos/05-visao-p2/atividades/util/q1.py) para realizar essa questão. Nesta classe, você deve implementar a função `run` para identificar as bandeiras. A função `run` recebe uma imagem e modifica a variável da classe `self.bandeiras` que é uma lista de tuplas no formato

```
(PAIS, (x1, y2), (x2, y2)`)
```

onde

- `PAIS` é uma string com o nome do país tratado (em minúsculas e sem espaços). Se você não conseguiu identificar uma bandeira, pode retornar uma lista.
- `(x1, y1)` é o ponto do topo esquerdo do retângulo em que a bandeira está inserida
- `(x2, y2)` é o ponto baixo direito do retângulo em que a bandeira está inserida

**A ordem dos elementos da lista não é importante, apenas seu conteúdo**

Os critérios de avaliação são os seguintes:

* **1,0** encontrou o canto de todas as bandeiras
* **1,5** identificou uma bandeira e acertou seus cantos em todas imagens de testes do arquivo `test_simples.py`
* **+0.4** para cada bandeira acertada corretamente em todas as imagens de testes do arquivo `test_simples.py`
* **+0.4** escrever o nome do país corretamente na imagem.

<p>
<details>
<summary>Resultados Esperados</summary>

1. teste1.png:
```python
[
    ('singapura', (192, 496), (456, 673)),
    ('monaco', (726, 163), (983, 369)),
    ('peru', (119, 121), (380, 295)),
],
```
2. teste2.png
```python
[
    ('irlanda', (705, 589), (970, 722)),
    ('italia', (343, 298), (607, 474)),
],
``` 
3. teste3.png
```python
[
    ('peru', (751, 445), (1012, 619)),
    ('singapura', (125, 261), (390, 437)),
],
``` 
4. teste4.png
```python
[
    ('peru', (767, 496), (1028, 671)),
    ('italia', (84, 477), (348, 653)),
    ('irlanda', (752, 114), (1017, 246)),
],
```