# Robótica Computacional 2023.1

## ACEITE A ATIVIDADE NO LINK: https://classroom.github.com/a/NAM0IG8k

EMAIL: ______________

Observações de avaliações nesta disciplina:

* Inicie a prova no Blackboard para a ferramenta de Proctoring iniciar. Só finalize o Blackboard quando enviar a prova via Github classroom
* Durante esta prova vamos registrar somente a tela, não a câmera nem microfone
* Ponha o nome no enunciado da prova no Github
* Você pode consultar a internet ou qualquer material que usamos no curso, mas não pode se comunicar com pessoas ou colegas a respeito da prova. Também não pode usar ferramentas de **IA** como chatGPT ou Github Copilot durante a prova.
* Faça commits e pushes frequentes no seu repositório
* Avisos importantes serão dados na sala da prova
* Permite-se consultar qualquer material online ou próprio. Não se pode compartilhar informações com colegas durante a prova.
* Faça commits frequentes. O primeiro a enviar alguma ideia será considerado autor original
* A responsabilidade por ter o *setup* funcionando é de cada estudante
* Questões de esclarecimento geral podem ser perguntadas
* É vedado colaborar ou pedir ajuda a colegas ou qualquer pessoa que conheça os assuntos avaliados nesta prova.

## Questão 1 (3,5)

Essa questão consiste em identificar bandeiras de vários países em imagens. Um processo anterior já removeu o fundo, então só nos resta dizer qual é qual. Iremos analisar os seguintes países. Veja na pasta `q1/img` exemplos de todas essas bandeiras nas imagens de teste.

1. Mônaco
2. Peru
3. Singapura
4. Irlanda
5. Itália

Você deverá editar a classe `IdentificadorBandeiras` do arquivo `q1.py` para realizar essa questão. Nesta classe, você deve implementar a função `run` para identificar as bandeiras. A função `run` recebe uma imagem e modifica a variável da classe `self.bandeiras` que é uma lista de tuplas no formato


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

</details>
</p>

<p>
<details>
<summary>Resposta</summary>

[q1](q1/q1_gab.py)

</details>
</p>

## Questão 2 (3,5)


Estamos criando um programa de supermercado para verificar se os produtos estão posicionados corretamente. Cada produto é marcado por uma etiqueta retangular. Para garantir que os produtos estão posicionados corretamente precisamos checar se sua etiqueta está na orientação correta e se estão na prateleira correta.


- Etiqueta amarela: primeira prateleira, posição vertical


- Etiqueta magenta: segunda prateleira, posição horizontal


Nas imagens analisadas, a primeira prateleira está na metade de cima da imagem. Você deve implementar a função `PrateleiraArrumada` para devolver quatro números


- número de produtos na prateleira de cima - (`self.prateleira_cima`)


- número de produtos que estão arrumados (prateleira correta e orientação correta) na prateleira de cima - (`self.prateleira_cima_arrumada`)


- número de produtos na prateleira de baixo - (`self.prateleira_baixo`)


- número de produtos que estão arrumados (prateleira correta e orientação correta) na prateleira de baixo - (`self.prateleira_baixo_arrumada`)


Os critérios de avaliação são os seguintes:


- **1,5** Conta quantos objetos de interesse em cada prateleira corretamente para todas imagens de teste e salva nas variáveis da classe `self.prateleira_cima` e `self.prateleira_baixo`;


- **1,0** Identifica o número de produtos que estão arrumados (prateleira correta e orientação correta) na prateleira de cima, para todas imagens de teste e salva na variável `self.prateleira_cima_arrumada`;


- **1,0** Identifica o número de produtos que estão arrumados (prateleira correta e orientação correta) na prateleira de baixo, para todas imagens de teste e salva na variável `self.prateleira_baixo_arrumada`




<p>
<details>
<summary>Resultados Esperados</summary>


1. teste1.png - respectivamente: 4, 4, 3, 3


2. teste2.png - respectivamente: 3, 1, 2, 1
   
3. teste3.png - respectivamente: 3, 2, 2, 1


4. teste4.png - respectivamente: 3, 1, 3, 1


</details>
</p>

<p>
<details>
<summary>Resposta</summary>

[q2](q2/q2_gab.py)

</details>
</p>

## Questão 3 (3,0)


Iremos desenvolver um sistema de monitoramento para um hotel de pets e precisamos garantir que animais não compatíveis fiquem perto demais uns dos outros.


- gatos estão em perigo se ficarem perto de outros gatos e de cachorros

- pássaros podem ficar juntos ou perto de cachorros, mas não de gatos


**Um animal está próximo de outro se seus centros estão a menos de 300 pixels de distância**


Nesta questão você deve:


1. **0,5** - Carregar a MobileNet - para isso copie os arquivos necessários.

2. **0,6** - A função `processar_animais` processa a saída da mobilenet e modifica o atributo da classe `self.animas`, que é um dicionário no formato abaixo.

```python
{
    'passaros': [(x1, y1, x2, y2),  ....] ,# lista de posições em que foram encontrados pássaros
    'cachorros': [(x1, y1, x2, y2),  ....] ,# lista de posições em que foram encontrados cachorros
    'gatos': [(x1, y1, x2, y2),  ....]# lista de posições em que foram encontrados gatos
}
```
3. **1,5** - crie uma função `checar_perigos` que, dado um dicionário de animais como mostrado acima, salva no dicionário `self.perigo` somente os animais em perigo. Ele deve estar no mesmo formato acima, mas conter somente os animais que satisfaçam o critério de proximidade. A função `run` deve escrever no terminal os animais em perigo e retornar o dicionário de animais em perigo.

4. **0,4** - escrever **PERIGO** sobre os animais em perigo na imagem de saída.

<p>
<details>
<summary>Resultados Esperados</summary>

1. teste1.png:
```python
{
    'passaros': [],
    'cachorros': [],
    'gatos': [],
}
```

2. teste2.png:
```python
{
    'passaros': [],
    'cachorros': [],
    'gatos': [(373, 397, 605, 734)],
}
```
   
3. teste3.png:
```python
{
    'passaros': [(748, 238, 965, 519)],
    'cachorros': [],
    'gatos': [],
}
```
</details>
</p>

<p>
<details>
<summary>Resposta</summary>

[q3](q3/q3_gab.py)

</details>
</p>
