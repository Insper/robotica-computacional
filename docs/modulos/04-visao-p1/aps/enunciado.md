# Entregável 4 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

## Configuração do Pacote (ROS 2)

- Não será necessário para este entregável.

____________________________________________________________________

# Exercício 1 - Conversão 2D->3D (5 pontos)

Você deve ter uma folha com o padrão da imagem abaixo.

**Dica:** Se não tiver, é possível fazer também com um tablet ou *smartphone*
 
<!-- <img src="fig/folha_atividade.png" width=300> -->
![folha_atividade](fig/folha_atividade.png)

Neste exercício vamos aprender a fazer uma conversão 2D->3D, ou seja, estimar a distância da câmera até objetos capturados na imagem. Para isso, vamos ver o modelo pinhole.

<!-- <img src="fig/pinhole.png" width=60%> -->
![pinhole](fig/pinhole.png)

A partir da geometria do modelo pinhole, podemos definir a seguinte relação entre a distância focal $f$, o tamanho do objeto virtual $h$, a distância da câmera ao objeto $D$, e o tamanho do objeto real $H$:

$$
\frac{h}{H} = \frac{f}{D}
$$

O objetivo deste exercício é estimar a distância $D$ da **SUA** câmera até a folha.

Vocês vão trabalhar no arquivo [./ex1.py](./ex1.py).

## Este exercício pede que vocês façam o seguinte:

1. Na função `run` você deve fazer o seguinte:

    1.1. Converter a imagem para o modelo de cor HSV;

    1.2. Obter as máscaras para os círculos `ciano` e `magenta`;
    
    1.3. Calcular a area dos círculos `ciano` e `magenta`;

    1.4. Se a diferença entre as áreas for maior do que 20000 (pode alterar esse valor) retorne a distância $D$ entre a câmera e a folha como `-1`. Caso contrário, calcule a média das áreas e o diâmetro do círculo.

    1.5. Escreva na imagem o valor da distância $D$ e o diâmetro do círculo, utilize apenas duas casas decimais.

2. Para a imagem `calib01.jpg`, vamos fazer o processo de calibração da camera. Utilize o valor da distância $D$ entre a câmera e a folha descrito na imagem para calcular a distância focal $self.f$ da câmera do professor (valor esperado é ~363).

3. Para a imagem `teste01.jpg`, utilize a distância focal $self.f$ para calcular a distância $D$ entre a câmera e a folha. (valor esperado é ~41 cm).

4. Agora repita o processo de calibração para a sua câmera, tirando uma foto da folha a uma distância $D$ conhecida.

5. Mude a função `main` para rodar a função `rodar_webcam` e faça um **vídeo** mostrando a sua câmera e a imagem da folha, mostre a distância $D$ e o diâmetro do círculo na imagem.

6. Adicione o link do vídeo no README.md do seu repositório.

__________

# Exercício 2 - Linha Amarela e Cruzamento (5 pontos)

Neste exercício você vai trabalhar no arquivo [./ex2.py](./ex2.py).

Primeiramente, baixe o vídeo no link abaixo e salve na pasta `img/q2` do seu repositório. 

[LINK DO VIDEO](https://insper-my.sharepoint.com/:v:/g/personal/diegops_insper_edu_br/EVNzpavCn6NPqMfgV0f9X_0Bcbn4SGEHJuudx7W54dJLFQ?e=j6adG7&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZyIsInJlZmVycmFsQXBwUGxhdGZvcm0iOiJXZWIiLCJyZWZlcnJhbE1vZGUiOiJ2aWV3In19)

Na função `main` altere entre testar com um frame, `rodar_frame`, ou rodar com o vídeo, `rodar_video`, comentando e descomentando a linha apropriada.

Você deve fazer o seguinte:

1. Filtrar a cor amarela do frame e binarizar a imagem, mostre a mascara em uma janela.

2. Ajuste os valores da mascara para que apenas a linha amarela seja detectada **em praticamente todos os frames**.

3. Corte a imagem em 3 colunas e calcule a area da linha amarela em cada coluna.

4. Se duas colunas tiverem área maior do que `valor` (você deve definir esse valor), então você deve escrever na imagem "Curva Detectada".

5. Se três colunas tiverem área maior do que `valor` (você deve definir esse valor), então você deve escrever na imagem "Cruzamento Detectado".

6. Ajuste cuidadosamente os valores para que o seu programa não detecte curvas e cruzamentos onde não existem (falso positivo) e que detecte corretamente onde existem (verdadeiro positivo).

7. Faça um vídeo mostrando a execução do seu programa e adicione o link do vídeo no README.md do seu repositório.