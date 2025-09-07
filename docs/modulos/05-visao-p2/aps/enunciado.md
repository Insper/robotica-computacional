# Entregável 5 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para este entregável, você deve utilizar o robô real, mas você pode testar o código no simulador.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_5`.
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `robcomp_util` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

____________________________________________________________________

# Exercício 0 - Organização & Qualidade (1 pontos)
Este exercício está avaliando a organização e qualidade dos vídeos dos exercícios 2, 3 e 4, do desafio e do arquivo `README.md`.

## Critérios de Avaliação:
* **Vídeo:** O vídeo foi gravado na **horizontal**.
* **Vídeo:** O vídeo foi gravado em um ambiente bem iluminado.
* **Vídeo:** O audio está claro e sem ruídos, se desejar, remova o audio e adicione uma música de fundo.
* **Vídeo:** Na descrição do vídeo no Youtube, está descrito o que o robô está fazendo.
* **Vídeo:** Pelo vídeo, é possível entender o que o robô está fazendo.
* **README.md:** O link do vídeo está correto e foi adicionado no campo específico.
* **README.md:** O arquivo README.md tem o nome completo e o email de todos os integrantes do grupo.____________________________________________________________________

# Baixe o zip com as imagens de teste
Clique no link abaixo para baixar o zip com as imagens de teste dos exercícios 1 e 2.
!!! download
    [Download das Imagens de Teste](fig_aps5.zip)

# Exercício 1 - Latinhas (3 pontos)
Resolva o exercicio na atividade [2 - Identificação de Objetos](https://insper.github.io/robotica-computacional/modulos/05-visao-p2/atividades/2-identificacao/).

Mesmo não sendo um nó da ROS 2, adicione o arquivo `latinhas.py` no diretório `entregavel_5` do seu repositório juntamente com as imagens de teste.

Grave um vídeo da execução do código nas 3 imagens de teste e adicione o link no arquivo `README.md` do seu repositório.

---

# Exercício 2 - Vanishing Point (3 pontos)

Resolva o exercicio no final da atividade [4 - Detecção de Retas e Círculos](https://insper.github.io/robotica-computacional/modulos/05-visao-p2/atividades/4-retas-circulos/).

Procure uma nova imagem da internet, salve no seu repositório e rode o código para detectar os pontos de fuga (vanishing points) na nova imagem.

Salve a imagem com o ponto de fuga detectado e adicione no seu repositório.

---

# Exercício 3 - Estimando Pose (4 pontos)

Você deve ter uma folha com o padrão da imagem abaixo.

**Dica:** Se não tiver, é possível fazer também com um tablet ou *smartphone*
 
<img src="fig/folha_atividade.png" width=300>

Neste exercício vamos aprender a fazer uma conversão 2D->3D, ou seja, estimar a distância da câmera até objetos capturados na imagem. Para isso, relembre o modelo pinhole visto em aula.

<img src="fig/pinhole.png" width=60%>

A partir da geometria do modelo pinhole, podemos definir a seguinte relação entre a distância focal $f$, o tamanho do objeto virtual $h$, a distância da câmera ao objeto $D$, e o tamanho do objeto real $H$:

$$
\frac{h}{H} = \frac{f}{D}
$$

O objetivo deste exercício é estimar a distância $D$ da **sua** câmera até a folha e o ângulo $\theta$ de inclinação da folha em relação a vertical. 

Voce vai trabalhar no arquivo [./ex3.py](./ex3.py). Algumas funções já foram criadas para ajuda-lo, mas voce pode criar outras funções se achar necessário.

## Este exercício pede que vocês façam o seguinte:

1. Utilizar a sua câmera para capturar uma imagem da folha em uma distância $D$ conhecida;
2. Medir a distância $H$ entre os dois círculos da folha;

Na função `run` você deve fazer o seguinte, chamando as respectivas funções:

1. Encontrar o centro dos dois círculos da folha - `encontrar_centros`;
2. Encontrar a distância $h$ entre os dois círculos da folha* - `calcular_h`;
3. Calcular o ângulo $\theta$ de inclinação da folha em relação a vertical - `calcular_theta`;
4. Estimar a distância $D$ - `encontrar_D`;
5. Escrever na imagem o valor da distância $D$ e do ângulo $\theta$, utilize apenas duas casas decimais**.
6. Retornar a imagem com as informações escritas, a distância $D$, o ângulo $\theta$ e a distância $h$.
7. O seu código deve ser robusto para funcionar caso a folha não esteja na imagem - retorne o valor `-1` para a distância $D$ e o ângulo $\theta$.

Na função `calibration` você fará a calibração da sua câmera, para isso você deve fazer o seguinte:

1. Modificar a imagem selecionada para a da sua câmera - `rodar_frame`;
2. Chamar a função `run` para encontrar os centros dos círculos;
3. Chamar a função `encontrar_foco` com os valores medidos para $D$ e $H$ para encontrar a distância focal $f$ da sua câmera - a variável deve ser armazenada como parâmetro da classe.

Uma vez que seu código passar no pytest, você deve mudar a `main` para rodar a função `rodar_webcam`. Nessa função você deve fazer o seguinte:

1. Modificar a imagem selecionada para rodar na sua câmera - `rodar_webcam`;
2. Chamar a função `calibration` para calibrar a sua câmera;
3. Dentro do loop da webcam, chamar a função `run` para encontrar a distância $D$ e o ângulo $\theta$ da folha;
4. Grave um vídeo mostrando a sua câmera e a imagem da folha, mostre a distância $D$ e o ângulo $\theta$ na imagem. No vídeo você deve mostrar a folha em diferentes distâncias $D$ e ângulos $\theta$.
5. De upload do vídeo no youtube e coloque o link no README.md do seu repositório.

<p>
<details>
<summary>Spoiler*</summary>

Distância entre o centro dos círculos: $h = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2}$

</details>
</p>

<p>
<details>
<summary>Spoiler**</summary>

Utilize a função `cv2.putText` para escrever na imagem.

</details>
</p>

## Valores Esperados

A imagem [./img/calib01.jpg](./img/calib01.jpg) de como você deveria tirar uma foto de calibração. Utilizando a sua camera, tire uma foto similar a essa, com a folha posicionada a uma distância $D$ conhecida.

<img src="./img/calib01.jpg" width=300>

Pela legenda, a distância $D$ entre a câmera e a folha é de 80 cm. A distância $H$ entre os dois círculos é de 12.7 cm.

<p>
<details>
Saída<summary>Saída Esperada</summary>

* Distância entre os círculos = 161

* Distância focal = 1014.1732283464568

</details>
</p>

As imagens [./angulo01.jpg](./img/angulo01.jpg), ..., [./angulo04.jpg](./img/angulo04.jpg) servem de exemplo para estimar o ângulo $\theta$ de inclinação da folha em relação a vertical.

<p>
<details>
<summary>Saída Esperada</summary>

* angulo01.jpg: Ángulo de -0.18 graos

* angulo02.jpg:  ngulo de -51.98 graus

* angulo03.jpg:  ngulo de -88.93 graus

* angulo04.jpg:  ngulo de 118.57 graus

</details>
</p>