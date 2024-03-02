# APS 2 - Visão Computacional

**Importante:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 1:** Não modifique o arquivo de teste, `test.py`.

**Aviso 2:** Lembre-se de dar commit e push no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

## Teste seu código

Para testar seu código, execute o  teste automático usando o comando abaixo no terminal:

```bash
pytest test.py
```

Para testar apenas um exercício:

```bash
pytest test.py::test_ex1
```

Caso aparece um erro de `ModuleNotFoundError`, execute o comando abaixo no terminal:

```bash
pip install pytest
```

Se discordar do resultado do teste, verifique se seu código está seguindo o que foi pedido, leia com atenção o enunciado e a saída do teste. Se ainda assim discordar, entre em contato com os professores ou outro membro da equipe da disciplina.

Alguns exercícios possuem um resultado esperado que você pode usar para conferir o seu resultado.
___


# Exercício 1

Você deve ter uma folha com o padrão da imagem abaixo.

**Dica:** Se não tiver, é possível fazer também com um tablet ou *smartphone*
 
<img src="fig/folha_atividade.png" width=300>

Neste exercício vamos aprender a fazer uma conversão 2D->3D, ou seja, estimar a distância da câmera até objetos capturados na imagem. Para isso, relembre o modelo pinhole visto em aula.

<img src="fig/pinhole.png" width=60%>

A partir da geometria do modelo pinhole, podemos definir a seguinte relação entre a distância focal $f$, o tamanho do objeto virtual $h$, a distância da câmera ao objeto $D$, e o tamanho do objeto real $H$:

$$
\frac{h}{H} = \frac{f}{D}
$$

O objetivo deste exercício é estimar a distância $D$ da **sua** câmera até a folha e o ângulo $\theta$ de inclinação da folha em relação a vertical. Vocês vão trabalhar no arquivo [./ex1.py](./ex1.py), ajuste a classe para herdar o módulo de visão e implemente o método `run`. Algumas funções já foram criadas para ajudar vocês, vocês podem criar outras funções se acharem necessário, desde que não modifiquem as entradas e saídas das funções já existentes.

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
4. Siga o tutorial do [link](https://insper.github.io/robotica-computacional/aps/screen_record/) para aprender como gravar a tela do seu linux.
5. Grave um vídeo mostrando a sua câmera e a imagem da folha, mostre a distância $D$ e o ângulo $\theta$ na imagem. No vídeo você deve mostrar a folha em diferentes distâncias $D$ e ângulos $\theta$.
6. De upload do vídeo no youtube e coloque o link no README.md do seu repositório.

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

A imagem [./img/calib01.jpg](./img/calib01.jpg) serve de exemplo para calibrar a sua câmera, essa imagem também é utilizada pelo pytest para verificar se o seu código está correto.

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
___


# Exercício 2 - Desafio

Neste exercício você deve modificar o código do exercício 5 da atividade 5 do módulo 2 para um vídeo do robô real no link abaixo.

[link do video](https://insper-my.sharepoint.com/:v:/g/personal/diegops_insper_edu_br/EVNzpavCn6NPqMfgV0f9X_0Bcbn4SGEHJuudx7W54dJLFQ?e=j6adG7&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZyIsInJlZmVycmFsQXBwUGxhdGZvcm0iOiJXZWIiLCJyZWZlcnJhbE1vZGUiOiJ2aWV3In19)

[link atividade 5](https://insper.github.io/robotica-computacional/modulos/02-modelo-de-camera/atividade5/)

Uma vez que seu código passar no pytest, você deve mudar a `main` para rodar a função `rodar_video`.
Além do que foi pedido na atividade 5, você deve:

1. Mostrar na imagem a reta que melhor se ajusta aos segmentos da pista amarela;
2. Escrever na imagem a inclinação da reta em relação a vertical;
3. Calcular a distância, na horizontal, do segmento de pista mais perto do robô até o centro da imagem.
4. Mostrar na imagem essa distância através de uma linha horizontal.
5. Escreva na imagem a distância calculada.
6. Grave um vídeo mostrando o seu código funcionando em TODO o percurso do robô.
7. De upload do vídeo no youtube e coloque o link no README.md do seu repositório.