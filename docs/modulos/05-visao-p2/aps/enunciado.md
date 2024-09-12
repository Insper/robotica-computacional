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

# Exercício 1 - Latinhas (2 pontos)
Resolva o exercicio na atividade [2 - Identificação de Objetos](https://insper.github.io/robotica-computacional/modulos/05-visao-p2/atividades/2-identificacao/).

Mesmo não sendo um nó da ROS 2, adicione o arquivo `latinhas.py` no diretório `entregavel_5` do seu repositório juntamente com as imagens de teste.

Grave um vídeo da execução do código nas 3 imagens de teste e adicione o link no arquivo `README.md` do seu repositório.

____________________________________________________________________

# Exercício 2 - Segmenta Linha (3 pontos)
Baseando-se no código `image_subscriber.py` do capítulo 5, crie um arquivo chamado `segmenta_linha.py` com um nó denominado `segmenta_linha_node`, que segmenta a linha amarela do chão e imprime a posição do segmento de linha mais próximo ao robô **real** na imagem.

* Adicione um subscriber que se inscreva no tópico de imagem **comprimida** e direcione para a função `image_callback`.

* A função `image_callback` deve filtrar a faixa amarela na pista e armazenar o centro da linha mais próximo nas variáveis `self.x`, `self.y`, e a metade da largura da imagem na variável `self.w`.

* A função `image_callback` deve ser executada apenas se a variável `self.running` for `True`.

* Exiba a imagem com o contorno da máscara amarela e o centro do segmento de linha mais próximo ao robô.

* Calcule também a distância do centro da linha ao centro da imagem.

* Caso o robô não veja nenhum contorno, retorne o centro como `(-1,-1)`, ou seja,`self.x = -1`, `self.y = -1`. Nesse caso, ainda deve mostrar a imagem sem nada desenhada.

## Critérios de Avaliação:

1. O nó filtra corretamente a imagem da câmera para encontrar a linha amarela.
2. O nó `segmenta_linha_node` foi desenvolvido com os comportamentos corretos.
3. **Vídeo:** Mostre o robô **real** executando o comportamento; para isso, controle o robô com o teclado e exiba as imagens e o terminal com as informações.
4. **Vídeo:** Em algum momento, vire o robô para que ele não encontre nenhum contorno.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.

