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
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_6`.
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

# Exercício 2 - Segue Linha (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `segue_linha.py` com um nó denominado `seguidor_node`, que faça com que robô **real** siga a linha amarela do chão. O nó deve:

* O nó deve ter estados, `centraliza` e `segue` e `para`.

* Adicione um subscriber, que se inscreve no tópico de imagem **comprimida** e direciona para a função `image_callback`.

* A função `image_callback` deve filtrar a faixa amarela na pista e armazenar o centro da linha na variável `self.x`, `self.y` e a largura da imagem na variável `self.w`.

* A função `image_callback` só deve executar se a variável `self.running` for `True`.

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

____________________________________________________________________

# Exercício 3 - Aproxima Creeper (3 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie dois arquivos, um chamado `aproxima.py` com um nó denominado `aproxima_node` que, faça com que o robô **real** se aproxime do creeper e outro arquivo chamado `filtro_cor.py` com um nó denominado `filtro_cor_node` que filtra a cor do creeper e publica uma mensagem do tipo **geometry_msgs/Point** com a **posição do creeper na imagem (x, y) e a largura da imagem (z)**.
O nó `aproxima_node` deve:

* Ter três estados, `segue`, `centraliza` e `stop`.

* No estado `centraliza`, o robô deve girar até encontrar o creeper da cor selecionada.

* No estado `segue`, o robô deve se aproximar do creeper.

* No estado `stop`, o robô deve parar quando estiver a menos de `0.5m` do creeper.

* Se o creeper for retirado da frente do robô, ele deve voltar para o estado `centraliza`.

O nó `filtro_cor_node` deve:

* Receber a cor do creeper como `string` no parâmetro `cor`.

* Os limites HSV devem ser definidos em um dicionário, `self.cores` no método `__init__`, com os nomes das cores como chave e um segundo dicionário com as chaves `inferior` e `superior` com os valores dos limites inferior e superior da cor no espaço HSV.

* Filtrar a cor do creeper

* Publicar uma mensagem do tipo **geometry_msgs/Point** com,

    * **x:** a posição do creeper na imagem.

    * **y:** a posição do creeper na imagem.

    * **z:** a largura da imagem.

**Dica:** leia a documentação oficial: [geometry_msgs/msg/Point](https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html).

## Critérios de Avaliação:

1. Desenvolveu o `filtro_cor_node` para filtra e publicar as informações do creeper.
2. Desenvolveu o nó `aproxima_node` com os comportamentos corretos.
3. Não utiliza nenhuma função de `sleep` e `while` no código. Com exceção do `sleep` para "dar boot" no robô.
4. **Vídeo:** Mostra o robô executando o comportamento e se aproximando de **2** creepers. Assim que ele parar a menos de `0.5m` do primeiro creeper, tire o creeper da frente do robô e coloque outro do outro lado do robô a uma distância de `2m`.
5. Repita o processo para cada cor de creeper, ou seja, devem ser gravados **3 vídeos**.
