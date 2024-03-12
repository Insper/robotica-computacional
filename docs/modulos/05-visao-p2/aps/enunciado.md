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
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `my_package` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

____________________________________________________________________

# Exercício 1 - Segue Linha (5 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `segue_linha.py` com um nó denominado `seguidor_node`, que faça o robô **real** siga a linha amaerla do chão. O nó deve:

* O robô deve iniciar no começo da pista, seguindo reto na primeira bifurcação.

* Depois o robô deve seguir e virar a esquerda na trifurcação, dar a volta, seguir reto na trifurcação e retornar para o começo da pista.

* Possui pelo menos três estados, `procura`, `forward` e `turn`.

* No estado `procura`, o robô deve girar até encontrar a linha amarela.

* No estado `forward`, o robô deve seguir a linha amarela.

* No estado `turn`, o robô deve virar quando for necessário.

## Critérios de Avaliação:

1. Nó filtra corretamente a imagem da câmera para encontrar a linha amarela.
2. Desenvolveu o nó `seguidor_node` com os comportamentos corretos.
3. Não utiliza nenhuma função de `sleep` e `while` no código. Com exceção do `sleep` para "dar boot" no robô.
4. Armazenou a posição inicial do robô.
5. Navega corretamente pela pista e retorna para o começo.
5. **Vídeo:** Mostra o robô executando o comportamento e navegando pela pista na trajetória correta.
6. **Vídeo:** O robô não colide com nenhum obstáculo.
7. **Vídeo:** Link do vídeo do robô em ação no Youtube.

____________________________________________________________________

# Exercício 2 - Aproxima Creeper (5 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie dois arquivos, um chamado `aproxima.py` com um nó denominado `aproxima_node` que, faça com que o robô **real** se aproxime do creeper e outro chamado `filtro_cor.py` com um nó denominado `filtro_cor_node` que filtra a cor do creeper e publique uma mensagem do tipo **geometry_msgs/Point** com a **posição do creeper na imagem (x, y) e a largura da imagem (z)**.
O nó `aproxima_node` deve:

* Ter três estados, `forward`, `procura` e `stop`.

* No estado `procura`, o robô deve girar até encontrar o creeper da cor selecionada.

* No estado `forward`, o robô deve se aproximar do creeper.

* Quando o robô estiver a menos de `0.5m` do creeper, ele deve parar.

O nó `filtro_cor_node` deve:

* Filtrar a cor do creeper

* Uma mensagem do tipo **geometry_msgs/Point** com,

    * **x:** a posição do creeper na imagem.

    * **y:** a posição do creeper na imagem.

    * **z:** a largura da imagem.


## Critérios de Avaliação:

1. Desenvolveu o `filtro_cor_node` para filtra e publicar as informações do creeper.
2. Desenvolveu o nó `aproxima_node` com os comportamentos corretos.
3. Não utiliza nenhuma função de `sleep` e `while` no código. Com exceção do `sleep` para "dar boot" no robô.
4. **Vídeo:** Mostra o robô executando o comportamento e se aproximando de **2** creepers. Assim que ele parar a menos de `0.5m` do primeiro creeper, tire o creeper da frente do robô e coloque outro do outro lado do robô a uma distância de `2m`.