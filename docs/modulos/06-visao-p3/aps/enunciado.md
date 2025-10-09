# Entregável 6 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para este entregável, você deve utilizar o robô real, mas você pode testar o código no simulador.

## Atualização do Pacote (ROS 2)

Execute os comandos abaixo para atualizar os pacotes da `ros2` obrigatórios para a prova:

```bash
cd ~/colcon_ws/src/my_simulation
git stash
git pull
cb
```

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_6`.
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `robcomp_util` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

---

# Exercício 0 - Organização & Qualidade (0 pontos)
Este exercício está avaliando a organização e qualidade dos vídeos dos exercícios 2, 3 e 4, do desafio e do arquivo `README.md`.

## Critérios de Avaliação:
* **Vídeo:** O vídeo foi gravado na **horizontal**.
* **Vídeo:** O vídeo foi gravado em um ambiente bem iluminado.
* **Vídeo:** O audio está claro e sem ruídos, se desejar, remova o audio e adicione uma música de fundo.
* **Vídeo:** Na descrição do vídeo no Youtube, está descrito o que o robô está fazendo.
* **Vídeo:** Pelo vídeo, é possível entender o que o robô está fazendo.
* **README.md:** O link do vídeo está correto e foi adicionado no campo específico.
* **README.md:** O arquivo README.md tem o nome completo e o email de todos os integrantes do grupo.

---

# Exercício 1 - Detector Perigo (5 pontos)
Resolva o exercicio na atividade [2 - Detecção de Objetos Complexos com Redes Neurais](https://insper.github.io/robotica-computacional/modulos/06-visao-p3/atividades/2-id_com_NN/).

Grave um vídeo da execução do código no robô **simulado** e adicione o link no arquivo `README.md` do seu repositório.

---

# Exercício 2 - Identifica Creeper (5 pontos)
Resolva o exercicio na atividade [3 - Pose e Transformação Coordenada Usando Marcadores AprilTag](https://insper.github.io/robotica-computacional/modulos/06-visao-p3/atividades/3-reconhecimento-marcadores/).

Grave um vídeo da execução do código no robô **real** e adicione o link no arquivo `README.md` do seu repositório.

---

# Desafio - Centraliza no Creeper (+1 ponto)

Modifique o código do exercício 2 para que o robô centralize em um creeper escolhido, além de um video do robô executando a tarefa, será nescessário mostrar o funcionamento do código para um dos professores durante a aula. Será aceito demonstrações do desafio até a aula de segunda-feira dia 13/10/25.
