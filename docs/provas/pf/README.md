# Robótica Computacional 2023.2 - Prova Final

# Link Classroom: https://classroom.github.com/a/pd_ZnlxU

2+2 = __

# *Nome* = ______________________________________


## Observações gerais

- Coloque o `nome` no enunciado da prova no Github;
- Você pode consultar a internet ou qualquer material utilizado ao longo do curso;
- **`NÃO`** pode se comunicar com pessoas ou colegas a respeito da prova. 
- **`NÃO`** pode usar ferramentas de **IA** como chatGPT ou Github Copilot durante a prova;
- Faça commits e pushes frequentes no seu repositório;
- Avisos importantes serão dados na sala da prova;
- A responsabilidade por ter o `setup` funcionando é de cada estudante, isso inclui Notebook, SSD com a infra instalada e configurada com Linux, Gazebo, ROS etc;
- O entendimento do enunciado faz parte da avaliação;
- Esse bullet point é um teste de atenção - escreva a resposta da soma acima como 42;
- **`ATENÇÃO`** Os exercícios exigem a gravação de um vídeo do robô executando a tarefa - lembre-se de reservar tempo para isso. Haverá penalização se o vídeo não for entregue ou se o vídeo não corresponder ao código entregue;
- A nota do desafio é somada apenas se o aluno atingir a nota máxima da questão e completar o desafio;

## Instruções para a prova

- Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/aps/screen_record/), e deve ser postado no Youtube. Ao final de cada questão existe o `Link do Vídeo` onde deve ser disponibilizado o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `Não de commit no vídeo`, somente no link.

- **`ATENÇÃO`** É necessário clonar este repositório dentro do diretório `~/catkin_ws/src`. Execute os comandos abaixo, substituindo `<REPO>` por seu repositório da PF.

```bash
cd ~/catkin_ws/src
git clone <REPO>
cd scripts
chmod a+x *.py
cd ~/catkin_ws/src/mybot_description
git pull
cd ~/catkin_ws/src/my_simulation
git pull
cd ~/catkin_ws
catkin_make
```

## Dicas:

1. Leia com atenção os enunciados e critérios de correção;
2. Leia todo o código de cada questão, incluindo os comentários;
3. Grande parte do desafios já foram discutidos e realizados em projetos ou APS. Você pode (e deve!) reaproveitar partes dos trabalhos desenvolvidos ao longo do curso;
4. Lembre-se de adicionar qualquer arquivo relevante para execução das questões tais como nós da ROS, módulos, etc.


## Questão 1 (3,0 + 1,0)

Nesta questão usaremos o cenário `roslaunch my_simulation pista_circulos.launch` e vamos trabalha no arquivo `q1.py`:

![](circulos.png)

Seu robô deverá ter os seguintes comportamentos:

1. Segue reto até entrar na pista vermelha;
2. Completa duas voltas na pista vermelha no sentido horário;
3. Continua caminhando pela pista vermelha até encontrar a pista verde;
4. Completa duas voltas na pista verde no sentido anti-horário;
5. Continua caminhando pela pista vermelha até encontrar a pista azul;
6. Completa duas voltas na pista azul no sentido horário e para.

Os sentidos estão indicados na imagem acima. Será aceito pequenas variações de trajetória, mas o robô deve seguir a pista corretamente. O ponto onde o robô começa a andar na pista é a **referência** para o início da volta.

Os seguintes critérios de correção serão usados:

- **R0)(até +1,0)**  Completa as duas voltas na pista vermelha no sentido horário;
- **R1)(até +1,0)**  Realiza **R0** e completa as duas voltas na pista verde no sentido anti-horário;
- **R2)(até +1,0)**  Realiza **R1** e completa as duas voltas na pista azul no sentido horário e `para` corretamente.

- **Bonus (+1,0) DESAFIO:** No arquivo `q1_des.py` faça o robô seguir infinitamente a trajetória descrita na imagem abaixo. Será aceito apenas pequenas variações de trajetória.

![](circulos_desafio.png)

### Link do Vídeo
Rode seu arquivo e grave um vídeo do robô seguindo executando a tarefa. Coloque o link do vídeo no espaço abaixo.

- Link do vídeo (q1.py): ______________
- Link do vídeo (bonûs desafio): ______________


## Questão 2 (3,0 + 1,0)

Nesta questão usaremos o cenário `roslaunch my_simulation reuniao.launch` e vamos trabalha no arquivo `q2.py`:

![](reuniao.png)

Nosso robô está em `berserk!` e deverá ter o seguinte comportamento:

1. Procura um creeper girando ao redor até localizá-lo;
2. Move-se em direção ao creeper e para a uma distância segura;
3. Emprega a garra para neutralizar o creeper;
4. Repete os `passos 1, 2 e 3` até que **sobre apenas um 1** os creeper;
5. O robô se aproxima do último creeper, pega ele e então **arremessa** o creeper, soltando o creeper enquanto sobe ou desce o braço;
6. Ao finalizar a eliminação de todos os creepers, exibe a mensagem `FIM` no terminal e encerra a operação.

Os seguintes critérios de correção serão usados:

- **R0)(até 0,5)** Capacidade de girar e neutralizar com êxito um único creeper;
- **R1)(até +1,0)**  Realiza **R0** e tem habilidade de gira e neutralizar mais de um creeper, embora não consiga eliminar todos;
- **R2)(até +1,0)**  Realiza **R0** e tem eficiência em neutralizar todos os creepers com sucesso e exibir a mensagem `FIM` no terminal;
- **R3)(até +0,5)** Realiza **R2** de forma autônoma, ou seja, sem intervenção externa para deletar do cenário os creepers que já foram derrubados.


* **Bonus (+1,0) DESAFIO:** No mesmo arquivo `q2.py`, o robô recebe uma lista de cores e deve derrubar todos os creepers somente daquela cor na ordem enviada. 

Por exemplo:

- Se a lista for `['ciano', 'amarelo', 'verde']`, o robô deve derrubar primeiro os todos os creepers ciano, depois todos os amarelos e por fim todos os verdes (Video desafio 1). 
- Se for enviado uma lista `['amarelo', 'ciano']` o robô deve derrubar primeiro todos os amarelos e depois todos os cianos e ignorar os creepers de outras cores (Video desafio 2). 
- Se for enviado uma lista vazia, o robô deve seguir o comportamento normal e derrubrar os creepers até faltar um. 
- Arremessa o último creeper.
- Ao final, o robô deve imprimir `FIM` no terminal independentemente do caso executado e parar.

### Link do Vídeo
Rode seu arquivo e grave um vídeo do robô seguindo executando a tarefa. Coloque o link do vídeo no espaço abaixo.

- Link do vídeo (parte 1): ______________
- Link do vídeo (desafio 1): ______________
- Link do vídeo (desafio 2): ______________






