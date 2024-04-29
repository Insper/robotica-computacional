# Projeto - Missão na 404

O objetivo deste projeto, realizado no contexto da disciplina, é programar nosso robô para cumprir uma série de missões na sala 404. Utilizaremos o cenário apresentado abaixo.

![](pista.jpeg)

Os grupos, compostos por quatro integrantes, deverão trabalhar juntos utilizando o repositório do GitHub Classroom disponibilizado. 

[Link para tarefa do GH Classroom](https://classroom.github.com/a/NkmSeO1t){ .ah-button }

# **Data de Entrega: 01/12 23:59**

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas os conceitos vocês **deveram gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link de cada video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para o projeto, as entregas são feitas utilizando o robô real. Entregas no simulador serão aceitas, conquistando no máximo **25%** da diferença de notas entre o último conceito com o robô real.

**Aviso 6:** O conceito **C** é o mínimo para aprovação e **deve ser feito com o robô real**.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `projeto_robcomp`.
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `my_package` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

## Atualização dos Repositórios

```bash
cd ~/catkin_ws/src/mybot_description
git pull
cd ~/catkin_ws/src/my_simulation
git pull
cd ~/catkin_ws
catkin_make
```

## Mapa Simulado

```bash
roslaunch my_simulation pista23-B.launch
```
-----------------------------------------
## Descrição das Missões

-----------------------------------------


!!! people "Contribuições"
    - ![Diego](equipe/diego.jpg) **Diego Pavan Soler** *Professor*
    - ![Arnaldo](equipe/arnaldo.jpeg) **Arnaldo Alves Viana Junior** *Prof. Auxiliar*
    - ![Rogério](/robotica-computacional/equipe/rogerio.jpeg) **Rogério Cuenca** *Técnico de lab*
    - ![Igor](/robotica-computacional/equipe/igor.jpg) **Igor Montagner** *Professor-23a*


