# Robótica Computacional 2025.2 - AF

Instruções para a Parte 1 da Avaliação Final:

* A prova tem duração de **4 horas**.
* Inicie a prova no Blackboard para a ferramenta do Smowl ser iniciada. 
* O Smowl é obrigatório durante toda a prova.
* Só finalize o Blackboard quando enviar a prova via Github Classroom incluindo o hash do último commit na resposta do Blackboard.
* Durante a prova vamos registrar, a camera, a tela, as páginas visitadas, os acessos online e os registro do teclado.
* Coloque seu `nome` e `email` no `README.md` do seu repositório.
* A prova deverá ser realizada de forma individual.
* Não é permitido consultar a internet, com exceção do site da disciplina, do site "Ferramenta para Ajuste de Máscaras", do `Blackboard` e do repositório da avaliação criado através do GitHub Classroom.
* `Não é permitido o uso de ferramentas de **IA** como chatGPT, Copilot, Gemini ou similares durante a prova`.
* `Não é permitido o uso de ferramentas colaborativas como Google Docs, Google Slides, ou similares durante a prova`.
* `Não é permitido o uso de ferramentas de comunicação como Discord, WhatsApp, Telegram ou similares durante a prova`.
* `Não é permitido o uso de editores de codigo com IA como Cursor ou Windsurf durante a prova, sendo permitido apenas o uso do **VSCode**`.
* `Não é permitido o uso do Copilot durante a prova. Então desative-o antes de iniciar a prova`.
* `Não é permitido o uso de redes sociais, fóruns ou plataformas de comunicação durante a prova`.
* Faça commits e pushes regularmente de sua avaliação.
* Eventuais avisos importantes serão realizados em sala durante a prova.
* Escreva a frase "robcompehlegal" como a resposta da soma no arquivo `README.md` como teste de sua atenção.
* A responsabilidade por *`infraestrutura`*, *`configurações`* e *`setup`* em funcionamento pleno, é de cada estudante.
* **SÓ SERÃO ACEITOS REPOSITÓRIOS DE ALUNOS QUE ASSINARAM A LISTA DE PRESENÇA.**

---
# IMPORTANTE
**Após finalizar a prova prática, feche o notebook e pegue com o professor ou monitor uma folha de avaliação para realizar a Parte 2 da Avaliação Final (Avaliação Escrita).**

## Avaliação Escrita - Parte 2

A avaliação escrita consiste em 2 questões dissertativas sobre um dos dois exercícios que você realizou na Parte 1 (Avaliação Prática). Nesta etapa, você deverá explicar o funcionamento do seu código, por meio de um diagrama ou uma lista numerada (como a que fizemos para o projeto final), detalhando os **estados**, **transições** e **ações** principais do seu nó.

Descreva apenas **o que foi implementado no seu código**, não o que você gostaria de ter implementado, ou o que estava no enunciado original.

A nota final AF será calculada da seguinte forma:
AF = (Nota Q1 Parte 1 * Nota Q1 Parte 2) + (Nota Q2 Parte 1 * Nota Q2 Parte 2)

## Rubrica da Avaliação Escrita - Parte 2
Cada questão será avaliada com a seguinte rubrica (0 a 1,05 pontos):
* [0] - O diagrama ou lista não representa o funcionamento do código.
* [0,5] - O diagrama ou lista representa parcialmente o funcionamento do código, mas os pontos principais estão ausentes ou incorretos.
* [1,0] - O diagrama ou lista representa o funcionamento do código.
* [1,05] - O diagrama ou lista representa o funcionamento do código, com detalhes adicionais que auxiliam na compreensão do funcionamento do código.

---
* **BOA PROVA!**

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

- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote chamado `avaliacao_af`.

    - **Dica:** Para utilizar os módulos desenvolvidos no capitulo 3, inclua o pacote `robcomp_util` e o pacote `robcomp_interfaces` como dependência do seu pacote, e então, importe como nos exemplos do capitulo 3.

---

# Exercício 0 - Organização & Qualidade
Este exercício avalia a organização e a qualidade dos vídeos dos exercícios e do arquivo `README.md`.

## Critérios de Avaliação:
* O pacote foi corretamente configurado.
* As dependências do pacote estão corretas.
* Os diretórios e arquivos estão organizados de forma adequada.
* Todos os scripts estão na pasta `avaliacao_af` dentro do pacote `avaliacao_af`.
* A configuração dos nós foi realizada corretamente.
* Os nós da ROS 2 foram executados utilizando o comando `ros2 run`.
* **Vídeo:** A ação do robô é claramente compreensível pelo vídeo.
* **README.md:** O link do vídeo foi adicionado corretamente no campo indicado.
* **README.md:** O arquivo `README.md` contém o nome completo e o e-mail do estudante.
---

# Exercício 1 - Segue Circulos (5)

Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `q1.py` contendo uma classe denominada `MudaPista`. Esta classe deve implementar um **nó** chamado `muda_pista_node`, responsável por fazer com que o robô **simulado** entre na linha vermelha e mude de pista conforme comando de um **Orquestrador**, seguindo as pistas na ordem: vermelha, verde e azul.

![pistas](figs/pistas.jpg)

Utilize o comando abaixo para iniciar o simulador no mapa da prova:
```bash
ros2 launch my_gazebo pista_circulos.launch.py
```

## O nó criado deve
1. avisar que está **pronto** ao iniciar.
2. aguardar o comando de mudar de pista do **Orquestrador**.
3. Ao receber o comando para mudar de pista pela primeira vez, o robô deve entrar na linha vermelha no sentido informado.
4. Ao receber o comando para mudar de pista, muda para a próxima pista na ordem (vermelha -> verde -> azul), sempre seguindo no sentido informado eternamente.
5. O robô pode andar diretamente até a próxima pista, ignorando as outras pistas no caminho.
6. Na pista azul, o robô deve ignorar comandos de mudança de pista e continuar seguindo a pista azul até completar uma volta completa, retornando diretamente para o ponto de início e enviar um comando de pronto novamente para o **Orquestrador**, repetindo o ciclo eternamente.

## Comunicação com o Orquestrador
* Publicar e assinar ao tópico `/controle` com o tipo `robcomp_interfaces.msg.MudaPista`.
* Ao iniciar, publicar **READY** no campo `status` com horário atual e **nome do aluno** nos campos apropriados.
* Ao receber o `status` **MUDAR_HORARIO** ou **MUDAR_ANTIHORARIO**, o robô deve mudar de pista conforme o sentido informado. como na imagens a seguir.
* Ao completar a volta na pista azul e retornar ao ponto inicial, publicar **READY** no campo `status` novamente, repetindo o ciclo.

![pistas](figs/mudar_horario.png)

![pistas](figs/mudar_antihorario.png)




## Requisitos

1. Deve existir o arquivo chamado `q1.py`.
2. O programa deve ser executado sem erros.
3. A classe deve se chamar `MudaPista`.
4. A implementação deve seguir a estrutura da classe `MudaPista`, conforme exemplo no `base_control.py`.
5. A função `control` deve ser a única a publicar no tópico `/cmd_vel`.
6. A função `control` deve ser idêntica à do arquivo `base_control.py`. Todas as decisões de controle devem ocorrer dentro dos nós, sem alterações na função `control`.
7. Não utilizar loops infinitos ou `sleep` durante o controle do robô.

## Rúbrica
1. O programa deve respeitar as restrições definidas.
2. Nota: +1,0 - [1] & o robô consegue enviar comandos para para o **Orquestrador** e consegue reconhecer os comandos de mudança de pista.
3. Nota: +1,0 - [2] & o robô consegue entrar na pista vermelha, seguindo-a corretamente e mudar para a pista verde conforme o comando recebido.
4. Nota: +2,0 - [3] & o robô consegue seguir o fluxo de mudança de pista corretamente, entrando na pista azul, dando a volta completa e retornando ao ponto inicial, reiniciando o ciclo, **mas ignora o sentido informado**.
5. Nota: +2,0 - [4] & o robô consegue seguir o fluxo de mudança de pista corretamente, entrando na pista azul, dando a volta completa e retornando ao ponto inicial, reiniciando o ciclo,**obedecendo o sentido informado**.

## Vídeo

Grave um vídeo mostrando que o robô é capaz de realizar o comportamento completo ou algum comportamento parcial. Publique os vídeos no YouTube e inclua apenas o `link` no arquivo `README.md` do seu repositório.

---

# Exercício 2 - Posição Angular de Figuras (4)

Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `q2.py` contendo uma classe denominada `EstimaPosicao`. Esta classe deve implementar um **nó** chamado `estima_posicao_node`, responsável por fazer com que o robô **simulado** execute uma volta completa em torno de seu eixo, reconheça a posição angular das figuras e armazene essas informações em um dicionário.

![pistas](figs/roda.jpg)

Utilize o comando abaixo para iniciar o simulador no mapa da prova:

```bash
ros2 launch my_gazebo roda.launch.py
```

## O nó criado deve
1. girar o robô em torno de seu eixo central até completar uma volta completa (360 graus).
2. Durante a rotação, o nó deve coletar a posição angular (em graus, 0 a 360) de cada figura detectada, com a melhor precisão possível.
3. O robô deve interromper a rotação, centralizar a figura no campo de visão, **PARAR** e coletar a posição angular, imprimir a figura e o valor angular em graus, e então continuar a rotação.
4. Armazenar as posições angulares em um dicionário, onde as chaves são os nomes das figuras (`"cat"`, `"dog"`, `"car"`, `"bicycle"`, `"person"`) e os valores são as posições angulares em graus.
5. Após completar a volta, imprimir o dicionário com as posições angulares de cada figura.

## Requisitos

1. Deve existir o arquivo chamado `q2.py`.
2. O programa deve ser executado sem erros.
3. A classe deve se chamar `EstimaPosicao`.
4. A implementação deve seguir a estrutura da classe `EstimaPosicao`, conforme exemplo no `base_control.py`.
5. A função `control` deve ser a única a publicar no tópico `/cmd_vel`.
6. A função `control` deve ser idêntica à do arquivo `base_control.py`. Todas as decisões de controle devem ocorrer dentro dos nós, sem alterações na função `control`.
7. Não utilizar loops infinitos ou `sleep` durante o controle do robô.

## Rúbrica
1. O programa deve respeitar as restrições definidas.
2. Nota: +1,0 - [1] & o robô consegue dar uma volta completo em torno de seu eixo e parar após completar a volta com precisão de 1 grau.
3. Nota: +1,5 - [2] & o robô consegue detectar e reconhecer a posição angular das figuras, armazenando corretamente no dicionário e imprimindo os valores. Mas não centraliza as figuras no campo de visão.
4. Nota: +1,5 - [3] & ao detectar uma figura, o robô interrompe a rotação, centraliza a figura no campo de visão, **claramente PARA**, reconhece a posição angular, imprime a figura e o valor angular em graus, e então continua a rotação, armazenando corretamente no dicionário e imprimindo os valores.

## Vídeo

Grave um vídeo mostrando que o robô é capaz de realizar o comportamento completo ou algum comportamento parcial, mostrando os valores e a comunicação no terminal. Publique os vídeos no YouTube e inclua apenas o `link` no arquivo `README.md` do seu repositório.