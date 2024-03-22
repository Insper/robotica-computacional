# Robótica Computacional 2024.1 - Simulado AI

Observações de avaliações nesta disciplina:

* A prova tem duração de **2 horas**.
* Inicie a prova no Blackboard para a ferramenta de Proctoring iniciar. Só finalize o Blackboard quando enviar a prova via Github classroom.
* Durante esta prova vamos registrar somente a tela, não a câmera nem microfone.
* Coloque seu nome e email no README.md do seu repositório.
* Você pode consultar a internet ou qualquer material que usamos no curso, mas não pode se comunicar com pessoas ou colegas a respeito da prova. Também não pode usar ferramentas de **IA** como chatGPT ou Github Copilot durante a prova.
* Faça commits e pushes frequentes no seu repositório.
* Avisos importantes serão dados na sala da prova.
* Permite-se consultar qualquer material online ou próprio. Não se pode compartilhar informações com colegas durante a prova.
* Faça commits frequentes. O primeiro a enviar alguma ideia será considerado autor original.
* A responsabilidade por ter o *setup* funcionando é de cada estudante.
* Questões de esclarecimento geral podem ser perguntadas.
* É vedado colaborar ou pedir ajuda a colegas ou qualquer pessoa que conheça os assuntos avaliados nesta prova.

## Atualização dos Pacote (ROS 2)
Execute os comandos abaixo para atualizar os pacotes da `ros2` obrigatórios para a prova:

```bash
cd ~/colcon_ws/src/my_simulation
git stash
git pull
cd ~/colcon_ws/src/insperbot
git pull
cb
```

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `simulado_ai`.
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `my_package` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

___________________________

# Exercício 1 (5)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `q1.py` com uma classe chamada `Explorador` com um nó denominado `explorador_node`, que faça o robô **simulado** de uma volta completa dentro do quadrado, ao lado da parede externa, ao final da volta, sai do quadrado e para.

Rode o comando abaixo para iniciar o simulador no mapa da prova:

```bash
ros2 launch my_gazebo tres_paredes.launch.py
```


O nó deve: 

* Mover para o robô para frente até encontrar uma parede.
* Quando encontrar a parede, o robô deve girar e seguir a próxima parede.
* Repetir o processo até completar uma volta e encontrar a abertura na parede externa.
* Quando encontrar a abertura, o robô deve sair do quadrado e parar.

## Restrições

1. Ter um arquivo chamado `q1.py`
2. O programa rodar sem erros
3. Deve ter uma classe chamada `Explorador`
4. Seguir a chamada da classe `Explorador` como no exemplo `base_control.py`
5. A função `control` deve ser a única função chamada que publica no tópico `/cmd_vel`
6. A função `control` deve ser a mesma do `base_control.py`. As decisões de controle devem ser feitas dentro dos nós e não no `control`.

## Rúbrica
1. [+0.5] - O programa segue as restrições acima.
2. [+1.0] - [1] + Gira o robô e faz com que ele se mova para frente até encontrar uma parede.
3. [+2.0] - [2] + Quando encontrar uma parede, o robô gira e segue para próxima parede até completar uma volta.
4. [+1.5] - [3] + O robô encontra a abertura na parede externa e sai do quadrado.

___________________________
# Exercício 2 (5)
Crie um arquivo chamado `q2.py` com uma classe chamada `DominoDetector` que tem um método chamado `run` que recebe um vídeo e escreve na tela o valor das peças de dominó que aparecem em cada frame do vídeo.

Baixe o vídeo [neste endereço](https://github.com/Insper/robot20/raw/master/media/dominoes.mp4)

Exemplo de peça de dominó:

![Domino](img/domino.jpg)

## Objetivo

Um programa que escreve *na tela* o valor da peça de dominó que aparece a cada frame do vídeo. 

Por exemplo para a peça acima, deve escrever **5 por 3**

**Grave um video** da saída, publique no Youtube e escreva o link no arquivo `README.md` do seu repositório.

## Restrições

1. Ter um arquivo chamado `q2.py`
2. O programa rodar sem erros
3. Deve ter uma classe chamada `DominoDetector`
4. A classe deve ter um método chamado `run` que recebe uma imagem e escreve na tela o valor da peça de dominó que aparece na imagem.
5. O programa deve ter uma função `main` que lê o vídeo e chama o método `run` para cada frame do vídeo.
6. Depois de chamar o método `run` para cada frame, o programa deve mostrar o frame com os valores das peças de dominó.
6. A função `main` só deve ser chamada se o arquivo for executado diretamente, não se for importado como módulo.

## Rúbrica

1. [+0.5] - O programa segue as restrições acima.
2. [+0.5] - [1] + O programa consegue distinguir as peças de dominó do fundo.
3. [+1.5] - [2] + O programa consegue identificar o valor das peças corretamente em metade dos frames e mostra o vídeo com os valores das peças.
4. [+1.5] - [3] + O programa consegue identificar o valor das peças corretamente em todos os frames e mostra o vídeo com os valores das peças.
5. [+1.0] - [4] + O programa identifica o valor das peças praticamente sem nenhuma falha (flutuações de 1 ou 2 valores são aceitáveis).


