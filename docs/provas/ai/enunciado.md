# Exercício 1 - Simon Diz (Desafio: +1,0)

Baseando-se no código `base_control.py` do capítulo 3, crie um arquivo chamado `q1.py` contendo uma classe denominada `JogadorSimon`. Esta classe deve implementar um nó chamado `jogador_simon_node`, responsável por fazer com que o robô simulado jogue o jogo **Simon Diz**.

No início da execução, o robô deve publicar que está pronto para o Simon, informando também o **horário atual** e o **nome do jogador** nos campos apropriados da mensagem. Imediatamente após isso, o robô deve começar a andar para frente.

Sempre que encontrar uma parede à frente, o robô deve parar e informar ao Simon quais lados estão disponíveis para movimento naquele ponto. As possibilidades são (literalmente):

- `direita e esquerda`
- `direita`
- `esquerda`
- `nenhuma`

O Simon irá verificar se os lados informados estão corretos e então poderá agir de uma das seguintes formas:

- publicar `Simon diz: vire para a direita`
- publicar `Simon diz: vire para a esquerda`
- publicar um comando **sem** a expressão `Simon diz:`

O robô **só pode executar um movimento se o comando recebido começar com `Simon diz:`**. Se o robô se mover quando Simon **não** tiver falado `Simon diz: ...`, Simon publicará no tópico que o jogador perdeu. Nesse caso, o robô deve parar e não deve mais se mover, sendo necessário reiniciar o nó. Caso o robô não se mova, Simon pode publicar o próximo comando depois de um pequeno intervalo de tempo, portanto o robô pode apenas esperar e não se mover até receber um comando válido.

Se o robô obedecer corretamente aos comandos válidos, ele deve continuar avançando no labirinto e repetindo o processo até chegar a uma posição em que nenhuma direção esteja disponível. Quando o robô publicar que nenhuma direção está disponível, Simon encerrará o jogo, publicando o **nome do aluno vencedor** e o **tempo total em segundos**.

## Modo deterministico

Se o robô publicar `modo_de_jogo = deterministico`, Simon deixará de escolher direções aleatoriamente e passará a sempre enviar o **próximo comando de uma sequência já definida**, de forma a levar o robô ao final do percurso em numero de passos predefinido.

Nesse modo, não há escolha aleatória nem retorno de alternativas, comandos inválidos são predefinidos assim como o delay entre os comandos. O objetivo é completar o percurso no menor tempo possível, obedecendo às regras do Simon.

## Simulador

Utilize o comando abaixo para iniciar o simulador no mapa da prova:

```bash
ros2 launch my_gazebo run_turtle.launch.py
```

## O nó criado deve:

- Publicar e assinar no tópico `\simon_says`, utilizando o tipo de mensagem especifico do topico.
- Ao iniciar, publicar literalmente a menssagem **simon, eu estou pronto**, com horário atual e nome do aluno nos campos apropriados.
- Opcionalmente preencher o campo `modo_de_jogo = deterministico` para disputar o **desafio** de menor tempo.
- Após avisar que está pronto, começar a andar para frente.
- Sempre que detectar uma parede à frente, informar ao Simon quais lados estão disponíveis.
- Executar movimentos **somente** quando receber comandos iniciados por `Simon diz:`.
- Se receber um comando sem `Simon diz:`, o robô deve permanecer parado e aguardar o próximo comando.
- Quando o jogo terminar com sucesso, o robô deve imprimir no terminal o nome do aluno vencedor e o tempo total recebido do Simon, então, o nó deve ser finalizado automaticamente.

## Requisitos

- Deve existir o arquivo chamado `q1.py`.
- O programa deve ser executado sem erros.
- A classe deve ser chamada `JogadorSimon`.
- A implementação deve seguir a estrutura da classe e de máquina de estados do exemplo `base_control.py`.
- A função `control` deve ser a única a publicar no tópico `/cmd_vel`.
- Todas as decisões de controle devem ocorrer dentro dos nós, sem alterações na função `control`.
- Não utilizar loops infinitos ou `sleep` durante o controle do robô.
- Não utilizar loops `while` dentro dos estados de controle, a máquina de estados e as ações devem ser acionadas apenas por callbacks de mensagens ou timers.
- Deve publicar e assinar corretamente no tópico de comunicação com o Simon.
- Deve publicar a mensagem inicial de pronto com nome e horário.
- Deve seguir as regras do jogo corretamente.
- Deve identificar corretamente os lados disponíveis utilizando os sensores ao encontrar uma parede.
- Deve ser capaz de executar o jogo completo para qualquer lado escolhido pelo Simon.
- Deve finalizar o nó automaticamente ao receber a mensagem de vitória, imprimindo o nome do vencedor e o tempo total.

## Rúbrica

O programa deve respeitar as restrições definidas.

* **Nota: +0,5** - [1] Sub, pub e comunicação com o Simon sem spamar.

* **Nota: +0,5** - [2] Nó consegue processar todas as possíveis mensagens do Simon.

* **Nota: +0,5** - [2] O robô só se move se ouvir `Simon diz:`, girando para o lado correto quando receber um comando válido, e permanecendo parado caso contrário.

* **Nota: +1,0** - [3] Consegue identificar de forma eficiente quais lados estão ocupados e quais estão disponíveis, informando corretamente ao Simon.

* **Nota: +1,0** - [4] Consegue executar o jogo completo corretamente para qualquer lado e comando válido enviado pelo Simon.

### Rúbrica Alternativa:

* **Nota Final = 2,0** - Completa ambos os trajetos fixos pré-definidos, sem considerar as mensagens do Simon.

## Vídeo

Grave um vídeo mostrando:

- o terminal do robô;
- o terminal da simulação;
- o `echo` do tópico de comunicação com o Simon;
- o robô executando o jogo no modo aleatório.

No vídeo do modo aleatório, o robô deve aparecer recebendo e executando **pelo menos 6 comandos do Simon**.

Publique o vídeo no YouTube e inclua **apenas o link** no arquivo `README.md` do seu repositório.

Entregas parciais são aceitas, sem garantia de nota. O aluno deve explicar no `README.md` e na descrição do vídeo até onde conseguiu implementar.

## Desafio (+1,0)

Para participar do desafio, o robô deve publicar `modo_de_jogo = deterministico`.

Nesse modo, Simon se comportará seguindo um script pré-definido, sem escolhas aleatórias, e o objetivo é completar o percurso no menor tempo possível, obedecendo às regras do Simon.

O aluno deve gravar um segundo vídeo mostrando:

- o terminal do robô;
- o terminal da simulação;
- o `echo` do tópico de comunicação com o Simon;
- o robô executando o percurso no modo `deterministico`;

Ao final da execução, o robô deve imprimir no terminal o tempo enviado pelo Simon e o nome do aluno vencedor e isto deve estar claro no vídeo.

O melhor tempo entre os alunos que completarem corretamente o desafio recebe **+1,0** ponto extra.

Os outros **2 alunos** entre os **3 melhores tempos** recebem **+0,5** pontos extra cada.
