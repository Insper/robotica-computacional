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

## Modo fast

Se o robô publicar `modo_de_jogo = fast`, Simon deixará de escolher direções aleatoriamente e passará a sempre enviar o **próximo comando da sequência que leva ao final**, de forma a levar o robô ao final do percurso no menor numero de passos possíveis.

Nesse modo, não há escolha aleatória nem retorno de alternativas. O robô deve apenas seguir corretamente os comandos enviados por Simon até o final.

## Simulador

Utilize o comando abaixo para iniciar o simulador no mapa da prova:

```bash
ros2 launch my_gazebo run_turtle.launch.py
```

## Comunicação com o Simon

O nó criado deve:

- Publicar e assinar o tópico definido para comunicação com o Simon, utilizando o tipo de mensagem especificado na prova.
- Ao iniciar, publicar uma mensagem equivalente a **estou pronto**, com horário atual e nome do aluno nos campos apropriados.
- Opcionalmente publicar o campo `modo_de_jogo = fast` para disputar o desafio de menor tempo.
- Após o início do jogo, andar imediatamente para frente.
- Sempre que detectar uma parede à frente, informar ao Simon quais lados estão disponíveis.
- Executar giros **somente** quando receber comandos iniciados por `Simon diz:`.
- Se receber um comando sem `Simon diz:`, não deve executar qualquer movimento associado a esse comando.
- Quando receber do Simon a mensagem de derrota, o robô deve parar e nunca mais se mover.
- Quando o jogo terminar com sucesso, deve parar e registrar no terminal o nome do vencedor e o tempo enviado pelo Simon.

## Execução do jogo

O robô deve:

- Começar sempre da posição inicial padrão do mapa.
- Andar para frente até encontrar uma parede.
- Identificar de forma eficiente quais lados estão livres naquele ponto do labirinto.
- Informar corretamente ao Simon as possibilidades de movimento.
- Aguardar o comando do Simon.
- Girar apenas se o comando recebido começar com `Simon diz:`.
- Após girar, voltar a andar para frente até encontrar a próxima parede.
- Repetir esse processo até chegar ao ponto final, em que nenhuma direção esteja disponível.
- Ao final, permanecer parado e não se mover mais.

## Requisitos

- Deve existir o arquivo chamado `q1.py`.
- O programa deve ser executado sem erros.
- A classe deve ser chamada `JogadorSimon`.
- A implementação deve seguir a estrutura da classe de exemplo em `base_control.py`.
- A função `control` deve ser a única a publicar no tópico `/cmd_vel`.
- A função `control` deve ser idêntica à do `base_control.py`. Todas as decisões de controle devem ocorrer dentro dos nós, sem alterações na função `control`.
- Não utilizar loops infinitos ou `sleep` durante o controle do robô.
- Deve publicar e assinar corretamente o tópico de comunicação com o Simon.
- Deve publicar a mensagem inicial de pronto com nome e horário.
- Deve se mover **somente** quando apropriado de acordo com as regras do Simon.
- Deve identificar corretamente os lados disponíveis ao encontrar uma parede.
- Deve ser capaz de executar o jogo completo para qualquer lado escolhido pelo Simon.
- Deve parar definitivamente em caso de derrota.
- Deve parar definitivamente ao final do jogo.

## Rúbrica

O programa deve respeitar as restrições definidas.

**Nota: +1,0** - [1] Sub, pub e comunicação com o Simon sem spamar.

**Nota: +1,0** - [2] O robô só se move se ouvir `Simon diz:`.

**Nota: +2,0** - [3] Consegue identificar de forma eficiente quais lados estão ocupados e quais estão disponíveis, informando corretamente ao Simon.

**Nota: +2,0** - [4] Consegue executar o jogo completo corretamente para qualquer lado e comando válido enviado pelo Simon.

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

Para participar do desafio, o robô deve publicar `modo_de_jogo = fast`.

Nesse modo, Simon sempre enviará o próximo comando correto da sequência para chegar ao final o mais rápido possível.

O aluno deve gravar um segundo vídeo mostrando:

- o robô executando o percurso no modo `fast`;
- o terminal da simulação visível;
- o tempo final sendo publicado pelo Simon;
- o terminal do robô mostrando claramente o tempo recebido.

Ao final da execução, o robô deve imprimir no terminal o tempo enviado pelo Simon.

O melhor tempo entre os alunos que completarem corretamente o desafio recebe **+1,0** ponto extra.

Os outros **2 alunos** entre os **3 melhores tempos** recebem **+0,5** ponto extra cada.

Para que o desafio seja considerado válido, deve estar claro no vídeo que:

- o tempo foi enviado pelo Simon;
- o robô chegou corretamente ao final;
- o terminal do simulador está visível.

## Observações finais

- O foco da prova é a lógica de comunicação, tomada de decisão e execução correta dos comandos.
- Soluções que funcionem apenas para um caminho fixo ou que ignorem as mensagens do Simon não atendem ao objetivo do exercício.
- Em caso de dúvida entre robustez e velocidade, priorize primeiro o funcionamento correto do jogo.

