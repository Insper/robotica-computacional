# Robótica Computacional 2025.2 - AI

Instruções para a avaliação:

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
* Escreva a frase "yey" como a resposta da soma no arquivo `README.md` como teste de sua atenção.
* A responsabilidade por *`infraestrutura`*, *`configurações`* e *`setup`* em funcionamento pleno, é de cada estudante.
* **SÓ SERÃO ACEITOS REPOSITÓRIOS DE ALUNOS QUE ASSINARAM A LISTA DE PRESENÇA.**

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

- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote chamado `avaliacao_ai`.

    - **Dica:** Para utilizar os módulos desenvolvidos no capitulo 3, inclua o pacote `robcomp_util` e o pacote `robcomp_interfaces` como dependência do seu pacote, e então, importe como nos exemplos do capitulo 3.

---

# Exercício 0 - Organização & Qualidade
Este exercício avalia a organização e a qualidade dos vídeos dos exercícios e do arquivo `README.md`.

## Critérios de Avaliação:
* O pacote foi corretamente configurado.
* As dependências do pacote estão corretas.
* Os diretórios e arquivos estão organizados de forma adequada.
* Todos os scripts estão na pasta `avaliacao_ai` dentro do pacote `avaliacao_ai`.
* A configuração dos nós foi realizada corretamente.
* Os nós da ROS 2 foram executados utilizando o comando `ros2 run`.
* **Vídeo:** A ação do robô é claramente compreensível pelo vídeo.
* **README.md:** O link do vídeo foi adicionado corretamente no campo indicado.
* **README.md:** O arquivo `README.md` contém o nome completo e o e-mail do estudante.
---

# Exercício 1 - Show Bot (4) - (Desafio: +1,0)

<img src="figs/show.png" alt="Show Bot" width="400">

Baseando-se no código `base_control.py` do capítulo 3, crie um arquivo chamado `q1.py` contendo uma classe denominada `ShowBot`. Esta classe deve implementar um **nó** chamado `showbot_node`, responsável por executar **figuras geométricas** sob comando do **Orquestrator**.

O Orquestrator enviará comandos para o robô executar uma "forma" definida por:

* um **lado** `L ∈ [0.5, 1.0]` (metros), e
* um **ângulo** `θ ∈ {40, 60, 90, 120}` (graus), aplicado na rotação entre segmentos.

O robô deve:

1. avisar que está **pronto** ao iniciar,
2. **executar** a forma repetindo "anda `L` → gira `θ` graus" até **retornar próximo do ponto inicial**, e
3. ao retornar, **informar** o Orquestrator que terminou e **aguardar a métrica de deriva**.
4. após avisar que terminou, o Orquestrator enviará a métrica de **deriva** (distância entre o ponto inicial e o ponto final do robô, em metros).
5. **Após receber a métrica de deriva** do Orquestrator, o robô deve imprimir o valor com o comando de log da ros2 (`self.get_logger().info(ESCREVA AQUI)`), e então, voltar a avisar que está **pronto** para o próximo comando.
6. Deve finalizar depois de completar 2 formas geométricas.

Utilize o comando abaixo para iniciar o simulador no mapa da prova:

```bash
ros2 launch my_gazebo empty_world.launch.py
```

Verifique se o orquestrator está rodando com a presença do topico `/orquestrator` e com a presença de prints no terminal da simulação.

Caso o orquestrator não esteja rodando, feche TODOS os terminais e atualize os pacotes com o comando acima.

---

## O nó criado deve

* **Comunicação com o Orquestrator**

  * Publicar e assinar ao tópico `/showbot` com o tipo `robcomp_interfaces.msg.OrquestratorMSG`.
  * Ao iniciar, publicar **READY** no campo `status` com horário atual e **nome do aluno** nos campos apropriados.
  * Ao receber o `status` **IN_PROGRESS**, recolha os valores do lado e do ângulo dos campos apropriados e inicie a execução da forma.
  * Ao finalizar a forma, publicar **DONE** no campo `status` e aguardar a mensagem do Orquestrator.
  * Ao receber o `status` **FEEDBACK**, imprima o valor da deriva com o comando de log da ros2 (`self.get_logger().info(ESCREVA AQUI)`) e publicar **READY** para aguardar novo comando.

* **Execução da forma**

  * Robô deve executar a forma com os parâmetros recebidos:
    * Andar `L` metros em linha reta.
    * Girar `θ` graus (positivo: sentido anti-horário; negativo: horário).
    * Repetir até finalizar a forma.
   * **Limite de tempo:** cada forma deve terminar em **até 4 minutos**. Se passar disso, o Orquestrator vai spamar **ERROR** até o robô enviar novo comando **READY** enquanto o robô estiver na origem.



!!! dica
Se precisar resetar a posição do robô, enquanto a atenção estiver na simulação, aperte as teclas `Ctrl + R`.

---

## Requisitos

1. Deve existir o arquivo chamado `q1.py`.
2. O programa deve ser executado sem erros.
3. A classe deve ser chamada `ShowBot`.
4. A implementação deve seguir a estrutura da classe de exemplo em `base_control.py`.
5. A função `control` deve ser a **única** a publicar no tópico `/cmd_vel`.
6. A função `control` deve ser idêntica à do `base_control.py`. Todas as decisões de controle devem ocorrer dentro dos nós, sem alterações no `control`.
7. **Não** utilizar loops infinitos ou `sleep` durante o controle do robô.
8. Deve se publicar e assinar no tópico `/showbot` com mensagens do tipo `robcomp_interfaces.msg.OrquestratorMSG`.
9. Deve **executar a forma** com os parâmetros recebidos, em no maximo 4 min, contado de quando enviar READY para quando envia DONE.
10. Deve escrever a métrica de deriva com o comando de log da ros2 (`self.get_logger().info(ESCREVA AQUI)`).
11. Nunca spamar mensagens no tópico `/showbot`.

## Rúbrica

1. O programa deve respeitar as restrições definidas.
2. Nota: +1,0 - [1] & o robô mantém a conversa com o Orquestrador sem spamar.
3. Nota: +1,0 - [2] & O robô deve conseguir executar a forma, avisar quando terminar, receber a métrica de deriva, imprimir o valor e voltar a avisar que está pronto.
4. Nota: até +2,0 - [3] & robô finaliza após segunda forma & **Qualidade de deriva** - medida após a execução de duas formas geométricas quaisquer.

   * **Desafio +1,0** se - `drift_m ≤ 0.10 m`;
   * **2,0** se `drift_m ≤ 0.20 m`;
   * **1,0** se `0.20 < drift_m ≤ 0.40 m`;
   * **0,0** se `drift_m > 0.40 m` ou não reportado.

---

## Vídeo

Grave um vídeo mostrando:

* o **echo** do tópico `/showbot`;
* o **terminal da simulação**;
* o **terminal do robô**;
* e o robô executando pelo menos duas formas, recebendo a deriva, e **claramente** mostrando no terminal do robô.

Publique o vídeo no YouTube e inclua apenas o **link** no arquivo `README.md` do seu repositório.

Entregas parciais são aceitas, sem garantia de nota; o aluno deve explicar no `README.md` e na descrição do vídeo **até onde foi feito**.

---

# Exercício 2 - Manipular Logo (2,5)

Crie um arquivo chamado `q2.py` com uma classe chamada `LogoManipulator`, que possui um método `run`. Este método deve receber duas imagens (um logomarca e um fundo) e retornar uma nova imagem onde as partes da logomarca, em branco, são preenchidas pelo fundo fornecido. 

O arquivo só pode ser executado se for rodado diretamente, chamando a função `main`. A função main deve aplicar o método às imagens fornecidas e salvar o resultado em disco, além de exibi-lo em uma janela.

### Exemplo de entrada e saída:

![alt text](figs/image.png)

## Importante

O objetivo é que apenas as partes da logomarca (simbolo e/ou texto) seja substituída pelos pixels da imagem de fundo, mantendo o restante da imagem inalterado.

### Entrada
- Uma imagem de logomarca com fundo branco (exemplo: `logo1.png`, `logo2.png`, ... `logo6.png`). Sempre com o logo em branco e o restante da imagem em outra cor.
- Uma imagem de fundo (exemplo: `background.png`).

### Saída esperada

Uma nova imagem onde o texto do logotipo aparece com o preenchimento do fundo colorido.

### Restrições 

- A logomarca pode ter diferentes resoluções e proporções, mas o método deve funcionar em qualquer caso.
- O fundo deve ser redimensionado para caber na logomarca.
- O restante da imagem deve permanecer inalterado.

### Requisitos

1. Deve existir um arquivo chamado `q2.py`
2. O programa deve ser executado sem erros.
3. A classe deve se chamar `LogoManipulator`
4. A classe deve possuir um método `run` que recebe duas **imagens** (logo, fundo) e retorna a imagem manipulada.
5. Função `run` não carrega as imagens do disco.
6. O programa deve conter uma função `main` que aplique o método e salve os resultados em disco.
7. `O método deve funcionar dinamicamente para qualquer imagem com as mesmas características, além das fornecidas`.

## Rúbrica

1. O programa respeita as restrições definidas.
2. Nota: +0,5 - [1] & é capaz de carregar e exibir as imagens corretamente.
3. Nota: +0,5 - [2] & é capaz de identificar corretamente as regiões da logomarca (desenhe os contornos para validar).
4. Nota: +1,5 - [2] & é capaz de substituir as regiões por pixels do fundo em todas as imagens.

## Vídeo

Na função `main` adicione um loop para processar e mostrar todas as imagens fornecidas. Grave um vídeo mostrando o funcionamento do programa para todas as imagens fornecidas. Em cada imagem, deve mostrar a imagem original, o fundo aplicado e o resultado final. Publique o vídeo no YouTube e inclua apenas o link no arquivo `README.md` do seu repositório.

---

# Exercício 3 - Detectar Colisões (3,5)

Modifique o arquivo `q3.py` que tem uma classe chamada `CollisionCounter` e um método `run`. Este método deve processar um frame do vídeo, identificar a nave principal (vermelha) e os tiros (amarelos). A classe deve **contar o número total de tiros que atingiram a nave** ao longo do vídeo.

Como pode observar, o arquivo `q3.py` já tem a classe `CollisionCounter` e o método `run` implementados, mas não estão fazendo nada. O arquivo também já tem a função `main` que carrega o vídeo, passa frame a frame para o método `run` e exibe o vídeo enquanto ele é processado.

## Importante

No vídeo existem:

- Uma nave principal vermelha que deve ser monitorada.
- Tiros amarelos que podem ou não atingir a nave.

O desafio é contar o número total de tiros que atingiram a nave principal ao longo do vídeo.

## Saída esperada

Em cada frame processado, escreva na imagem o número de colisões detectadas até aquele momento. Ao final do vídeo, imprima no terminal o número total de colisões detectadas.

## Restrições

- A nave alvo é sempre vermelha, mas pode variar de formato em vídeos diferentes.
- Os tiros são sempre amarelos.
- O programa deve funcionar mesmo se a nave principal estiver em posições diferentes no vídeo.
- Considere que um tiro atingiu a nave se houver uma sobreposição significativa entre o tiro e a nave em um frame.

## Requisitos

- Deve existir um arquivo chamado `q3.py`
- O programa deve ser executado sem erros até o final do vídeo.
- A classe deve se chamar `CollisionCounter`.
- A classe deve possuir um método `run` que recebe um vídeo e retorna o número de colisões da nave principal com tiros.
- O programa deve conter uma função `main` que processa o vídeo, exibe os resultados e imprime o número de colisões.
- O método deve ser dinâmico e funcionar para qualquer vídeo com as mesmas características (nave vermelha, tiros amarelos).

## Rúbrica

2. Nota: +0,5 [1] - O método consegue identificar a nave e os tiros (desenhe os contornos para validar).
4. Nota: +1 [2] - O programa consegue identificar colisões e contar a quantidade de tiros que atingiram a nave, mas o valor final está incorreto. Ele também mostra o número de colisões na imagem.
5. Nota: +2 [3] - O método contabiliza precisamente o número total de tiros que acertaram a nave principal, para qualquer vídeo com as mesmas características, e exibe o número de colisões na imagem.

## Vídeo

Grave um vídeo mostrando o funcionamento do programa em **AMBOS** os vídeos fornecidos.
Em cada frame processado, deve mostrar: a nave principal segmentada, os tiros detectados e escrever na imagem o número de colisões detectadas até aquele momento.
Ao final, deve imprimir o número total de colisões detectadas e **claramente mostrar isso no terminal.**
Publique o vídeo no YouTube e inclua apenas o link no arquivo README.md do seu repositório.