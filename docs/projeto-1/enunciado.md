# Projeto 1 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `projeto_1`.
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `robcomp_util` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

____________________________________________________________________
# **IMPORTANTE**
Atualize o pacote do `robcomp_interfaces` que existe em seu SSD com os comandos abaixo:
```bash
cd ~/colcon_ws/src/my_simulation/
git stash
git pull
cb
```
____________________________________________________________________

# Exercício 0 - Organização & Qualidade (1 pontos)
Este exercício está avaliando a organização e qualidade dos vídeos dos exercícios da APS e do arquivo `README.md`.

## Critérios de Avaliação:
* O pacote foi corretamente configurado.
* As dependências do pacote estão corretas.
* A configuração dos nós foi realizada corretamente.
* Os diretórios e arquivos estão organizados de forma adequada.
* **Vídeo:** Utilize o comando `ros2 run` para executar o nó, mostre o comando sendo executado no terminal.
* **Vídeo:** O vídeo foi gravado na **horizontal**.
* **Vídeo:** O vídeo foi gravado em um ambiente bem iluminado.
* **Vídeo:** O audio está claro e sem ruídos, se desejar, remova o audio e adicione uma música de fundo.
* **Vídeo:** Na descrição do vídeo no Youtube, está descrito o que o robô está fazendo.
* **Vídeo:** Pelo vídeo, é possível entender o que o robô está fazendo.
* **README.md:** O link do vídeo está correto e foi adicionado no campo específico.
* **README.md:** O arquivo README.md tem o nome completo e o email de todos os integrantes do grupo.

____________________________________________________________________
# Exercício 0 - Projeto 1 - Mapeamento (1 pontos)

Nesta questão usaremos o cenário `roslaunch my_simulation corrida_de_obstaculos.launch` e vamos trabalha no arquivo `q3.py`:


![](obstaculos.png)


Seu robô deverá ter os seguintes comportamentos:


1. Sortear um tempo aleatório entre 0.0 e 6.0 segundos.
2. Girar o robô em uma velocidade constante de 0.4 rads/s.
3. Ao longo da execução do giro deve exibir o log "girei $$ segundos de ## segundos" onde: `#` representa o tempo decorrido em segundos com 1 casa decimal e `$` representa o valor sorteado em segundo com 1 casa decimal.
4. O robô deve parar e exibir o log "giro aleatório de ## segundos concluído!"
5. O robô deve girar novamente até encontrar a pista mais próxima, independente da cor.
6. Seguir a pista detectada até encontrar um portal, que está identificado no mapa pela cor amarela.
7. O robô deve identificar quando passa por baixo de um portal e exibir o log "passei pelo portal".
8. Com o auxilio da garra o robô deve derrubar o portal.
9. O robô deve seguir a pista e parar quando completar uma volta.
10. Após completar a volta completa, um gráfico deve ser exibido com as coordenadas de posição (x,y) de todos os portais derrubados com um círculo da cor correspondente a pista que o robô estava seguindo quando derrubou o portal, por exemplo: portal derrubado na pista magenta, plota circulo da cor magenta.
9. Salvar o gráfico em um arquivo `png` com o nome `obstaculos_derrubados.png` no diretório `scripts`.
10. O robô deve imprimir `FIM` no terminal e **encerrar** a operação.


**AVISO:** Os portais podem ser deslocados para qualquer posição dentro da pista - inclusive em pistas de cores diferentes. A distância mínima entre os portais é de 0,5m.


Os seguintes critérios de correção serão usados:


- **R0)(até 0,5)** Robô completa o giro aleatório e encontra a pista mais próxima.
- **R1)(até +0,5)**  Realiza **R0** e completa uma volta na pista parando no ponto de partida.
- **R2)(até +0,5)**  Realiza **R1** e identifica quando passa por baixo de um portal, identifica corretamente todos os portais mas não derruba.
- **R3)(até +0,5)** Realiza **R2** e derruba ao menos um portal ao passar por baixo deles.
- **R4)(até +1,0)** Realiza **R3** e derruba corretamente todos os portais ao passar por baixo deles.
- **R5)(até +0,5)** Realiza **R2** exibe e salva o gráfico com todas as posições e cores dos portais corretamente.
- **R6)(até +0,5)** Realiza **R4** e **R5** e exibe o log `FIM` e encerra a operação finalizando o nó.