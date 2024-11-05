# Entregável 3 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para este entregável, você deve utilizar o robô real, mas você pode testar o código no simulador.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_8`.
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

# Exercício 1 - Explorar Labirinto (3 pontos)
Utilizando o pacote `Cartographer` e o pacote `Navigation`, explore o labirinto do nosso laboratório, a partir do ponto interno do labirinto, enviando comandos de navegação para o robô. Por fim, salve o mapa do labirinto e adicione o arquivo `map.pgm` e `map.yaml` no seu repositório.

!!! dica
    O `Navigation` atualmente tem um bug que impede o robô de navegar contornando obstáculos, então, envie vários pontos para o robô, para que ele possa navegar pelo labirinto.

## Vídeo

Grave um vídeo do robô explorando o labirinto do laboratório e adicione o link no arquivo `README.md` do seu repositório. No vídeo, mostre a tela do computador com o Rviz e então, mostre o robô explorando o labirinto.

____________________________________________________________________

# Exercício 2 - Labirinto-GoTo (6 pontos)

Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `teseu.py`, como uma classe `Teseu` e com um nó denominado `teseu_node`, a partir de uma sequência de pontos, obtidas do mapa do labirinto, faça o robô **real** escape do labirinto. O nó deve:

* Utilize o script `visualizar_mapa.py` para obter a sequência de pontos do labirinto (Cap. 9)

* Receber uma sequência de pontos do labirinto.

* Utilizar, de alguma forma, a ação `GoTo` para fazer o robô se movimentar entre os pontos.

* Ao chegar no último ponto, o robô deve entrar em um estado `stop` e parar.

## Critérios de Avaliação:

1. Execute o pacote `Navigation` para navegar no labirinto.
2. Execute o nó `teseu_node` para fazer o robô escapar do labirinto.
3. Não utiliza nenhuma função de `sleep` para controlar o tempo de execução.
4. **Vídeo:** Mostra o robô executando o comportamento e escapando do labirinto.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.