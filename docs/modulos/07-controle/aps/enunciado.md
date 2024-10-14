# Entregável 6 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para este entregável, você deve utilizar o robô real, mas você pode testar o código no simulador.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_6`.
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
Este exercício está avaliando a organização e qualidade dos vídeos dos exercícios 2, 3 e 4, do desafio e do arquivo `README.md`.

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

# Exercício 1 - Segue Linha P (4 pontos)
Resolva o exercicio na atividade [2 - Segue-Linha-P](https://insper.github.io/robotica-computacional/modulos/07-controle/atividades/2-seguelinha-proporcional/).

Repita o vídeo do **Exercício 2 - Segue Linha** da **APS 5**, agora com controle proporcional e adicione o link no arquivo `README.md` do seu repositório. Nesse caso, o robô deve seguir a linha de forma suave e precisa.

____________________________________________________________________

# Exercício 2 - Pega Creeper (5 pontos)
Baseando-se no código `base_control.py` do módulo 3, crie um arquivo chamado `pega.py` com uma classe denominada `PegaCreeper` e um nó denominado `pega_creeper_node` que olhe para 3 creepers, com duas cores possíveis e dois IDs possíveis e pegue o crreeper desejado.

* Recebe como atributo da classe `PegaCreeper` a cor e o ID do creeper que deve ser pego.

* Se comunica com o nó `creepers` que identifica os ID/Cor dos creepers e suas posições.

* Não se increver no tópico de visão, mas sim no tópico de creepers.

* Se aproxima do creeper reduzindo a velocidade linear conforme se aproxima. (ou seja, controle proporcional)

* Ao chegar perto do creeper, para e desce a garra.

* Continua a se aproximar do creeper até que esteja bem perto.

* Fecha a garra e levanta o creeper.

* Gire 180 graus e vá para o centro da arena.

* Desça o creeper e pare.

## Arena
A arena é um quadrado de 2m x 2m, com o robô iniciando no centro. Em um dos cantos da arena, há um conjunto de 3 creepers, **dois de mesma cor e id diferentes** e **dois de mesmo id e cores diferentes**. O robô deve ser capaz de pegar qualquer um dos creepers.

## Critérios de Avaliação:

1. Desenvolveu um nó que publica corremente os creepers.
2. O nó `pega_creeper_node` não se inscreve no tópico de visão.
3. O robô se aproxima de cada creeper e pegar o creeper desejado.
4. O robô gira 180 graus e retorna ao centro com o creeper em suas garras.
5. O robô desce o creeper e para.
4. **Vídeo:** Mostra o robô executando o comportamento desejado e pegando o creeper correto.
5. **Vídeo:** Link do vídeo do robô em ação no Youtube.