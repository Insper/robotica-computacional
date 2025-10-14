# Entregável 7 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para este entregável, você deve utilizar o robô real, mas você pode testar o código no simulador.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_7`.
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

---

# Exercício 1 - Ação Segue Linha (8 pontos)
Primeiro resolva o exercicio na atividade [2 - Segue-Linha](https://insper.github.io/robotica-computacional/modulos/07-controle/atividades/2-seguelinha-proporcional/).

Em seguida, baseando-se no `base_control.py` (Módulo 3), crie um arquivo chamado `seguidor_de_linha.py` com um nó `seguidor_de_linha_node` que faça o robô **real** de uma volta completa no exterior do circuito de linha amarela do laboratório e **pare próximo do ponto de partida**.

## Ação Cliente (Principal)

O nó principal deve:

1. Ter dois estados: `segue_linha`, `done`.
2. Herdar de ??.
3. Instanciar a ação `segue_linha`.
4. Guardar ???.
5. Definir uma flag `self.iniciando = True`.
6. Iniciar a ação `segue_linha` no estado `segue_linha`.
7. Durante o estado `segue_linha` ele deve calcular ???? usando ???.
8. Quando ???? for maior que {valor} ele deve mudar a flag `self.iniciando = False`.
9. Quando a flag `self.iniciando` for `False` e ???? for menor que {valor} ele deve finalizar a ação e mudar para o estado `done`. 

# Desafio - Segue Linha Time Attack (+0 ou +1 ou +2 pontos)
Modifique o código do exercício 1 para que o robô complete o circuito de linha amarela do laboratório no menor tempo possível durante o desafio na aula. O tempo é contado de quando o aluno incia o robo até quando ele para "perto de onde iniciou". Cada grupo terá 3 tentativas para completar o circuito ou 15 minutos, podendo modificar o código entre as tentativas. O grupo que completar o circuito no menor tempo ganha 2 pontos extras, o segundo lugar e o terceiro lugar ganham 1 ponto.

---
# Exercício 2 - Ação de Pegar o Creeper (2 pontos + 0.5 extra no projeto (para o individuo, não o grupo do projeto))

Resolva o exercicio na atividade [3 - Controlando a Garra](https://insper.github.io/robotica-computacional/modulos/07-controle/atividades/3-garra/).

Grave um vídeo do robô real procurando e pegando um creeper selecionado pelo professor. **Pelo menos 9 creepers** devem estar espalhados na frente do robô com ambiguidades de cor e id com o creeper desejado. Finalize o vídeo com o robô segurando o creeper.

Para ganhar o ponto extra, tem 3 chances de pegar dois creepers em sequência ao vivo na aula. Depois de pegar o primeiro, o nó deve finalizar a ação, o aluno pode reposicionar o robô e remover o creeper da garra, e então iniciar a ação novamente para pegar o segundo creeper, **SEM MODIFICAR O CÓDIGO**.