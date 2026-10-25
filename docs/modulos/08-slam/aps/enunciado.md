# Entregável 8 de Robótica Computacional

**Data de entrega:** {{ data_APS8 }}

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Siga as instruções de [Sobre as APS](../../../aps.md): envie no Blackboard o link do repositório e o SHA completo do commit que deve ser avaliado, até o prazo de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para este entregável, você deve utilizar o robô real, mas você pode testar o código no simulador.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** O repositório é informado no Blackboard (veja [Sobre as APS](https://insper.github.io/robotica-computacional/aps/)). Clone-o **dentro da pasta** `colcon_ws/src/` no seu SSD.
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

# Exercício 0 - Organização & Qualidade (0 pontos)
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

# Exercício 1 - Robô Quadrado Melhorado (7 pontos)
Primeiro resolva o exercicio na atividade [1 - GoTo](https://insper.github.io/robotica-computacional/modulos/08-slam/atividades/1-goto/).

Em seguida, baseando-se no `base_control.py` (Módulo 3), crie um arquivo chamado `quadrado.py` com um nó `quadrado_node` que faça o robô **real** percorrer os quatro vértices do quadrado de referência no chão do laboratório e retornar ao ponto inicial. O nó deve:

## Ação Cliente (Principal)

O nó principal deve:

1. Ter dois estados: `goto`, `done`.
2. Instanciar a ação `GoTo`.
3. Definir uma flag `self.iniciando = True`.
4. O estado `goto` deve:
    * Definir os vértices no mesmo referencial da odometria usada pelo `GoTo`.
    * Usar a flag para iniciar cada destino uma única vez e aguardar a ação chegar a `done` antes de enviar o próximo ponto.
    * Percorrer os quatro lados, incluindo o trecho de volta ao ponto inicial.
    * Ao concluir o percurso, mudar para `done` e garantir que o robô pare.

Grave um vídeo do robô real desenhando o quadrado de referência no chão do laboratório. Salve o vídeo no Youtube e adicione o link no arquivo `README.md` do seu repositório.

---

# Exercício 2 - SLAM no Labirinto (3 pontos)

Utilizando o `Cartographer`, mapeie o labirinto do laboratório com o **robô real**, percorrendo os corredores para representar as paredes e as áreas livres. Nesta APS, a exploração pode ser conduzida por teleoperação ou por destinos enviados pela interface do Nav2; não é necessário implementar exploração autônoma. Por fim, salve o mapa e adicione os arquivos `map.pgm` e `map.yaml` ao repositório.

!!! dica
    Se usar o Nav2, envie destinos em regiões já mapeadas e livres. Para esta atividade, você também pode fazer todo o mapeamento usando o teclado, sem iniciar o Nav2.

## Vídeo

Grave um vídeo do robô explorando o labirinto do laboratório e adicione o link no arquivo `README.md` do seu repositório. No vídeo, mostre o robô real explorando o labirinto, o mapa sendo construído no RViz e o resultado final do mapeamento.