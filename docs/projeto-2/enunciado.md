# Projeto - Missão na 404

O objetivo do projeto é programar nosso robô para cumprir um conjunto de missões na sala 404. Utilizaremos o cenário apresentado a seguir.

![](pista.png)

Os grupos, que podem ser compostos por `até quatro integrantes`, deverão trabalhar em conjuto no repositório do GitHub Classroom disponibilizado. 

[Link para repositório do projeto GitHub Classroom](https://classroom.github.com/a/4fM2pHCF){ .ah-button }

# **`Data de Entrega: 26/05 às 23:59h`**

________________________________________________________

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de realizar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todos os conceitos os grupos **`devem gravar um vídeo do seu robô executando a tarefa`**. O vídeo deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve preencher com apenas o link de cada video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

**Aviso 5:** Para o projeto, as entregas são feitas utilizando o robô real. Entregas no simulador serão aceitas, conquistando no máximo o conceito **D**.

**Aviso 6:** São aceitos videos com entregas parciais que comprovam que seu robô é capaz de realizar parte das tarefas. Inclua o link dos vídeos no README com um comentário. 

- É possível realizar a entrega parcial de conceitos avançados, por exemplo, entregar a parte de um conceito **B** antes de entregar a parte de um conceito **C**.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote chamado `projeto_robcomp`.
    - **Dica:** Para utilizar os modulos desenvolvidos no módulo 3, inclua o pacote `robcomp_util` como dependência do seu pacote, e então, importe como nos exemplos do módulo 3.

________________________________________________________

## Atualização dos Repositórios

Em um novo terminal, execute os comandos a seguir linha por linha.

```bash
cd ~/colcon_ws/src/my_simulation
git stash
git pull
cb
```

## Mapa Simulado

Para inicializar o mapa do desafio, em um terminal digite:

```bash
ros2 launch my_gazebo pista-24B.launch.py
```
________________________________________________________


## Descrição das Missões

O projeto é composto por 4 missões de complexidade crescente, envolvendo tanto o design de software quanto a utilização dos sensores e comportamentos do robô. **É preciso concluir todas as missões anteriores para obter a nota da missão subsequente**.

Cada missão deverá ser registrada em um ou mais vídeos, com o link adicionado no arquivo `README` e o codigo deve ser versionado utilizando o **Releases** do GitHub, com a tag do conceito atingido.

As missões envolvem os seguintes elementos:

* **Pista**: O robô deve permanecer dentro da pista, retornando a ela o mais rápido possível caso saia.

* **Creepers**: Bonecos semelhantes aos do Minecraft, que devem ser transportados até a área de entrega. 
    
    - Quatro creepers estão posicionados no **creeper_place** em com dois cores diferentes e dois ids diferentes.

* **"Drop Area"**: Caixas com imagens detectáveis pela MobileNet, local onde os creepers devem ser depositados. Elas são colocadas em locais fixos da pista.

* **Placas**: Arucos de `ID` `100`, `150` e `250`. **O grupo pode coloca-las em qualquer lugar**, fora da linha branca, para auxiliar na navegação.

!!!tip
    No simulador, você pode alterar a posição das placas no arquivo `pista24B.world`, em um terminal digite:

    ```cmd
    code /home/borg/colcon_ws/src/my_simulation/my_gazebo/worlds/pista24B.world
    ```

### Para completar uma missão, o grupo deve:

1. Gravar o robô realizando a missão na pista real.

2. Incluir o link do vídeo no README do seu repositório e criar um Release com a tag referente ao último conceito alcançado.

3. Cada missão deve ser concluída em menos de 15 minutos.

4. **Deixar** o creeper em uma "Drop Area" inclui:

    4.1. Parar próximo à "Drop Area".

    4.2. Posicionar-se de frente da "Drop Area".

    4.3. Aproximar-se até ficar a 0.5m de distância.

    4.4. Abaixar a garra e soltar o creeper em pé.

    4.5. Retornar à pista sem derrubar o creeper.

As missões podem ser validadas ao longo do projeto. Solicite a validação da parte de requisitos de software antes de gravar.

Sua classe deve receber como entrada, cor do creeper, ID do creeper e "Drop Area".

<!--- 
!!! Atenção
    Se uma missão for completada apenas no simulador, será concedido 25% da diferença dos conceitos de nota. **A missão C é obrigatória no robô real**.
--->
________________________________________________________

### Missão **C**

!!! warning
    Realizar a Missão no robô real

Essa missão é a mais simples do projeto e envolve aplicar diretamente os conceitos trabalhados na disciplina, para pegar um creeper qualquer e retornar para a posição inicial. Os seguintes passos devem ser realizados:

1. O robô sai da posição inicial e visita o local onde estão os creepers.

2. O robô deve pegar **qualquer** creeper.

3. O robô deve retornar para a posição inicial.

**Requisitos de projeto de software**:

* **Uso de Classes**: O código deve ser estruturado de forma orientada a objetos, utilizando classes para organizar as funcionalidades

* **Máquina de Estados**: Implemente uma máquina de estados para gerenciar as diferentes etapas da missão;

* **Controle Proporcional**: Utilize técnicas de controle proporcional para manter o robô na trajetória desejada, especialmente ao seguir a linha.

**Nota final desta missão:** 5,0

________________________________________________________


### Missão **B**

!!! warning
    Realizar a Missão no robô real

Essa missão expande a missão anterior, mas agora o robô deve pegar um creeper específico (ID e cor) e então retornar para a posição inicial. 
Os seguintes passos devem ser realizados:

1. A classe recebe a cor e o ID do creeper **desejado**.

2. O robô sai da posição inicial e visita o local onde estão os creepers.

3. O robô deve pegar o creeper **desejado**.

4. O robô deve retornar para a posição inicial.

**Requisitos de projeto de software**:

- Mesmo desafio da missão **C**

**Nota final desta missão:** 7,0


________________________________________________________

### Missão **A**

!!! warning
    Realizar a Missão no robô real

Essa missão expande a missão anterior, mas agora o robô deve pegar um creeper específico (ID e cor), entrega-lo em uma "Drop Area" **qualquer** e então retornar para a posição inicial.
Os seguintes passos devem ser realizados:

1. A classe recebe a cor e o ID do creeper **desejado**.

2. O robô sai da posição inicial e visita o local onde estão os creepers.

3. O robô deve pegar o creeper **desejado**.

4. O robô deve se mover para uma "Drop Area" **qualquer**.

5. O robô deve deixar o creeper na "Drop Area".

6. O robô deve retornar para a posição inicial.

**Requisitos de projeto de software**:

- Mesmo do **C**

* **Subscriber para Aruco**: Crie um nó específico que identifica o creeper pela cor e ID desejados - tanto pode publicar todos os creepers encontrados quanto apenas o creeper desejado - é uma boa prática este nó se inscrever em um tópico *flag* que indica se o nó deve ou não processar.

* **Subscriber para MobileNet**: Crie um nó para a leitura da MobileNet. Ele deve publicar um tópico com a posição (na imagem) e classe da caixa detectada - é uma boa prática este nó se inscrever em um tópico *flag* que indica se o nó deve ou não processar.

**Nota final desta missão:** 9,0


________________________________________________________

### Missão **A+**

!!! warning
    Realizar a Missão no robô real

Essa missão expande a missão anterior, mas agora o robô deve pegar um creeper específico (ID e cor), procurar e entrega-lo para uma "Drop Area" **desejada** e então retornar para a posição inicial.
Os seguintes passos devem ser realizados:

1. A classe recebe a cor e o ID do creeper **desejado** e a "Drop Area" **desejada**.

2. O robô sai da posição inicial e visita o local onde estão os creepers.

3. O robô deve pegar o creeper **desejado**.

4. O robô deve se mover para a "Drop Area" **desejada**.

5. O robô deve deixar o creeper na "Drop Area".

6. O robô deve retornar para a posição inicial.

**Requisitos de projeto de software**:

- Mesmo do **A**

**Nota final desta missão:** 10,0
