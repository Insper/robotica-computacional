# Missão na 404

No projeto da disciplina iremos programar o nosso TurtleBot para realizar missões robóticas na sala 404. O cenário que usaremos é o abaixo.

![](pista.jpeg)

O projeto deverá ser feito em grupos de 4 pessoas usando o repositório do GH Classroom abaixo. 


[Link para tarefa do GH Classroom](https://classroom.github.com/a/4tk_fGP5){ .ah-button }

**Data de Entrega: 31/05 23:59**

## Começando o projeto

O repositório do Github Classroom estará vazio. Para iniciar o projeto um dos membros do grupo deverá seguir o [guia de início de projetos ROS](criar-projeto.md). 

Quando isso for finalizado, o restante do grupo deverá clonar o projeto ROS na pasta `~/catkin_ws/src` usando o seguinte comando:

```bash
git clone SEU_REPO_AQUI ~/catkin_ws/src/projeto-robcomp
```

Também é necessário atualizar os repositórios locais para baixar a pista do projeto. Faça isso usando os comandos abaixo. **Todos os membros do grupo precisam fazer este passo**.

```bash
cd ~/catkin_ws/src/mybot_description
git pull
cd ~/catkin_ws/src/my_simulation
git pull
cd ~/catkin_ws
catkin_make
```

Para lançar a simulação da pista do projeto, execute o comando abaixo.

```bash
roslaunch my_simulation pista23-B.launch
```

Cada missão deverá ser colocada em um arquivo `.py` separado com o nome da missão que ele realiza. 

## Missões

O projeto consistirá em diversas missões diferentes no cenário acima. As missões tem complexidade crescente e requisitos tanto de projeto de software quanto de uso dos sensores e comportamentos do robô. É necessário realizar todas as missões anteriores para conseguir a nota de uma missão mais avançada. As missões envolvem os seguintes elementos:

- **Pista**: seu robô deve andar sempre dentro da pista. Se ele sair da pista, deve voltar o mais rápido possível 
- **Creepers**: bonequinhos que parecem os do minecraft e que devem ser levados até
- **Drop Area**: caixas com imagens detectáveis pela mobilenet. Creepers devem ser depositados perto dessas caixas
- **Slalom**: sequência de 3 caixas coloridas em forma de zigue zague. 

Para completar uma missão você deve:

- [ ] capturar a tela do robô realizando a missão no simulador 
- [ ] filmar o robô realizando a missão na pista da sala
    - para filmar é preciso pedir a preparação da pista para a equipe de técnicos. Isso garante que todos terão condições parecidas nas missões.
- [ ] Coloque o link do video no README de seu repositorio e crie um **Releases** com a tag do ultimo conceito completado.
- [ ] cada missão demora menos de 15 minutos para ser completada
- [ ] Entende-se por **DEIXAR** o creeper em uma drop area como realizar as seguintes acoes:
    - Parar perto da drop area
    - Olhar diretamente para a drop area
    - Se aproximar ate ficar a 0.5m da drop area
    - Descer a garra e soltar o creeper em **pe**
    - Retornar a pista sem derrubar o creeper

Missões podem ser completadas durante o andamento do projeto. É só pedir a validação da parte dos requisitos de software antes de gravar. 

!!! Atenção
    Se uma missão só for completada no simulador so recebera 25% da diferenca dos conceitos nota. **A missão D é obrigatório rodar no robô real**. 

### Missão **D**

Essa missão é a mais simples do projeto e envolve aplicar diretamente os conceitos trabalhados na APS04. Os seguintes passos devem ser realizados:

1. Seu codigo inicia e pede ao usario a cor do creeper **desejado**.
2. O robô sai da posição inicial e segue reto na bifurcação. 
3. Ele segue pela área da direita até juntar novamente os caminhos
4. Vai em direção aos creepers e derruba o creeper **desejado**
5. Retorna para a posição inicial usando a mesma pista

**Requisitos de projeto de software**:

- [ ] o código deve usar classes
- [ ] o código deve usar máquina de estados
- [ ] é usado controle proporcional para o robô seguir a linha

**Nota final desta missão:** 3,0

!!! Atenção
    É obrigatório rodar essa missão no robô real

### Missão **C**

A missão **C** envolve agora usar a garra para pegar o creeper (e possivelmente o LiDAR para medir a distância até ele). 

1. Seu codigo inicia e pede ao usario a cor do creeper **desejado**.
2. O robô sai da posição inicial e segue reto na bifurcação. 
3. Ele segue pela área da direita até juntar novamente os caminhos
4. Ele pega o creeper **desejado** e retorna pela pista do centro. 
5. Ele **deixa** o creeper na bicicleta
6. Para na posição inicial


**Requisitos de projeto de software**:

- [ ] o código deve usar classes
- [ ] o código deve usar máquina de estados
- [ ] é usado controle proporcional para o robô seguir a linha
- [ ] existe um subscriber específico para a leitura do aruco
- [ ] existe um subscriber específico para a leitura da mobileNet (mesmo que não use)

**Nota final desta missão:** 5,0

### Missão **C+**

A missão **C+** adiciona um novo desafio de controle: fazer o **Slalom**

1. Seu codigo inicia e pede ao usario a cor do creeper **desejado**.
1. O robô sai da posição inicial, vira à esquerda na bifurcacao
3. Segue ate a pista da esquerda
2. Faz o **Slalom** e prossegue até a área dos creepers
3. Pega o creeper **verde** e deposita na drop area da pista do centro
4. Volta para posição inicial

**Requisitos de projeto de software**:

- Mesmo do **C**

**Nota final desta missão:** 6,5

### Missão **B**

Agora vamos integrar tudo, com objetivos que dependem da *MobileNet*

1. O programa recebe dois argumentos na linha de comando: cor do creeper e drop area
2. Vai até a área do creepers usando qualquer caminho que quiser e pega o creeper
3. Encontra a drop area passada como argumento. **Neste item ela sempre estará no centro ou na direita.**
4. Deposita o creeper na drop area requisitada
5. Retorna ao ponto inicial

!!! important
    Para demonstrar o funcionamento do seu robô será necessário fazer vários testes:

    - pegar creeper de cada cor uma vez
    - depositar creeper em cada uma das 2 drop areas

    Planeje bem o desenvolvimento do seu projeto para que isso seja feito na menor quantidade possível de filmagens

**Requisitos de projeto de software**:

- [ ] código deve usar classes
- [ ] código deve usar máquina de estados
- [ ] é usado controle proporcional para o robô seguir a linha
- [ ] código não leva em conta conhecimento sobre a posição das drop areas, mas sim a detecção da imagem das caixas


**Nota final desta missão:** 8,0

### Missão **B+**

Esse item é exatamente igual ao anterior, mas deve-se demonstrar a missão acima com caixas em posições diferentes das que estão no cenário simulado. Ou seja, faça dois vídeos em que

1. todas as drop areas estão em locais diferentes (e longe dos locais originais)
2. trocou as drop areas de lado (a do centro foi para a direita e a da direita para o centro) 

**Requisitos de projeto de software**:

- [ ] código deve usar classes
- [ ] código deve usar máquina de estados
- [ ] é usado controle proporcional para o robô seguir a linha
- [ ] código não leva em conta conhecimento sobre a posição das drop areas, mas sim a detecção da imagem das caixas
- [ ] parte de visão do código está feita em um nó separado do ROS, que publica um ou mais tópicos com as informações detectadas


**Nota final desta missão:** 9,0

### Missão **A+**

A missão final é a mesma da **Missão B**, mas com um adendo: qualquer uma das 4 drop areas pode ser passada na linha de comando e elas podem estar em qualquer lugar da pista. Isso significa que iremos juntar tudo o que foi feito em um único programa:

1. **Garra** e **LiDAR** para pegar os creepers
2. **mobile net** para detectar as drop areas corretas
3. **aruco** para encontrar encruzilhadas e creepers
4. **controle proporcional** para andar na pista sem sair
5. **planejamento e máquina de estados** para criar comportamento complexo a partir de estados "simples"

!!! important
    Aqui será necessário criar várias filmagens para validar. Isso dá um certo trabalho fazer no robô real, então algumas delas podem ser feitas no simulador. Faça várias combinações possíveis de

    - cores do creeper a ser pego
    - drop area para depositar (colocar no lado do slalom e no outro lado)

**Nota final desta missão:** 10,0

-----------------------------------------


!!! people "Contribuições"
    - ![Diego](equipe/diego.jpg) **Diego Pavan Soler** *Professor*
    - ![Arnaldo](equipe/arnaldo.jpeg) **Arnaldo Alves Viana Junior** *Prof. Auxiliar*
    - ![Rogério](/robotica-computacional/equipe/rogerio.jpeg) **Rogério Cuenca** *Técnico de lab*
    - ![Igor](/robotica-computacional/equipe/igor.jpg) **Igor Montagner** *Professor-23a*


