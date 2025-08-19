# Entregável 2 de Robótica Computacional

## Instruções Gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembrem-se de fazer `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencham o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além do seu repositório, para todas as questões vocês **devem gravar um vídeo do robô executando a tarefa**. O vídeo deve ser feito gravando a tela do Linux ([tutorial](https://insper.github.io/robotica-computacional/screen_record/)) e deve ser postado no YouTube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo`, onde vocês devem inserir o link do vídeo no YouTube. Certifiquem-se de que o vídeo está público e que o link está correto. **NUNCA façam commit do vídeo**, apenas adicionem o link.

## Atualização do Pacote (ROS 2)

Execute os comandos abaixo para atualizar os pacotes da `ros2` obrigatórios para a prova:

```bash
cd ~/colcon_ws/src/my_simulation
git stash
git pull
cb
```

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom.
- Você deve fazer o clone do repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_2`.

# Exercício 1 (5 pontos)

## Instruções:

Com base no `first_node.py` do módulo 2, crie `publisher.py` com um nó chamado **`publisher_node`** que **publique** no tópico **`/publisher`** mensagens do tipo `robcomp_interfaces/PubSubAPS`.

**A mensagem deve conter:**

* O **horário atual** em segundos desde a época Unix (1/1/1970), com fração de **nanossegundos**;
* Um **contador** que começa em `0` e é incrementado a cada publicação;
* **Todos os campos obrigatórios** definidos em `robcomp_interfaces/PubSubAPS`.

O nó também deve **imprimir no terminal** uma linha como:

```bash
Olá, são 1677878366.175707817 e estou publicando pela 217ª vez
```

O nó pode ser iniciado com:

```bash
ros2 run entregavel_2 publisher
```

Use para verificar se o nó está publicando:

```bash
ros2 topic echo /publisher
```

!!! tip
    **DICA 1:** Verifique a estrutura da mensagem do tipo `PubSubAPS` na ROS 2 usando o comando:

    ```bash
    ros2 interface show robcomp_interfaces/msg/PubSubAPS
    ```

!!! tip
    **DICA 2:** Tempo atual em segundos (com nanos):

    ```python 
    current_time = self.get_clock().now().to_msg()
    current_time = float(current_time.sec) + float(current_time.nanosec)/10**9
    ```

!!! exercise long 
    Qual a estrutura da mensagem do tipo `PubSubAPS`?

    !!! answer
        Esse tipo de mensagem contém os seguintes campos:
        - `int8 counter`
        - `float34 time`
        ou seja, a mensagem é composta por um campo de tempo do tipo `float32` e um contador do tipo `int8`.

# Exercício 2 (5 pontos)

## Instruções:

Com base no `second_node.py` do módulo 2, crie `subscriber.py` com um nó chamado **`subscriber_node`** que **assine** o tópico **`/publisher`** do tipo `robcomp_interfaces/PubSubAPS`.

A função `control` deve calcular o **tempo decorrido** e **imprimir** o número da mensagem recebida e o **delay** entre a publicação e a recepção, como no exemplo:

```bash
Olá, estou recebendo a mensagem: 217 que demorou 0.005347013 segundos para ser recebida
```

Para executar:

```bash
ros2 run entregavel_2 subscriber
```

---

# Entrega

!!! tip
    O subscriber só vai receber as mensagens se o publisher estiver rodando, então certifique-se de que o nó `publisher` está rodando antes de iniciar o nó `subscriber`.
    
    ```bash
    ros2 run entregavel_2 publisher
    ```
    e
    ```bash
    ros2 run entregavel_2 subscriber
    ```

Grave um vídeo com dois terminais, um com o comando rodando o nó `publisher` e outro rodando o nó `subscriber`. O vídeo deve mostrar o funcionamento do nó `publisher` e o nó `subscriber` recebendo as mensagens e imprimindo o delay entre a publicação e a recepção da mensagem.

O vídeo deve ser postado no Youtube e o link deve ser adicionado no arquivo `README.md` do seu repositório.
