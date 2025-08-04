# Entregável 4 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom.
- Você deve fazer o clone do repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_2`.

# Exercício 1 (4 pontos)

## Instruções:
Baseando-se no código `first_node.py` do módulo 2, crie um arquivo chamado `publisher.py` com um nó denominado `publisher_node` que publique uma mensagem no tópico `publisher` do tipo `std_msgs/PubSub`. 

**A mensagem deve conter:**

* O horário atual em segundos desde a época Unix (1 de janeiro de 1970) com precisão de nanossegundos;

* Um contador que começa em 0 e é incrementado a cada mensagem publicada;

* Todos nos campos apropriados para o tipo `robcomp_interfaces/PubSub`.

O nó **também deve imprimir no terminal** uma alerta como na linha a seguir:

```bash
Ola, são 1677878366175707817.935347013 e estou publicando pela 217 vez
```

O nó pode ser iniciado com o comando `ros2 run entregavel_2 publisher`.
Utilize o comando `ros2 topic echo /publisher` para verificar se o exercício está correto.

!!! tip
    **DICA 1:** Verifique a estrutura da mensagem do tipo `PubSub` na ROS 2 usando o comando:

    ```bash
    ros2 interface show robcomp_interfaces/PubSub
    ```

!!! tip
    **DICA 2:** Para pegar o horário atual

    ```python 
    current_time = self.get_clock().now().to_msg()
    current_time = float(current_time.sec) + float(current_time.nanosec)/10**9
    print(f"Horário atual: {current_time}")
    ```
!!! exercise long 
    Qual a estrutura da mensagem do tipo `PubSub`?

    !!! answer
        Esse tipo de mensagem contém os seguintes campos:
        - `float time`
        - `int counter`
        ou seja, a mensagem é composta por um campo de tempo do tipo `float` e um contador do tipo `int`.

# Exercício 2 (4 pontos)

## Instruções:
Baseando-se no código `second_node.py` do módulo 2, crie um arquivo chamado `subscriber.py` com um nó denominado `subscriber_node` que se inscreva no tópico `publisher` do tipo `robcomp_interfaces/PubSub`.

A função `control` deve calcular o tempo que passou e imprimir número da mensagem recebida e o delay entre quando a messagem foi publicada e quando foi recebida, como no exemplo a seguir,

```bash
Ola, estou recebendo a mensagem: 217 que demorou 0.005347013 segundos para ser recebida
```



# Entrega

!!! tip
    Como desejamos ver a saída do terminal do `subscriber.py` e do `publisher.py`, é importante executá-los utilizando os comandos abaixo para as saída aparecerem corretamente (utilizando `ros2 run`).
    
    ```bash
    ros2 run entregaval_2 publisher
    ```
    e
    ```bash
    ros2 run entregaval_2 subscriber
    ```

Grave um vídeo com dois terminais, um com o comando rodando o nó `publisher` e outro rodando o nó `subscriber`. O vídeo deve mostrar o funcionamento do nó `publisher` e o nó `subscriber` recebendo as mensagens e imprimindo o delay entre a publicação e a recepção da mensagem.

O vídeo deve ser postado no Youtube e o link deve ser adicionado no arquivo `README.md` do seu repositório.

# Exercício 3 ( 2 pontos)
Crie um `bag` com as mensagens publicadas e grave um novo vídeo mostrando o nó `subscriber` recebendo as mensagens do bag. Lembre-se de parar a execução do nó `publisher` antes de rodar o nó `subscriber` e o `ros2 bag play`.

O vídeo deve ser postado no Youtube e o link deve ser adicionado no arquivo `README.md` do seu repositório.


## Caso Tenha Apenas Desenvolvido o Exercício 1!

Neste caso, grave um vídeo com dois terminais, no primeiro terminal com o comando rodando o nó `publisher` e no segundo terminal execute o comando `ros2 topic echo /publisher`. O vídeo deve mostrar o funcionamento do nó `publisher`.

O vídeo deve ser postado no Youtube e o link deve ser adicionado no arquivo `README.md` do seu repositório.

