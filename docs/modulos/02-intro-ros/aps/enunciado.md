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
Baseando-se no código `first_node.py` do módulo 2, crie um nó denominado `publisher` que publique uma mensagem no tópico `publisher` do tipo `std_msgs/String`. 

**A mensagem deve conter:**

* O horário atual em segundos;

* Um contador que começa em 0 e é incrementado a cada mensagem publicada;

* Ambos separados por um espaço. Ou seja, a mensagem deve ter o formato: "**{tempo_atual}** **{contador}**".

O nó **também deve imprimir no terminal** uma alerta como na linha a seguir:

```bash
Ola, são 1677878366175707817 e estou publicando pela 117 vez
```

!!! importante
    Note que a mensagem e o que deve ser impresso no terminal são diferentes!
    A **mensagem** deve conter o tempo atual e o contador separados por um espaço. Já o que deve ser impresso no terminal é uma string mais descritiva.

Utilize o comando `ros2 topic echo /publisher` para verificar se o exercício está correto.

!!! tip
    **DICA:** Para pegar o horário atual

    ```python 
    current_time = self.get_clock().now().to_msg()
    print(f"Horário atual: {current_time.sec}.{current_time.nanosec}")
    ```

# Exercício 2 (4 pontos)

## Instruções:
Baseando-se no código `second_node.py` do módulo 2, crie um nó denominado `subscriber` que se inscreva no tópico `publisher` do tipo `std_msgs/String`. A cada nova mensagem recebida, a função `callback` deve separar o tempo do contador no conteúdo da mensagem. Lembre-se de checar a estrutura da mensagem.

A função `control` deve calcular o tempo que passou e imprimir número da mensagem recebida e o delay entre quando a messagem foi publicada e quando foi recebida, como no exemplo a seguir,

```bash
Ola, estou recebendo a mensagem: 217 que demorou 0.005347013 segundos para ser recebida
```

!!! exercise long 
    Qual a estrutura da mensagem do tipo `String`?

    !!! answer
        `string data`. O conteúdo da mensagem é armazenado na variável `data`. Então para acessar o conteúdo, deve-se utilizar `msg.data`. Depois pode separar o tempo do contador utilizando o comando `msg.data.split()`.

# Entrega
Grave um vídeo com dois terminais, um com o comando rodando o nó `publisher` e outro rodando o nó `subscriber`. O vídeo deve mostrar o funcionamento do nó `publisher` e o nó `subscriber` recebendo as mensagens e imprimindo o delay entre a publicação e a recepção da mensagem.

O vídeo deve ser postado no Youtube e o link deve ser adicionado no arquivo `README.md` do seu repositório.

# Exercício 3 ( 2 pontos)
Crie um `bag` com as mensagens publicadas e grave um novo vídeo mostrando o nó `subscriber` recebendo as mensagens do bag. Lembre-se de parar a execução do nó `publisher` antes de rodar o nó `subscriber` e o `ros2 bag play`.

O vídeo deve ser postado no Youtube e o link deve ser adicionado no arquivo `README.md` do seu repositório.


## Caso Tenha Apenas Desenvolvido o Exercício 1!

Neste caso, grave um vídeo com dois terminais, no primeiro terminal com o comando rodando o nó `publisher` e no segundo terminal execute o comando `ros2 topic echo /publisher`. O vídeo deve mostrar o funcionamento do nó `publisher`.

O vídeo deve ser postado no Youtube e o link deve ser adicionado no arquivo `README.md` do seu repositório.

