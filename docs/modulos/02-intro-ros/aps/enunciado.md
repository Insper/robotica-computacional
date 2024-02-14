# Entregável 4 de Robótica Computacional

## Instruções gerais

**Aviso 1:** Sempre desenvolvam nos arquivos `.py` dos respectivos exercícios.

**Aviso 2:** Lembre-se de dar `commit` e `push` no seu repositório até o horário limite de entrega.

**Aviso 3:** Preencha o nome completo dos integrantes do seu grupo no arquivo `README.md` do seu repositório.

**Aviso 4:** Além de seu repositório, para todas as questões você **deve gravar um vídeo do seu robô executando a tarefa**. O vídeo deve ser feito gravando a tela do linux, [tutorial](https://insper.github.io/robotica-computacional/screen_record/), e deve ser postado no Youtube. 

No arquivo `README.md` do seu repositório existe o campo `Link do Vídeo` onde você deve colocar o link do video no youtube. Certifique-se de que o vídeo está público e que o link está correto. `NUNCA de commit no vídeo`, somente adicione o link.

## Configuração do Pacote (ROS 2)

- **Preparação Inicial:** Primeiro, aceite o convite do GitHub Classroom e clone o repositório **dentro da pasta** `colcon_ws/src/` no seu SSD.
- **Criação do Pacote ROS 2:** **Dentro do diretório do seu repositório**, crie um novo pacote nomeado `entregavel_2`.

# Exercício 1 (5 pontos)

## Instruções:
Baseando-se no código `first_node.py` do módulo 2, crie um nó denominado `publisher` que publique uma mensagem no tópico `publisher` do tipo `std_msgs/String`. 

A mensagem deve conter:

* O horário atual em segundos;

* Um contador que começa em 0 e é incrementado a cada mensagem publicada;

* Ambos separados por um espaço. Ou seja, a mensagem deve ter o formato: "**{tempo_atual}** **{contador}**".

O nó também deve imprimir no terminal uma alerta como na linha a seguir:

```bash
Ola, são 1677878366175707817 e estou publicando pela 117 vez
```

Utilize o comando `ros2 topic echo /publisher` para verificar se o exercício está correto.

**DICA 1** - Para pegar o horário atual
```python 
tempo_sec = rospy.Time.now().to_sec()
```
## Critérios de Avaliação:
1. **(+ 0,5)** O pacote foi criado corretamente.
2. **(+ 1,0)** O publicador foi criado corretamente.
4. **(+ 2,0)** A mensagem foi criada e publicada corretamente.
5. **(+ 0,5)** O arquivo `setup.py` foi configurado corretamente.
6. **(+ 1,0)** O nó funciona corretamente.


# Exercício 2 (5 pontos)

## Instruções:
Baseando-se no código `second_node.py` do módulo 2, crie um nó denominado `subscriber` que se inscreva no tópico `publisher` do tipo `std_msgs/String`. A cada nova mensagem recebida, a função `callback` deve separar o tempo do contador no conteúdo da mensagem. Lembre-se de checar a estrutura da mensagem.

A função `control` deve calcular o tempo que passou e imprimir número da mensagem recebida e o delay entre quando a messagem foi publicada e quando foi recebida, como no exemplo a seguir,

```bash
Ola, estou recebendo a mensagem: 217 que demorou 0.005347013 segundos para ser recebida
```

**DICA 1** - Pode carregar um valor float referente a um tempo da seguinte forma:

!!! exercise long 
    Qual a estrutura da mensagem do tipo `String`?

    !!! answer
        `string data`. O conteúdo da mensagem é armazenado na variável `data`. Então para acessar o conteúdo, deve-se utilizar `msg.data`. Depois pode separar o tempo do contador utilizando o comando `msg.data.split()`.

## Critérios de Avaliação:
1. **(+ 0,5)** Definiu corretamente o subscriber.
2. **(+ 2,0)** A função `callback` foi implementada corretamente.
3. **(+ 1,0)** A função `control` foi implementada corretamente.
4. **(+ 0,5)** O arquivo `setup.py` foi configurado corretamente.
5. **(+ 1,0)** O nó funciona corretamente.

# Entrega
Grave um vídeo com dois terminais, um com o comando rodando o nó `publisher` e outro rodando o nó `subscriber`. O vídeo deve mostrar o funcionamento do nó `publisher` e o nó `subscriber` recebendo as mensagens e imprimindo o delay entre a publicação e a recepção da mensagem.

O vídeo deve ser postado no Youtube e o link deve ser adicionado no arquivo `README.md` do seu repositório.

## Caso Tenha Apenas Desenvolvido o Exercício 1!

Neste caso, grave um vídeo com dois terminais, um com o comando rodando o nó `publisher` e outro com o comando `ros2 topic echo /publisher`. O vídeo deve mostrar o funcionamento do nó `publisher`.

O vídeo deve ser postado no Youtube e o link deve ser adicionado no arquivo `README.md` do seu repositório.

