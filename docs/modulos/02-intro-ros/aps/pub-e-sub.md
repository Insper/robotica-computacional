# Exercícios ROS
No arquivo `base.py` fornecemos uma estrutura básica de um script da ROS usando classe no python.

A função `main.py` cria um nó com o nome `'Controler'` e construir um a classe `Control()`, depois até que o `core` da ROS seja interrompido vai executar o conteúdo da função `Control().control` em loop.

```python
def main():
	rospy.init_node('Controler')
	control = Control()
	rospy.sleep(1) # Espera 1 segundo para que os publishers e subscribers sejam criados

	while not rospy.is_shutdown():
		control.control()
```

Para fazer com que o nó se inscreva em um tópico deve-se primeiro pegar o tipo de mensagem e então comando com a seguinte estrutura de comando pode se inscrever no tópico:
```python
self.sub = rospy.Subscriber({topico},{tipo de mensagem},self.callback)
```

Sempre que uma nova mensagem seja enviada ao tópico “topico”, a função `self.callback` será executada com esta nova mensagem.

Para fazer com que um nó publique em um tópico, utilize a seguinte estrutura de comando: 

```python
self.pub = rospy.Publisher({topico},{tipo de mensagem},queue_size=10)
```

O argumento `queue_size` indica o tamanho máximo da fila de mensagens. Em condições normais a fila não ultrapassa o tamanho 1, este argumento só é relevante quando o conteúdo da mensagem é muito extenso, como uma imagem.

## Antes de começar
Após extrair os arquivos e rodar `catkin_make`, para rodar os exercícios, execute os seguintes comandos no terminal:

```bash
roscd modulo4
cd scripts
chmod +x *.py
```

## Q1 - Publisher
Começando do arquivo `publisher.py` complete as partes do código com ??? para que o código funcione sem erros. O nó deve publicar uma mensagem no tópico `publisher` do tipo `std_msgs/String` contendo o horário atual e um o número da mensagem enviada, **separadas por um espaço**. Também deve imprimir no terminal uma alerta utilizando o comando `rospy.loginfo` com a seguinte estrutura:

```bash
[INFO] [1677878366.175759]: Ola, são 1677878366175707817 e estou publicando pela 117 vez
```

Utilize o comando `rostopic echo publisher` para verificar se o exercício está correto.

**DICA 1** - Para pegar o horário atual
```python 
tempo = rospy.Time.now()
tempo_sec = rospy.Time.now().to_sec()
```
??? details "Resposta"
    [Resposta](../modulo4/scripts_resp/publisher.py){ .ah-button }

# Q2 - Subscriber
Agora vamos trabalhar em um nó que se inscreve no tópico que criamos no exercício anterior. A função `callback`, deve separar o tempo do contador no conteúdo da mensagem,lembre-se de checar a estrutura da mensagem. A função `control` deve calcular o tempo que passou e utilizar o comando `rospy.loginfo` para mostrar o número da mensagem e o delay dela no terminal, como no exemplo a seguir,

```bash
[INFO] [1677878948.424955]: Ola, estou recebendo a mensagem: 217 e se passaram 0.005347013 segundos
```

**DICA 1** - Pode carregar um valor float referente a um tempo da seguinte forma:
```python 
time = rospy.Time( float( rospy.Time.now().to_sec() ) )
```

!!! exercise long 
    Qual a estrutura da mensagem do tipo `String`?

    !!! answer
        `string data`. O conteúdo da mensagem é armazenado na variável `data`. Então para acessar o conteúdo, deve-se utilizar `msg.data`. Depois pode separar o tempo do contador utilizando o comando `msg.data.split()`.

??? details "Resposta"
    [Resposta](../modulo4/scripts_resp/subscriber.py){ .ah-button }