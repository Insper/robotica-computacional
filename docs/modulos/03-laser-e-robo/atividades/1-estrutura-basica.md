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