## Controlando o robô via código - TODO

Agora, com tudo limpo, tudo em paz, crie um arquivo python em branco e cole o código abaixo.

Para criar o arquivo:

```bash
code ~/roda_robozinho.py
```

Para dar permissão de execução para o arquivo:

```bash
chmod a+x ~/roda_robozinho.py
```

Código em python que faz o robô andar em círculos eternamente:

```bash
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys

#função que publica valores de velocidade no robô
def move_robot(lin_vel,ang_vel,sleep):
    #inicializando um node no ROS
    rospy.init_node('move_robot', anonymous=True) 

    #definindo o tópico que será utilizado pela função, o tipo da mensagem e o 
    #tamanho da fila de mensagens que serão enviadas para o tópico definido
		#no caso é cmd_vel, topico do tipo publisher que controla os motores do robô
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)

		#definindo um metodo to tipo Twist()
		#http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    vel = Twist()       

    vel.linear.x = lin_vel # Velocidade linear (eixo x)
    vel.linear.y = 0
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = ang_vel  # Velocidade angular (eixo z)

		# exibindo no terminal a velocidade linear e a velocidade angular atual
    rospy.loginfo("Linear Vel = %f: Angular Vel = %f",lin_vel,ang_vel)
		# publicando a velocidade via ROS no robô
    pub.publish(vel)
		# aguardando um tempinho para dar tempo de executar o comando
    rospy.sleep(sleep)

if __name__ == '__main__':

     while not rospy.is_shutdown(): #loop do ROS
				#enviando os valores de velocidade para a função move_robot
        move_robot(0.5,0.5,0.5) 
```

!!! tip
    Você pode executarseu programa tanto com `python3 ~/roda_robozinho.py` quanto com `~/roda_robozinho.py`. Isso é possível pois demos permissão de execução ao programa (com o comando `chmod` acima) e colocamos na primeira linha que queremos executar esse programa usando o comando `python3`

O seu resultado deve ser algo parecido com isso:

![robo_python.gif](imgs/robo_python.gif)