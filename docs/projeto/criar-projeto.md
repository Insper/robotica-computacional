# Criando um projeto ROS

O ROS trabalhará sempre no diretório `~/catkin_ws/src`, então precisaremos colocar o código do nosso projeto lá. 

!!! tip
    Quando usamos `~` em um caminho esse valor é substituído pela pasta *home* do usuário atual. Logo, no SSD de robótica isso será `/home/borg`, pois todos os SSDs tem por padrão o usuário de nome `borg`

Execute os comandos abaixo em um terminal. 

```bash
cd ~/catkin_ws/src
catkin_create_pkg projeto-robcomp std_msgs sensor_msgs geometry_msgs rospy roscpp
```

A saída deve ser parecida com a abaixo.

```bash
borg@ubuntu:~/catkin_ws$ cd ~/catkin_ws/src
borg@ubuntu:~/catkin_ws/src$ catkin_create_pkg projeto-robcomp std_msgs sensor_msgs geometry_msgs rospy roscpp
Created file projeto-robcomp/CMakeLists.txt
Created file projeto-robcomp/package.xml
Created folder projeto-robcomp/include/meu_projeto
Created folder projeto-robcomp/src
Successfully created files in /home/borg/catkin_ws/src/projeto-robcomp. Please adjust the values in package.xml.
```

Vamos criar um diretório para os programas Python:

```bash
cd ~/catkin_ws/src/projeto-robcomp
mkdir scripts
cd scripts
```

Todos nossos programas Python serão armazenados nesta pasta. Abra a pasta do seu projeto no VSCode e crie um novo arquivo `rodar.py` na pasta `scripts` criada acima.

```python
#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3

v = 0.2  # Velocidade linear
w = 0.5  # Velocidade angular

if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
            vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
            pub.publish(vel)
            rospy.sleep(2.0)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
```

!!! important
    Toda vez que **adicionar** novos programas na pasta `scripts` execute o comando `catkin_make` para registrá-los no ROS. 

Para rodar esse programa de testes 

1. lance a simulação da ísta que usamos nos exercícios
2. execute o comando `rosrun projeto-robcomp rodar.py`

Se deu tudo certo então adicione os arquivos criados no projeto e faça um commit para seus colegas receberem a estrutura inicial do projeto.
