# Robótica Computacional - Atividade 1 do projeto

Atenção: o grupo de 3 pessoas desta atividade deve se manter fixo até a entrega do projeto. Você **pode trocar** o grupo com que fez atividades anteriores. 


**Data de entrega: 6/04**

A entrega deverá ser via Blackboard com **código** e **comprovação em vídeo**. 


No ROS a OpenCV trabalha com base em eventos. Esta atividade permite que você estude isso mais a fundo


## 1. Setup da infra


*Apague* a pasta `robot19`, se você tiver:

```

    nautilus ~/catkin_ws/src
```



Faça um clone de [https://github.com/insper/robot20/](https://github.com/insper/robot20/) **dentro** de sua pasta `catkin_ws/src`.

    cd ~/catkin_ws/src
    git clone https://github.com/insper/robot20/

Atualize ou clone o `my_simulation`:


```bash

    cd ~/catkin_ws/src/my_simulation
    git pull  

```

Certifique-se de que o caminho dos modelos contidos no `my_simulation` esteja contido no `.bashrc` (olhe se contém a linha abaixo): 

    export GAZEBO_MODEL_PATH=~/catkin_ws/src/my_simulation/models:${GAZEBO_MODEL_PATH}

Lembre-se de **sempre** executar o `catkin_make` depois de criar novos arquivos `*.py`

    cd ~/catkin_ws
    catkin_make

## Para executar



Vamos subir o nosso mundo simulado

```bash    

    roslaunch my_simulation sala404_creepers.launch

````

Neste momento você vai ver algo parecido com o que temos abaixo:

![](sala404_creepers.png)


Estude o código de `cor.py`, que está em `robot20/ros/exemplos_python/scripts` Você pode começar executando este programa.

Lembre-se de que você vai precisar estar conectado a algum robô simulado para poder testar, como o [my simulation ](https://github.com/.arnaldojr/my_simulation).


E em outro terminal

    rosrun exemplos_python cor.py



##  O que é para fazer

Modique este programa para que o robô centralize num creeper **azul** ou **verde**.

Depois de centralizar, usando a informação do *laser* (tópico `\scan`) faça o robô se aproximar do *creeper* e para a $15cm$ dele *ou* tocá-lo gentilmente.  Isso vai ser base para depois incluirmos o comando da garra do robô. 

Você deve testar com o robô a cerca de $1.75 m$ do creeper.

Dica: use o *teleop* para se aproximar do creeper e capture uma imagem para facilitar a escolha de cores *HSV*. Relembre da [atividade da Aula 02](https://github.com/Insper/robot20/blob/master/aula02/aula2_OpenCV_Filtragem.ipynb).


Para fazer o teleop: 

    rqt_image_view

E depois digite:

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Você vai ver uma imagem como a abaixo:

![](rqt_image_view_creepers.png)



Crie [seu próprio projeto](https://github.com/Insper/robot20/blob/master/guides/projeto_rospython.md), não trabalhe na pasta clonada do professor.

Dentro do seu projeto, copie os arquivos `cor.py` e `cor

O jeito sugerido de trabalhar é criar um projeto no Github, clonar seu Github *dentro* do `catkin_ws/src`  e criar um projeto ROS dentro dele.

No seu gerenciador de arquivos do Linux, deve ficar assim:

![](como_criar_projeto.png)

## Vídeo do guia

Em caso de dúvida sobre como proceder, por favor acompanhe o vídeo

[https://www.youtube.com/watch?v=OKphJFyhnSg&feature=youtu.be](https://www.youtube.com/watch?v=OKphJFyhnSg&feature=youtu.be)


Se der alguma coisa errada, consulte esse guia de sobrevivência ROS Gazebo, com alguns problemas comuns e suas soluções

[https://www.youtube.com/playlist?list=PLM8rZg4fCalht-rexa91MO1y3jxm9mwOa](https://www.youtube.com/playlist?list=PLM8rZg4fCalht-rexa91MO1y3jxm9mwOa)
