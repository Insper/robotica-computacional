# Projeto final

Atenção: este <font color=red>**não**</font> é o enunciado do projeto final. São só insumos para *brainstorm* dos grupos

## Objetivos

* Grupo funcionar bem como equipe
* Aprendizado de um novo conceito / técnica
* O projeto precisa ter uma finalidade clara
* Ter um demo claro
* Deixar uma presença online
* Fazer uma apresentação


## Entregáveis

* Artigo de 5 páginas

* Presença online: blog ou páginas

* Vídeo de um demo (ou demo online)


# Hardware disponível


* Turtlebot Burger (20) e [Turtlebot Waffle (1)](https://www.youtube.com/watch?v=V8VJUkWWaO8)
* Neatos  - robô aspirador (com RPi 3)
* [Intel Realsense F200 - Câmera 3D Embarcada](http://roscon.ros.org/2016/presentations/ROSCon2016_Intel_RealSense.pdf) (http://roscon.ros.org/2015/presentations/ROSCon-2015-Intel-RealSense-Robotics-Innocation-Program-ROS-interface.pdf)
* [Intel Robotics Kit with Realsense - Câmera 3D embarcada](https://software.intel.com/en-us/realsense/previous)
* [Microsoft Kinect V2](https://www.youtube.com/watch?v=YZwlt2msvpI)
* [Parrot Bebop 2 - Drone](https://www.youtube.com/watch?v=INXMAZvtCw8)
* [IMU Razor](https://www.youtube.com/watch?v=NUNXcr_u9pM)


# Idéias para projetos

## Carro autônomo
Desenvolver algoritmos para o carro autônomo usando o simulador da Udacity
[https://github.com/udacity/self-driving-car-sim](https://github.com/udacity/self-driving-car-sim)

Há algumas opções de projeto *default* para usar o carro da Udacity:

* Localização - filtro de partículas
* Aprender a dirigir com um humano [usando redes neurais](http://jeremyshannon.com/2017/02/10/udacity-sdcnd-behavioral-cloning.html). [Como iniciar seu projeto](https://medium.com/towards-data-science/introduction-to-udacity-self-driving-car-simulator-4d78198d301d)
* Manter o carro na faixa




## SLAM 3D (com câmera 3D ou Kinect V2)
SLAM signfica *Simultaneous localization and Mapping*

[https://www.youtube.com/watch?v=AMLwjo80WzI](https://www.youtube.com/watch?v=AMLwjo80WzI)

[https://www.youtube.com/watch?v=_qiLAWp7AqQ](https://www.youtube.com/watch?v=_qiLAWp7AqQ)

[https://www.youtube.com/watch?v=o1GSQanY-Do](https://www.youtube.com/watch?v=o1GSQanY-Do)

[https://www.youtube.com/watch?v=AMLwjo80WzI&t=11s](https://www.youtube.com/watch?v=AMLwjo80WzI&t=11s)

## Robótica afetiva

O enfoque deve ser mais em diagnosticar o estado emocional dos humanos do que criar um robô fofinho. O robô pode ser virtual

As câmeras 3D são um bom sensor para se fazer isso

O exemplo clássico é o Kismet, do MIT.

[https://www.youtube.com/watch?v=8KRZX5KL4fA](https://www.youtube.com/watch?v=8KRZX5KL4fA)


## Robô para Design de Software

Criar um robô pequeno e barato usando Raspberry Pi Zero ou Node MCU para ser usado na disciplinas de Design de Software

Inspiração: [MicroPython](https://micropython.org/)

## Turtlebot 3 Genérico

Criar uma variante do Turtlebot 3 Burger usando hardware mais barato - um servo controller, um Arduino e servos mais econômicos.



## Classificação de objetos em 3D

Criar uma missão para um robô em que seja necessário reconhece rum objeto em 3D. 

Exemplo: [https://www.youtube.com/watch?v=yhVjM5LKuaY](https://www.youtube.com/watch?v=yhVjM5LKuaY)

## Controle de robôs com gestos
Usar uma câmera 3D para entender o que o usuário quer

[https://www.youtube.com/watch?v=Yag0UCiUdwM](https://www.youtube.com/watch?v=Yag0UCiUdwM)

## Robôs para estoque

A Kiva foi uma startup de robótica [adquirida pela Amazon por US$ 775 mi](https://www.therobotreport.com/news/filling-the-void-left-by-kiva-systems-acquisition-by-amazon) .

Uma possível idéia de projeto é criar um robô que transporta items em um estoque de um ponto a outro, planejando a rota.

[https://youtu.be/UtBa9yVZBJM?t=30](https://youtu.be/UtBa9yVZBJM?t=30)


## Atuadores e braços robóticos

Exemplos de projetos:

* Incluir um braço no Turtlebot Waffle  (por exemplo um me-arm) e controlá-lo através do ROS.
* [Atuadores flexíveis](https://www.extremetech.com/extreme/174723-soft-robotic-gripper-uses-vacuum-pressure-and-a-beanbag-to-move-objects)


## Robô humanóide

O ROS Kinetic / Gazebo podem simular robôs humanóides como os do DARPA Challenge.

Missões envolvendo robôs humanóides são projetos válidos
[https://www.youtube.com/watch?v=yVICMC_BAiU](https://www.youtube.com/watch?v=yVICMC_BAiU)



## Fusão sensorial

Fusão sensorial é usar mais de uma fonte de medidas do mesmo tipo para melhorar a precisão. Exemplos:
* Fusão sensorial Alvar + IMU: Usar marcadores + uma unidade de medida de inércia (IMU) para estimar com precisão a posição de um robô
* Fusão sensorial odometria +  IMU - Usar uma IMU para melhorar permitir o SLAM usando o Robotics Kit sem robô


## Controle  do Turtlebot 3 via Android

[Conectar um Android ao Turtlebot 3 via USB OTG e substituir a Raspberry Pi, usando o acelerômetro e a câmera do SmartPhone, e o projeto Android Sensors Driver]
(https://github.com/chadrockey/android_sensors_driver)


## Futebol de robôs com Turtlebot 3

Os Turtlebot 3 são uma boa plataforma para prototipar técnicas do futebol de robôs

[https://www.youtube.com/watch?v=qNaBUs7gP_A](https://www.youtube.com/watch?v=qNaBUs7gP_A)


# Missões com drones

[Swarms de drones](https://www.youtube.com/watch?v=hhpsaITWMoY)

[Estabilização visual de vôo de drones](https://www.youtube.com/watch?v=59TWljDYmB8)

[Slam monocular](https://www.youtube.com/watch?v=ufvPS5wJAx0)

[Visual Navigation for Flying Robots](https://www.youtube.com/watch?v=f5khDPUMYmQ)

[Construção de mosaicos on-line](https://www.youtube.com/watch?v=-kSTDvGZ-YQ)

[Interface natural de controle de drones](https://www.youtube.com/watch?v=3janOGurQbQ)

# Demos do Turtlebot 3 estendidos

Os demos do Turtlebot são open source. Podem servir de base para missões robóticas


[Autorace - vídeo 1](https://www.youtube.com/watch?v=1RBOfPWdpsc) e [outro exemplo](https://www.youtube.com/watch?v=47YnSBAssOM&t=7s)

[Follow](https://www.youtube.com/watch?v=w9YTxZVY6yQ)


Parallel Tracking and Mapping
[http://www.robots.ox.ac.uk/~gk/PTAM/](http://www.robots.ox.ac.uk/~gk/PTAM/)

Visão ativa - odometria visual
[https://ewokrampage.wordpress.com/](https://ewokrampage.wordpress.com/)

Criação de um mosaico de imagens
[http://zhaoyong.adv-ci.com/researches/map2dfusion/]()
[http://eprints.fri.uni-lj.si/2164/1/Kostelec_P_D-1.pdf]()

## Sphero Mist

[Replicar aspectos deste robô, por exemplo a interface que percebe o humor da pessoa](https://www.theverge.com/platform/amp/circuitbreaker/2018/5/2/17311124/misty-robotics-sphero-programmable-robot)


## Se você gostou de PROLOG

Agentes lógicos e controle para um robô [KnowRob](http://www.knowrob.org/blog/application_of_knowrob_tidying_up_a_kitchen)


Controle de NPCs em jogos: [http://web.cs.ucdavis.edu/~vemuri/classes/gamesclass/Cognitive.pdf](http://web.cs.ucdavis.edu/~vemuri/classes/gamesclass/Cognitive.pdf)

[Path planning](https://qiao.github.io/PathFinding.js/visual/)


## Evolução, Algoritmos genéticos

[Evolução de veículos em Javascript:](http://rednuht.org/genetic_cars_2/)

[Evolução de padrões de controle para bípedes](http://robohub.org/pleurobot-multimodal-locomotion-in-a-bioinspired-robot/)

[Evolução de robôs inteiros](https://www.youtube.com/watch?v=JBgG_VSP7f8)

## Explore!

### Artificial Intelligence a Modern Approach
Qualquer tema que conste do  livro texto do curso [Artificial Intelligence - A Modern Approach](http://aima.cs.berkeley.edu/contents.html)


## Redes deep learned

SSD Multibox
[https://towardsdatascience.com/understanding-ssd-multibox-real-time-object-detection-in-deep-learning-495ef744fab](https://towardsdatascience.com/understanding-ssd-multibox-real-time-object-detection-in-deep-learning-495ef744fab)

Demo do YOLO (Vimos em aula)
[https://www.youtube.com/watch?v=VOC3huqHrss](https://www.youtube.com/watch?v=VOC3huqHrss)
Módulo do YOLO para ROS
[https://github.com/pgigioli/darknet_ros](https://github.com/pgigioli/darknet_ros)




### Seja criativo!

Temais mais livres e artísticos, desde que empreguem técnicas de robótica, também são possíveis. Exemplos:
* Um robô que identifica e segue seu dono
* Robôs que desenham no chão (rastreados por Alvar)
