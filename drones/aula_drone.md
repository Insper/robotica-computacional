# Semana de competição de drones

Nesta semana iremos trabalhar com o drone Bebop, da Parrot. Na aula de laboratório (terça ou quinta), iremos à quadra do 5o. andar do prédio 1 para participarmos de uma competição entre os grupos do projeto.

Cada grupo irá receber um drone e um SSD configurado com os programas para controlar o Bebop usando o ROS. Veja o [guia](https://github.com/Insper/bebop_sphinx/blob/master/docs/bebop_tutorial.md#como-conectar-no-drone-se-voc%C3%AA-j%C3%A1-tem-o-bebop_autonomy-instalado) para se conectar ao Bebop real, que funciona apenas com o SSD fornecido.

No cenário da competição haverá três caixas de cores distintas. O objetivo da competição é criar um programa em Python que faça o drone sobrevoar as três caixas duas vezes, parando por 1 segundo sobre cada uma, fazendo o percurso no menor tempo possível. Veja que quando o Bebop ficar sobre a caixa ela poderá não ser visível, então a isso deve ser tratado com lógica de programação.

O desenvolvimento do programa deve ser realizado com o auxílio do simulador do Bebop. Siga os passos no [guia do simulador do Bebop] (https://github.com/Insper/404/blob/master/tutoriais/robotica/guia_drone_ros_noetic.md).

Após instalar o simulador, você pode simular um cenário com caixas através do comando abaixo:

```bash
roslaunch exemplos212 mav_fake_driver.launch 
```

Então faça o *take off* e execute o programa de exemplo, que está na pasta [ros/exemplos](./ros/exemplos).

```bash
rosrun exemplos212 visao_bebop.py 
```

Não se esqueça de que o SSD do programa que se coneecta ao robô real possui apenas o Python 2, então talvez seja necessário adaptar o programa. 

No dia da competição, cada grupo terá direito a 5 saídas com o drone, com o objetivo de:

- Capturar imagens para ajustar as faixas de cores
- Ajustar as temporizações para a dinâmica do Bebop real
- Fazer a tomada de tempo

Não se esqueça de ter sempre um computador de backup com o seguinte comando pronto para parar o Bebop em caso de emergência:

```
rostopic pub --once /bebop/reset std_msgs/Empty
```

Boa sorte!


