
# Projeto 1

Novo deadline 19/11


```
      Nov 2021        
Su Mo Tu We Th Fr Sa  
    1  2  3  4 5 6  
 7  8  9 10 11 12 13  
14 15 16 17 18 *19* 20  
21 22 23 24 25 26 27  
28 29 30

```

# Iniciando o projeto

Todos os integrantes do seu grupo deverão aceitar a tarefa no Github classroom que se encontra neste link:
https://classroom.github.com/a/KHtHUd-S


Para criar a pasta do projeto, siga as instruções em:
https://github.com/Insper/robot21.2/blob/main/guides/projeto_rospython.md


Em especial atenção ao nome da pasta do reposório do projeto, como está especificado em:
https://github.com/Insper/robot21.2/blob/main/guides/projeto_rospython.md#usando-o-github-classroom


# Rubricas


**Conceito I - Mínimo para ter direito à Delta**

O robô percorre toda a pista, parando onde começou, usando
a odometria para saber de onde saiu. É preciso gravar todo o circuito,  que deve ser percorrido em mendos de 30 min.



**Conceito C**

Robô é capaz de percorer toda a pista, parando onde começou, e no meio do caminho se chocar contra um creeper da cor certa, voltando depois à pista. A cor do creeper deve ser escolhida como um parâmetro do programa, ou seja, deve funcionar para qualquer cor pedida. É preciso gravar todo o circuito, que deve ser percorrido em mendos de 30 min.


**Conceito C+**

Robô é capaz de percorer toda a pista, parando onde começou, e no meio do caminho encostar em um creeper da cor certa, voltando depois à pista. A cor do creeper deve ser escolhida como um parâmetro do programa, ou seja, deve funcionar para qualquer cor pedida. 

Encostar no creeper significa, especificamente: chegar bem perto do creeper com a garra abaixada, e então parar o robô e levantar a garra.

É preciso gravar todo o circuito, que deve ser percorrido em mendos de 30 min.


**Conceito B** 

O robô deve percorrer toda a pista, parando onde começou, e no meio do caminho pega o creeper da cor e ID corretos com a garra e volta para a pista. 

A cor e o ID devem ser parâmetros do programa, ou seja, ele deve funcionar para qualquer valor de cor e ID. 

O código tem que estar bem modularizado e cada função deve estar documentada explicando de forma resumida o que faz, como no exemplo abaixo:

```python

def soma(a,b):
	“””
	Retorna a soma de a e b
	”””
  	return a + b
```

É preciso gravar todo o circuito, que deve ser percorrido em mendos de 30 min.

**Conceito B+** 

Além dos itens do conceito B, o robô deve deixar o creeper na na base certa e completar a volta na pista.
A base também deve ser fornecida como parâmetro do projeto. 


**Conceito A**

Itens do conceito B+ um uso de classes e objetos Python    

Só pode ter sleep dentro do `while` principal. 

Fazer um dentre os cinco *especiais*:
* Uso de classes e objetos Python em todos os arquivos criados para o projeto
* Gravar e filmar no robô real funcionado
* Fazer um controle proporcional derivativo ou PD para manter o robô na pista e fazer funcionar rápido baseado no ângulo de visão da pista.
* Mapear os Arucos usando o modo 3D e mapeamento (https://github.com/Insper/404/blob/master/tutoriais/robotica/navigation_gazebo_simulador.md)
* Estruturar o programa com pelo menos um node ROS prestando serviço para o outro.

Para saber como implementar controle proporcional derivativo ou PD se inspire [neste link](https://www.a1k0n.net/2018/11/13/fast-line-following.html)


**Conceito A+**

Itens do conceito A e fazer 3 especiais em vez de apenas um.

**Conceito A+ para grupos de 4**

Itens do conceito A e fazer todso os 5 especiais

## Grupos de 4 pessoas 

Os grupos de 4 pessoas terão sempre um conceito deslocado em relacão aos de 3 pessoas.

Para obter A+ num grupo de 4, é necessário  fazer os 4 especiais.

Assim, o necessário para conceito B em grupos de 3 representa o conceito C+ em grupos de 4.


<img src="./pista virtual.png">


# Objetivos 

Cores válidas do creeper: blue, green, pink Estações válidas: dog, horse, cow e car



### Exemplos de objetivos: 

Os objetivos abaixo são exemplos de objetivos possíveis.  O seu robô deve ser capaz de realizar *qualquer* objetivo. 


```python
goal1 = ("blue", 22, "dog")

goal2 = ("green", 13, "dog")

goal3 = ("orange", 11, "cat")
```


Por exemplo, o objetivo `("blue", 22, "dog")` significa *Encontre o creeper azul de ID 22 e o traga até a caixa com figura de cão*. 

A lista de todas as possibilidades que seu programa pode encontrar [está neste link](./todas_possibilidades.md). Lembre-se de que o código deve estar preparado para funcionar com *qualquer uma*. 


# Objetivos que devem ser filmados 

```python
goal1 = ("blue", 12, "dog")

goal2 = ("green", 23, "car")

goal3 = ("orange", 11, "cow")
```


# Instruções

Comandos para atualizar os repositório
```bash
    cd ~catkin_ws/src/mybot_description
    git pull
    cd ~catkin_ws/src/my_simulation
    git pull
    cd ~catkin_ws/src/robot21.2
    git pull
```

Para executar:

	roslaunch my_simulation trevo.launch

Para habilitar o controle da garra executar:

	roslaunch mybot_description mybot_control2.launch 	


Como atividade inicial, sugiro que tente fazer o robô *seguir a pista* . Você pode se basear em sua Atividade 3, ou ainda desenvolver uma abordagem baseada em centro de massa da linha amarela, como [encontrada neste link](https://github.com/osrf/rosbook/blob/master/followbot/follower_color_filter.py)




# Exemplo do ARUCO 

Exemplo de como interpretar os markers ARUCO 
[./aruco/aruco.ipynb] (./aruco/aruco.ipynb)

Exemplo de como programar usando os markers ARUCO em 3D 
[https://github.com/Insper/robot21.2/blob/master/ros/exemplos/scripts/aruco.py](https://github.com/Insper/robot21.2/blob/master/ros/exemplos/scripts/aruco.py)

# Instruções sobre os tópicos da garra 

[Fonte: https://github.com/arnaldojr/mybot_description/](https://github.com/arnaldojr/mybot_description/)

Launch para subir os controles da garra e RViz

    roslaunch mybot_description mybot_control2.launch 

Para publicar um topico da garra:

Joint1 = braço da garra. Valores máximos:

    Garra recolhida: -1
    Garra para frente: 0
    Garra levantada: 1.5
    
    No terminal:
    rostopic pub -1 /joint1_position_controller/command std_msgs/Float64 "data: 0"
    
Joint2 = Pinça da garra.

    Pinça fechada: 0
    Pinça aberta: -1
    
    No terminal:
    rostopic pub -1 /joint2_position_controller/command std_msgs/Float64 "data: 0"
    
Visualizar arvore:

    rosrun rqt_gui rqt_gui 
    
Exemplo de codigo py

[https://github.com/Insper/robot21.2/blob/master/ros/exemplos/scripts/move_garra.py](https://github.com/Insper/robot21.2/blob/master/ros/exemplos/scripts/move_garra.py)


