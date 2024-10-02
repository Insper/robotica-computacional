# Utilizando a Garra

# Entendendo a utilização da garra
A utilização da garra no nosso robô é bem simples. Para mover o **ombro** publique no tópico `/joint1_position_controller/command` uma mensagem do tipo `std_msgs.Float64`.
* Para colocar o ombro pra cima, publique um valor de `1.5`.
* Para colocar o ombro pra baixo, publique um valor de `-1.0`.
* Para colocar o ombro pra frente, publique um valor de `0.0`.

Para mover a **garra** publique no tópico `/joint2_position_controller/command` uma mensagem do tipo `std_msgs.Float64`.
* Para abrir a garra, publique um valor de `-1.0`.
* Para fechar a garra, publique um valor de `0.0`.

## Módulo da Garra

Vamos encapsular a os comandos da garra em uma classe que pode ser facilmente importado em qualquer nó na ROS 2.

Dentro do pacote `robcomp_util`, crie um arquivo denominado `garra.py` e uma classe chamada `Garra` sem herança. Essa classe deve:

* Não iniciar um nó.

* Inicialize uma variável `self.delay`, que será utilizada para controlar o tempo de espera enquanto a garra se move.

* Definir os publishers `self.ombro_pub` e `self.garra_pub` que publicam no tópico `/joint1_position_controller/command` e `/joint2_position_controller/command`, respectivamente.

* Definir um método `controla_garra` que recebe um commando do tipo `str` e executa o movimento da garra de acordo com o comando recebido. O método deve:
    * Se receber o comando **abrir** abre a garra.
    * Se receber o comando **fechar** fecha a garra.
    * Se receber o comando **cima** move o ombro para cima.
    * Se receber o comando **baixo** move o ombro para baixo.
    * Se receber o comando **frente** move o ombro para frente.