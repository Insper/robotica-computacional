# Salvando Eventos na ROS 2

Nesta atividade vamos aprender nossa primeira ação, a ação de andar uma distancia `d`.
Como estudamos em `Explorando Tópicos e Mensagens`, a velocidade linear do robo pode ser controlada se publicarmos uma mensagem de um certo tipo para um tpoico, mas, em robotica, muitas vezes queremos mover uma distacia conhecidada, mas como fazer isso apenas com a velocidade?

Em robotica uma forma de controlar o robo para se deslocar uma distancia `d` é utilizando um metodo conechecido como `Dead Reckoning`, Neste metodo, você se desloca em **velocidade constante** por um **tempo fixo**, sem receber feedback de quanto, realmente, se deslocou.

