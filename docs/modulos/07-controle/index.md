# Módulo 7 - Controle Proporcional

Neste módulo vamos aprender uma forma mais elegante de controlar o robô utilizando a ROS 2 implementando controle proporcional. O controle proporcional é uma técnica de controle que utiliza a diferença entre o valor desejado e o valor medido para calcular a ação de controle.

Nesse ponto o seguidor de linha funciona em dois estados, um para seguir a linha e outro para centralizar o robô. O problema é que o movimento do robô é quebrado em pequenos passos. Uma forma mais elegante de controlar o robô é, enquanto o robô anda, ele gira proporcionalmente a distância entre a linha e o centro do robô.

## Atividades
As atividades deste módulo focam em introduzir controle proporcional

Esse conceito será explorado nas seguintes atividades,

- [Atividade 1](atividades/1-controle-proporcional.ipynb) - Introdução ao conceito de controle proporcional.
- [Atividade 2](atividades/2-seguelinha-proporcional.md) - Aplicação do controle proporcional no seguidor de linha.
- [Atividade 3](atividades/3-controlar-angulo.md) - Aplicações do controle proporcional na orientação do robô.
- [Atividade 4](atividades/4-garra.md) - Entendendo a utilização da garra.

## Para entregar

!!! exercise
    Clique no link abaixo para ser direcionado para o Github Classroom da APS 6.

    As APSs são em dupla dentro da mesma turma, no link você deve escolher seu parceiro e/ou criar um grupo.

    As entregas da APS 6 são em vídeo. Siga o tutorial [guia de configuração da APS](https://insper.github.io/robotica-computacional/screen_record/) para saber como fazer a gravação do vídeo no Ubuntu. Feito isso, realize o upload do vídeo no YouTube e coloque o link no arquivo `README.md` do seu repositório.

    [APS 6 - Github Classroom](https://classroom.github.com/a/d-jiiY5d)

    A data final de entrega é **{{ data_APS6 }}**.
