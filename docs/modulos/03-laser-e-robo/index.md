# Módulo 3 - Controlando o Robô

!!! pdf
    ![](slides.pdf)


Neste módulo vamos aprender a controlar o robô utilizando a ROS 2. Vamos aprender a controlar o robô utilizando maquina de estados, e a ler os dados do sensor laser e da odometria, e utilizar esses dados para controlar o robô. Também vamos aprender a visualizar os dados dos sensores utilizando o `rqt_image_view` e o `RViz`.

## Atividades
As atividades deste módulo focam em introduzir técnicas úteis para 

1. Entender o conceito básico de maquina de estados.
2. Entender o conceito de odometria.
3. Entender o conceito de sensor laser.
4. Aprender a visualizar os dados dos sensores utilizando o `rqt_image_view` e o `RViz`.

Estes conceitos são explorados nas seguintes atividades,

- [Atividade 1](atividades/1-maquina-de-estados.md) - Introdução ao conceito de maquina de estados.
- [Atividade 2](atividades/2-estrutura-basica.md) - Aqui fornecemos uma estrutura básica um nó qualquer e um nó de controle do robô com maquina de estados.
- [Atividade 3](atividades/3-odometria.md) - Entendendo o conceito de Pose e Odometria. Também vamos aprender a criar um módulo de odometria para ser facilmente importado em outros programas.
- [Atividade 4](atividades/4-sensor-laser.md) - Entendendo a leitura do sensor laser. Também vamos aprender a criar um módulo do sensor laser para ser facilmente importado em outros programas.
- [Atividade 5](atividades/5-visualizacao.md) - Entendendo como visualizar os dados da câmera, utilizando o `rqt_image_view`, e os dados do sensor laser e da odometria, utilizando o `RViz`.

<!-- ## Para entregar

!!! exercise
    Clique no link abaixo para ser direcionado para o Github Classroom da APS 1.

    As APSs são em dupla dentro da mesma turma, no link acima você deve escolher seu parceiro e criar um grupo.

    Uma das entregas da APS 2 é um vídeo, siga o [guia de configuração da APS](../../aps/screen_record.md) para saber como fazer a gravação do vídeo no Ubuntu. Depois, de upload do vídeo no Youtube e coloque o link no arquivo `README.md` do seu repositório.

    [APS 2 - Github Classroom](https://classroom.github.com/a/NBBBoAiG)

    A data final de entrega é **{{ data_APS2 }}**. -->
