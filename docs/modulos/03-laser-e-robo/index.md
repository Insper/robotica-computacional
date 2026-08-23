# Módulo 3 - Controlando o Robô

!!! pdf
    ![](slides.pdf)


Neste módulo vamos aprender a controlar o robô utilizando a ROS 2. Vamos aprender a controlar o robô utilizando maquina de estados e ações, a ler os dados do sensor laser e da odometria, e utilizar esses dados para controlar o robô. Também vamos aprender a visualizar os dados dos sensores utilizando o `rqt_image_view` e o `RViz`.

## Atividades
As atividades deste módulo focam em introduzir técnicas úteis para 

1. Entender o conceito básico de maquina de estados.
2. Entender o conceito de odometria.
3. Entender o conceito de sensor laser.
4. Entender o conceito de ações e **como implementá-las**.
5. Aprender a visualizar os dados dos sensores utilizando o `rqt_image_view` e o `RViz`.

Estes conceitos são explorados nas seguintes atividades,

- [Atividade 1](atividades/1-maquina-de-estados.md) - Introdução ao conceito de maquina de estados.
- [Atividade 2](atividades/2-estrutura-basica.md) - Aqui fornecemos uma estrutura básica composta por um nó qualquer e um nó de controle do robô por meio de maquina de estados.
- [Atividade 3](atividades/3-odometria.md) - Entendendo o conceito de `Pose` e `Odometria`. Também vamos aprender a criar um módulo de odometria para ser facilmente importado em outros programas.
- [Atividade 4](atividades/4-laser.md) - Entendendo a leitura do sensor laser. Também vamos aprender a criar um módulo do sensor laser para ser facilmente importado em outros programas.
- [Atividade 5](atividades/5-acao-andar.md) - Entendendo como implementar a ação de andar.
- [Atividade 6](atividades/6-visualizacao.md) - Entendendo como visualizar os dados da câmera, utilizando o `rqt_image_view`, e os dados do sensor laser e da odometria, utilizando o `RViz`.

<!-- ## Para entregar

!!! exercise
    Clique no link abaixo para ser direcionado para o Github Classroom da APS 3.

    As APSs são em dupla dentro da mesma turma, no link você deve escolher seu parceiro e/ou criar um grupo.

    As entregas da APS 3 são em vídeo. Siga o tutorial [guia de configuração da APS](https://insper.github.io/robotica-computacional/screen_record/) para saber como fazer a gravação do vídeo no Ubuntu. Feito isso, realize o upload do vídeo no YouTube e coloque o link no arquivo `README.md` do seu repositório.

    [APS 3 - Github Classroom]( {{ link_APS3 }} )

    A data final de entrega é **{{ data_APS3 }}**. -->
