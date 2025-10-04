# Módulo 6 - YOLO e AprilTag

!!! pdf
    ![](slides.pdf)

Neste módulo vamos continuar nosso estudo de processamento de imagens. Primeiro vamos aprender a nos inscrever em tópicos da ROS 2 para receber e processar imagens da câmera do robô. Em seguida, vamos aprender a utilizar a [YOLO](https://arxiv.org/abs/1506.02640) (You Only Look Once), uma rede neural pré-treinada, para identificar objetos em imagens. Vamos também aprender a utilizar marcadores fiduciais do tipo [AprilTag](https://ieeexplore.ieee.org/document/5979561) para identificar e estimar a posição e orientação de objetos em relação à câmera.

## Atividades

As atividades deste módulo focam em melhorar nossas habilidades de identificação de objetos, utilizando a YOLO, e de estimativa de pose, utilizando marcadores fiduciais do tipo AprilTag.

- [1 - Processando Imagens na ROS 2](atividades/1-image_subscriber.md) - Assinatura de imagens da câmera do robô e processamento de imagens na ROS 2.
- [2 - Detecção de Objetos Complexos com Redes Neurais](atividades/2-id_com_NN.md) - Utilização da YOLO para identificação de objetos em imagens no OpenCV.
- [3 - Marcadores Fiduciais](atividades/3-reconhecimento-marcadores.md) - Conceitos de Pose e estimativa de Pose com marcadores feduciais do tipo AprilTag.


## Para entregar

!!! exercise
    Clique no link abaixo para ser direcionado para o Github Classroom da APS 6.

    As APSs são em dupla dentro da mesma turma, no link você deve escolher seu parceiro e/ou criar um grupo.

    As entregas da APS 6 são em vídeo. Siga o tutorial [guia de configuração da APS](https://insper.github.io/robotica-computacional/screen_record/) para saber como fazer a gravação do vídeo no Ubuntu. Feito isso, realize o upload do vídeo no YouTube e coloque o link no arquivo `README.md` do seu repositório.

    [APS 6 - Github Classroom]( {{ link_APS6 }} )

    A data final de entrega é **{{ data_APS6 }}**.