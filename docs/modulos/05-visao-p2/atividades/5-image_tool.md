# Ferramenta de Imagem

Para ajudar na visualização e processamento de imagens, foi criada uma ferramenta que permite a visualização de imagens recebidas pela ROS. Nesta ferraemnta, é possível abrir imagens salvas no computador ou imagens recebidas pela ROS em tempo real.

Na pasta `colcon_ws/src` entre no GitHub abaixo para clonar o repositório da ferramenta de imagem:

[Ferramenta de Imagem](https://github.com/insper-education/robcomp-image-tool)

Depois de clonar o repositório, faça o build do pacote e execute a ferramenta de imagem com o comando:

```bash
ros2 run image_tool start_image_tool
```

A ferramenta de imagem será aberta e você poderá visualizar as imagens recebidas pela ROS 2 e ajustar os limites de cor para filtrar a imagem.

## Image Module

A ferramenta de imagem importa uma classe chamada `ImageModule`, do arquivo `image_module.py`, que faz o processamento da imagem recebida pela ROS. Por padrão, a classe apenas faz a filtragem de cor da imagem com os limites de cor definidos na GUI da ferramenta de imagem.

Na função `run` da classe `ImageModule` você pode adicionar o processamento que desejar para a imagem, como por exemplo, a detecção de contornos e a identificação de objetos. 

Dessa forma você pode testar os limites e depois, você pode levar a classe `ImageModule` para o seu código e utilizá-la para processar as imagens recebidas pela ROS.