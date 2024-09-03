# Ferramenta de Imagem

Para ajudar na visualização e processamento de imagens, foi criada uma ferramenta que permite a visualização de imagens recebidas pela ROS. Nesta ferraemnta, é possível abrir imagens salvas no computador ou imagens recebidas pela ROS em tempo real.

Na pasta `colcon_ws/src` entre no GitHub abaixo para clonar o repositório da ferramenta de imagem:

[Ferramenta de Imagem](https://github.com/insper-education/robcomp-image-tool)

Depois de clonar o repositório, faça o build do pacote e execute a ferramenta de imagem com o comando:

```bash
ros2 run image_tool start_image_tool
```

A ferramenta de imagem será aberta e você poderá visualizar as imagens recebidas pela ROS 2 e ajustar os limites de cor para filtrar a imagem.