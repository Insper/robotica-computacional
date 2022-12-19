# Lendo o ARUCO

Estimar a posição relativa a um objeto é de grande importância em muitas aplicações de visão computacional, como navegação de robôs, realidade aumentada e muito mais. Este processo se baseia em encontrar correspondências entre pontos no ambiente e sua projeção de imagem 2D. Esta é geralmente uma etapa difícil e, portanto, é comum o uso de marcadores para facilitar.
Uma das abordagens mais populares é o uso de marcadores binários. O principal benefício desses marcadores é que um único marcador fornece correspondências suficientes (seus quatro cantos) para obter a posição da câmera em relação ao marcador. Além disso, a codificação binária interna os torna especialmente robustos, permitindo a possibilidade de aplicar técnicas de detecção e correção de erros.

O módulo aruco é baseado na biblioteca ArUco, uma biblioteca popular para detecção de marcadores de referência desenvolvida por Rafael Muñoz e Sergio Garrido, se quiser saber mais, recomendo a [documentação](https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/aruco_basics.html) que foi usada como base para desenvolver esse guia.

## Aruco 2D

O Aruco 2D é a implementação mais simples utilizando a técnica de marcadores aruco, com ela conseguimos capturar o ID do aruco e os seus “corners” que são as coordenadas dos “cantos” do aruco.

Se estiver usa do o simulador, certifique-se de que o seu arquivo [robotica.sh](http://robotica.sh) está configurado corretamente para se conectar ao robô simulado (Se não sabe do que eu estou falando, volte ao guia robo-simulado).

Se estiver usando o robô físico, configure corretamente o arquivo [robotica.sh](http://robotica.sh) (veja o guia robo-real se tiver dúvidas sobre como se conectar ao robô fisico)

Inicialize o Gazebo com a pista_s2.launch **se estiver usando o robô simulado**

```bash
roslaunch my_simulation pista_s2.launch
```

Execute o código [aruco_turtle_2d.py](aruco_turtle_2d.py) 

![Untitled](imgs/Untitled.png)

Estude o código [aruco_turtle_2d.py](https://www.notion.so/aruco_turtle_2d.py), faça as modificações necessárias para que o robô gire em torno de si mesmo e seja capaz de contar quantos arucos de Id diferentes estão no campo de visão de robô no cenário pista_s2.launch.

## Aruco 3D

Com o aruco 3D além de obter os corners e os IDs dos arucos, também conseguimos calcular o vetor de transalação e o vetor de rotação do aruco em relação ao robô, com essas informações é possível extrair a distância euclidiana e a distância focal do aruco em relação ao robô, essas implementações estão disponíveis para você no código exemplo [aruco_3d.py](aruco_3d.py), para executar:

Certifique-se de que o terminal está aberto na mesma página que está o código, usando o comando “ls” é possível listar o conteúdo do diretório, é preciso que os arquivos abaixo estejam na pasta.

***aruco_3d.py cameraDistortion_realsense.txt cameraMatrix_realsense.tx***

 

```python
ls
```

 

![Untitled](imgs/Untitled1.png)

Execute o arquivo aruco_3d.py

```bash
python3 aruco_3d.py
```

![Untitled](imgs/Untitled2.png)

No seu terminal será exibida a lista com todos os arucos que o robô detectou, e no display da imagem, os valores de distância do primeiro aruco da lista.

Estude o código exemplo, modifique o programa para que:

- As distâncias de todos os arucos encontrados são salvas em uma lista ou em um dicionário, você escolhe.
- O robô se aproxima e se mantém a aproximadamente 1m do primeiro aruco da lista.