# Atividade 3 de visão

## Verificação prévia

Mude para seu [ambiente de Python 2](https://github.com/Insper/robot18/blob/master/aula_01/opencv_anaconda.md)

Rode no prompt Python:


```
import cv2
cv2.__version__

```

E veja a versão da OpenCV. Deverá ser maior que 3.

## Preparação

Leia os seguintes exemplos:

Exemplo de transformada de Hough para deteção de círculos
[draw_circles_video.py](draw_circles_video.py)

Exemplo de deteção de *features* e encontro de correspondências:
[py_feature_homography.py](py_feature_homography.py)

Exemplo de transformada de Hough para deteção de linhas (não vai ser usado):
[houghlines.py](houghlines.py)


## O que deve ser feito

Pegue uma cópia [deste arquivo PDF](padrao_rastrear.pdf) com o professor.  

![Padrão a ser rastreado](padrao_rastrear.png)

Você deve fazer um programa que lê a imagem da webcam e combina o detector de features e o detector de círculos

Seu programa deve imprimir se a folha foi detectada ou não


Desafios:

* Verificar se o círculo está na frente da raposa

* Verificar se tanto círculo quanto raposa estão sobre uma região de papel branco

