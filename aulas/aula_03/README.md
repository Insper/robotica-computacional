# Atividade 3 de visão

## Orientações sobre integridade intelectual

<font color='red'>ATENÇÃO: </font> tenham sempre em mente as [orientações do Insper sobre integridade intelectual em atividades de programação](https://tinyurl.com/comp-insper-atividades).

## Verificação prévia

Rode no prompt Python:

```
import cv2
cv2.__version__
```

E veja a versão da OpenCV. Deverá ser maior que 3.

## Preparação

Leia os seguintes exemplos:

- Exemplo de transformada de Hough para deteção de círculos [draw_circles_video.py](draw_circles_video.py)
- Exemplo de deteção de features e encontro de correspondências: [py_feature_homography.py](py_feature_homography.py)
- Exemplo de transformada de Hough para deteção de linhas (não vai ser usado): [houghlines.py](houghlines.py)

## O que deve ser feito

Pegue uma cópia [deste arquivo PDF](padrao_rastrear.pdf) com o professor.  

![Padrão a ser rastreado](madfox.jpg)

Você deve fazer um programa que lê a imagem da webcam e combina o detector de features e o detector de círculos

Seu programa deve imprimir se a folha foi detectada ou não


Desafios:

* Verificar se o círculo está na frente da raposa
* Verificar se tanto círculo quanto raposa estão sobre uma região de papel branco
