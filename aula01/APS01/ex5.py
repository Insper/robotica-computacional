#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())


def realiza_diferencas(bgr): 
    """
        Abra a imagem `hall_box_battery_atividade5.png`  e a converta para tons de cinza de `0` a `255`.

        Forneça como saída uma uma cópia da imagem  em que cada pixel  recebe o módulo da subtração entre o pixel que vem depois e o que vem antes dele (na horizontal).
        
        Note que nas primeira coluna não existe pixel anterior, e na última coluna não existe posterior.
        Nota também que uma variável uint8 não admite valores '< 0' nem '> 255', então considere fazer conversões de tipos de dados 
    """
    res = bgr.copy()
    return res


if __name__ == "__main__":
    img = cv2.imread("hall_box_battery_atividade5.png")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Faz o processamento
    saida = realiza_diferencas(gray)
    cv2.imwrite("ex5_diferencas.png", saida)


    # NOTE que a OpenCV terminal trabalha com BGR
    cv2.imshow('entrada', img)
    cv2.moveWindow('entrada',200,200)
    cv2.imshow('saida', saida)
    cv2.moveWindow('saida',800,200)

    cv2.waitKey()
    cv2.destroyAllWindows()

