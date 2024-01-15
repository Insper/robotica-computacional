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


def substitui_x_por_cinza(gray: np.ndarray) -> np.ndarray:
    """
        Localiza todas as posições em que há um X 3x3 na imagem de entrada e pinta estas posições com tom de cinza 127. 
        Na saída deve ficar um quadrado cinza onde havia X 
    """
    res = gray.copy()
    return res

if __name__ == "__main__":
    bgr = cv2.imread("img/ex6.png")
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # Faz o processamento
    saida = substitui_x_por_cinza(gray)
    cv2.imwrite("ex6_troca_x.png", saida)

    # Rescale para melhorar a visualização
    gray = cv2.resize(gray, None, fx=10, fy=10, interpolation=cv2.INTER_NEAREST)
    saida = cv2.resize(saida, None, fx=10, fy=10, interpolation=cv2.INTER_NEAREST)
    cv2.imshow('entrada', gray)
    cv2.moveWindow('entrada',200,200)
    cv2.imshow('saida', saida)
    cv2.moveWindow('saida',800,200)

    cv2.waitKey()
    cv2.destroyAllWindows()

