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


def antartida(bgr: np.ndarray) -> np.ndarray:
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e devolver o recorte da imagem da antartida
    """
    res = bgr.copy()
    return res

def canada(bgr: np.ndarray) -> np.ndarray:
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e devolver o recorte da imagem do canadá
    """
    res = bgr.copy()
    return res

def run(bgr: np.ndarray) -> None:
    """Não mude ou renomeie esta função
        deve chamar as funções antartida e canada, salvar os resultados em arquivo e mostrar na tela em janelas separadas.
    """
    pass  


if __name__ == "__main__":
    bgr = cv2.imread("img/ex4.png")
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    run(bgr)


