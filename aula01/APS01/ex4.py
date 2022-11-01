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


def antartida(bgr): 
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e devolver o recorte da imagem da antartida
    """
    res = bgr.copy()
    return res

def canada(bgr): 
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e devolver o recorte da imagem do canadá
    """
    res = bgr.copy()
    return res


if __name__ == "__main__":
    img = cv2.imread("ant_canada_250_160.png")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Faz o processamento
    antartida = antartida(img)
    canada = canada(img)
    cv2.imwrite( "ex4_antartida_resp.png", antartida)
    cv2.imwrite("ex5_canada_resp.png", canada)    


    # NOTE que a OpenCV terminal trabalha com BGR
    cv2.imshow('entrada', img)

    cv2.imshow('antartida', antartida)
    cv2.imshow('canada', canada)

    cv2.waitKey()
    cv2.destroyAllWindows()


