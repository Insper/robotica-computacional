#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import math
import numpy as np

from sklearn.linear_model import LinearRegression

def segmenta_linha_amarela(bgr):
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e retornar uma máscara com os segmentos amarelos do centro da pista em branco.
        Utiliza a função cv2.morphologyEx() para limpar ruidos na imagem
    """

    return bgr

def encontrar_contornos(mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca e retornar todos os contornos encontrados
    """
   
    return None

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    x = int(x)
    y = int(y)
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)

def encontrar_centro_dos_contornos(bgr, contornos):
    """Não mude ou renomeie esta função
        deve receber uma lista de contornos e retornar, respectivamente,
        a imagem com uma cruz no centro de cada segmento e o centro de cada. 
        formato: img, x_list, y_list
    """

    img = bgr.copy()
    x_list = []
    y_list = []
    return img, x_list, y_list


def desenhar_linha_entre_pontos(bgr, X, Y, color):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e retornar uma imagem com uma linha entre os centros EM SEQUENCIA do mais proximo.
    """
    img = bgr.copy()

    return img

def regressao_por_centro(bgr, x_array, y_array):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e estimar a melhor reta, utilizando o metodo preferir, que passa pelos centros. Retorne a imagem com a reta e os parametros da reta
        
        Dica: cv2.line(img,ponto1,ponto2,color,2) desenha uma linha que passe entre os pontos, mesmo que ponto1 e ponto2 não pertençam a imagem.
    """
    img = bgr.copy()

    return img, None

def calcular_angulo_com_vertical(img, lm):
    """Não mude ou renomeie esta função
        deve receber uma imagem contendo uma reta, além da reggressão linear e determinar o ângulo da reta com a vertical, utilizando o metodo preferir.
    """

    return None

if __name__ == "__main__":
    print('Este script não deve ser usado diretamente')