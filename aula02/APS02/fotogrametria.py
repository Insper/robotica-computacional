#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np
import math

def encontrar_foco(D,H,h):
    """Não mude ou renomeie esta função
    Entradas:
       D - distancia real da câmera até o objeto (papel)
       H - a distancia real entre os circulos (no papel)
       h - a distancia na imagem entre os circulos
    Saída:
       f - a distância focal da câmera
    """
    f = h / H * D

    return f

def segmenta_circulo_ciano(hsv): 
    """Não mude ou renomeie esta função
    Entrada:
        hsv - imagem em hsv
    Saída:
        mask - imagem em grayscale com tudo em preto e os pixels do circulos ciano em branco
    """
    mask = cv2.inRange(hsv,(150/2, 100, 100),(210/2, 255, 255))
    
    return mask

def segmenta_circulo_magenta(hsv):
    """Não mude ou renomeie esta função
    Entrada:
        hsv - imagem em hsv
    Saída:
        mask - imagem em grayscale com tudo em preto e os pixels do circulos magenta em branco
    """
    mask = cv2.inRange(hsv,(270/2, 50, 50),(330/2, 255, 255))
    
    return mask
def encontrar_maior_contorno(segmentado):
    """Não mude ou renomeie esta função
    Entrada:
        segmentado - imagem em preto e branco
    Saída:
        contorno - maior contorno obtido (APENAS este contorno)
    """
    
    contours, hierarchy = cv2.findContours(segmentado, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    maxarea = 0
    the_contour = None
    for contour in contours:
        if abs(cv2.contourArea(contour) > maxarea):
            the_contour = contour
    
    return the_contour

def encontrar_centro_contorno(contorno):
    """Não mude ou renomeie esta função
    Entrada:
        contorno: um contorno (não o array deles)
    Saída:
        (Xcentro, Ycentro) - uma tuple com o centro do contorno (no formato 'int')!!! 
    """  

    Xcentro = int(contorno[:,:,0].mean())
    Ycentro = int(contorno[:,:,1].mean())
    
    return (Xcentro, Ycentro)

def calcular_h(centro_ciano, centro_magenta):
    """Não mude ou renomeie esta função
    Entradas:
        centro_ciano - ponto no formato (X,Y)
        centro_magenta - ponto no formato (X,Y)
    Saída:
        distancia - a distancia Euclidiana entre os pontos de entrada 
    """
    
    distancia = ((centro_magenta[0]-centro_ciano[0])**2 + (centro_magenta[1]-centro_ciano[1])**2)**.5
    return distancia

def encontrar_distancia(f,H,h):
    """Não mude ou renomeie esta função
    Entrada:
        f - a distância focal da câmera
        H - A distância real entre os pontos no papel
        h - a distância entre os pontos na imagem
    Saída:
        D - a distância do papel até câmera
    """
    D = H / h *f
    return D

def calcular_distancia_entre_circulos(img):
    """Não mude ou renomeie esta função
    Deve utilizar as funções acima para calcular a distancia entre os circulos a partir da imagem BGR
    Entradas:
        img - uma imagem no formato BGR
    Saídas:
        h - a distância entre os os circulos na imagem
        centro ciano - o centro do círculo ciano no formato (X,Y)
        centro_magenta - o centro do círculo magenta no formato (X,Y)
        img_contornos - a imagem com os contornos desenhados
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_ciano = segmenta_circulo_ciano(hsv)
    mask_magenta = segmenta_circulo_magenta(hsv)
    
    contorno_ciano = encontrar_maior_contorno(mask_ciano)
    contorno_magenta = encontrar_maior_contorno(mask_magenta)
    
    img_contornos = img.copy()
    cv2.drawContours(img_contornos,[contorno_ciano, contorno_magenta],contourIdx=-1, color=(0,255,0))
    
    centro_magenta = encontrar_centro_contorno(contorno_magenta)
    centro_ciano = encontrar_centro_contorno(contorno_ciano)
    
    h = calcular_h(centro_ciano, centro_magenta)

    return h, centro_ciano, centro_magenta, img_contornos

def calcular_angulo_com_horizontal_da_imagem(centro_ciano, centro_magenta):
    """Não mude ou renomeie esta função
        Deve calcular o angulo, em graus, entre o vetor formato com os centros do circulos e a horizontal.
    Entradas:
        centro_ciano - centro do círculo ciano no formato (X,Y)
        centro_magenta - centro do círculo magenta no formato (X,Y)
    Saídas:
        angulo - o ângulo entre os pontos em graus
    """
    angulo = math.atan2(centro_ciano[1] - centro_magenta[1], centro_ciano[0] - centro_magenta[0])
    angulo = math.degrees(angulo)
 
    return angulo
