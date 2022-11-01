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
        deve receber respectivamente a distancia real, o a distancia real entre os circulos e a distancia na image entre os circulos e deve retornar o foco
    """

    return None

def segmenta_circulo_ciano(hsv): 
    """Não mude ou renomeie esta função
        deve receber uma imagem em hsv e devolver uma nova imagem com tudo em preto e os pixels do circulos ciano em branco
    """

    return None

def segmenta_circulo_magenta(hsv):
    """Não mude ou renomeie esta função
        deve receber uma imagem em hsv e devolver uma nova imagem com tudo em preto e os pixels do circulos magenta em branco
    """

    return None

def encontrar_maior_contorno(segmentado):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca e retornar APENAS o maior contorno obtido
    """
    
    return None

def encontrar_centro_contorno(contornos):
    """Não mude ou renomeie esta função
        deve receber um contorno e retornar o centro dele, formato: (XX, YY)
    """  

    return (None, None)

def calcular_h(centro_ciano, centro_magenta):
    """Não mude ou renomeie esta função
        deve receber dois pontos e retornar a distancia absoluta entre eles
    """
    
    return None

def encontrar_distancia(f,H,h):
    """Não mude ou renomeie esta função
        deve receber respectivamente o foco a distancia real entre os circulos e a distancia na image entre os circulos e retornar a distancia real
    """

    return None

def calcular_distancia_entre_circulos(img):
    """Não mude ou renomeie esta função
        deve utilizar as funções acima para retornar a disntacia entre os circulos, o centro_ciano, o centro_magenta e a imagem com os contornos desenhados
    """

    return None, None, None, None

def calcular_angulo_com_horinzontal_da_imagem(centro_ciano, centro_magenta):
    """Não mude ou renomeie esta função
        deve calcular o angulo, em radiano, entre o vetor formato com os centros do circulos e a horizontal.
        Retornar o angulo em graos
    """

    return None
