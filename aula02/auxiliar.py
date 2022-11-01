#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
    Atenção: usado no notebook da aula. 
    Não precisa ser usado diretamente
"""

print("Este script não deve ser executado diretamente")

from ipywidgets import widgets, interact, interactive, FloatSlider, IntSlider
import numpy as np
import cv2
import matplotlib.pyplot as plt

import os
print("Trabalhando em ", os.getcwd())


def make_widgets_mat(m, n):
    """
        Makes a m rows x n columns 
        matriz of  integer Jupyter Widgets
        all values initialized to zero
    """
    list_elements = []
    for i in range(m):
        row = []
        for j in range(n):
            row.append(widgets.IntText(value=0))
        list_elements.append(row)

    rows = []
    for row in list_elements:
        rows.append(widgets.HBox(row))
        
    widgets_mat = widgets.VBox(rows)
        
    return list_elements, widgets_mat

def make_widgets_mat_from_data(data):
    """
        Creates a matrix of int Widgets given 2D-data
    """
    n = len(data)
    m = len(data[0])
    elements, mat = makeMat(n, m)
    for i in range(n):
        for j in range(m):
            elements[i][j].value = data[i][j]
    return elements, mat
            
def make_np_from_widgets_list(widgets_list):
    """
        Takes as input a list of lists of widgets and initializes a matrix
    """
    widgets = widgets_list
    n = len(widgets)
    m = len(widgets[0])
    array = np.zeros((n,m), dtype=np.float32)
    for i in range(n):
        for j in range(m):
            array[i][j] = widgets[i][j].value
    return array

def convert_to_tuple(html_color):
    colors = html_color.split("#")[1]
    r = int(colors[0:2],16)
    g = int(colors[2:4],16)
    b = int(colors[4:],16)
    return (r,g,b)

def to_1px(tpl):
    img = np.zeros((1,1,3), dtype=np.uint8)
    img[0,0,0] = tpl[0]
    img[0,0,1] = tpl[1]
    img[0,0,2] = tpl[2]
    return img

def to_hsv(html_color):
    tupla = convert_to_tuple(html_color)
    hsv = cv2.cvtColor(to_1px(tupla), cv2.COLOR_RGB2HSV)
    return hsv[0][0]

def ranges(value):
    hsv = to_hsv(value)
    hsv2 = np.copy(hsv)
    hsv[0] = max(0, hsv[0] - 25)
    hsv2[0] = min(180, hsv[0] + 40)
    hsv[1:] = 60
    hsv2[1:] = 255
    return hsv, hsv2 

def center_of_mass(data):
    """ 
        Retorna uma tupla (cx, cy) que desenha o
        centro de data, que pode ser contorno ou matriz
    """
    M = cv2.moments(data)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return (int(cX), int(cY))

def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

font = cv2.FONT_HERSHEY_SIMPLEX
def texto(img, a, p, color=(0, 255, 255), font=font, width=2, size=1):
    """
        Escreve na img RGB
        dada a string a na posição
        definida pela tupla p
    """
    cv2.putText(img, str(a), p, font,size,color,width,cv2.LINE_AA)

# Analises dos canais:
def hsv_hists(img, plt):
    """
        Plota o histograma de cada um dos canais HSV
        img - imagem HSV
        plt - objeto matplotlib
    """
    plt.figure(figsize=(20,10));
    img_h = img[:,:,0]
    img_s = img[:,:,1]
    img_v = img[:,:,2]
    histo_plot(img_h, "r","H", plt);
    histo_plot(img_s, "g","S", plt);
    histo_plot(img_v, "b","V", plt);

def make_hist(img_255, c, label, plt):
    """ img_255 - uma imagem com 3 canais de 0 até 255
        c a cor do plot
        label - o label do gráfico
        plt - matplotlib.pyplot
    """
    hist,bins = np.histogram(img_255.flatten(),256,[0,256])
    cdf = hist.cumsum()
    cdf_normalized = cdf * hist.max()/ cdf.max()

    # plt.plot(cdf_normalized, color = c)
    plt.hist(img_255.flatten(),256,[0,256], color = c)
    plt.xlim([0,256])
    plt.legend(label, loc = 'upper left')
    plt.plot()

def histo_plot(img, cor, label, plt):
    """
        img - imagem
        cor - cor
        plt - matplotlib.pyplot object
    """
    plt.figure(figsize=(10,5))
    make_hist(img, cor, label, plt)
    plt.show()
    plt.figure(figsize=(10,5))
    plt.imshow(img, cmap="Greys_r")#, vmin=0, vmax=255)
    plt.title(label)

