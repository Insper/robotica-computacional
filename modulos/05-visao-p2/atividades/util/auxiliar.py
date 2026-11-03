#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
    Atenção: usado no notebook da aula. 
    Não precisa ser usado diretamente
"""

from ipywidgets import widgets, interact, interactive, FloatSlider, IntSlider
import numpy as np
import cv2
import matplotlib.pyplot as plt

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
            row.append(widgets.FloatText(value=0))
        list_elements.append(row)

    rows = []
    for row in list_elements:
        rows.append(widgets.HBox(row))
        
    widgets_mat = widgets.VBox(rows)
        
    return list_elements, widgets_mat

def make_widgets_mat_from_data(data):
    """
        Creates a matriz of int Widgets given 2D-data
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
    hsv[0] = max(0, hsv[0]-10)
    hsv2[0] = min(180, hsv[0]+ 10)
    hsv[1:] = 50
    hsv2[1:] = 255
    return hsv, hsv2 

def plot_filter(img, kernel):
    plt.figure(figsize=(20,10))
    # kernel = aux.make_np_from_widgets_list(widgets)
    out3 = cv2.filter2D(img, -1,  kernel)
    plt.subplot(121)
    plt.axis(False)
    plt.title("Original")
    plt.imshow(img, cmap="gray", vmin=0, vmax=255)

    plt.subplot(122)
    plt.axis(False)
    plt.title("Transformado")
    plt.imshow(out3, cmap="gray", vmin=0, vmax=255)
    plt.show()

def plot_filter_3(img, kernel_1, kernel_2, title_1, title_2):
    plt.figure(figsize=(20,10))
    # kernel = aux.make_np_from_widgets_list(widgets)
    out1 = cv2.filter2D(img, -1,  kernel_1)
    out2 = cv2.filter2D(img, -1,  kernel_2)

    plt.subplot(131)
    plt.axis(False)
    plt.title("Original")
    plt.imshow(img, cmap="gray", vmin=0, vmax=255)

    plt.subplot(132)
    plt.axis(False)
    plt.title(title_1)
    plt.imshow(out1, cmap="gray", vmin=0, vmax=255)

    plt.subplot(133)
    plt.axis(False)
    plt.title(title_2)
    plt.imshow(out2, cmap="gray", vmin=0, vmax=255)
    plt.show()

if __name__ == "__main__":
    print("Este script não deve ser executado diretamente")
