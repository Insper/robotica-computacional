"""
Funções úteis para mostrar o histograma da imagem
"""
import cv2
from matplotlib import pyplot as plt
import numpy as np
import time as t

def plot_hists(img):
    """
        Plota o histograma de cada um dos canais RGB
        img - imagem RGB
    """
    plt.figure(figsize=(20,10)); 
    img_h = img[:,:,0]
    img_s = img[:,:,1]
    img_v = img[:,:,2]
    histo_plot(img_h, "r","R");
    histo_plot(img_s, "g","G");
    histo_plot(img_v, "b","B")

def make_hist(img_255, c1, label, c2=None):
    """ img_255 - uma imagem com 3 canais de 0 até 255
        c1 - a cor do plot do histograma
        label - o label do gráfico do histograma
        c2 (opcional) - a cor do plot da distribuição cumulativa
    """
    hist,bins = np.histogram(img_255.flatten(),256,[0,256])
    cdf = hist.cumsum()
    cdf_normalized = cdf * hist.max()/ cdf.max()

    # plt.plot(cdf_normalized, color = c)
    plt.hist(img_255.flatten(),256,[0,256], color = c1)
    if c2 is not None: plt.plot(cdf_normalized, color = c2)
    plt.xlim([-1,256])
    if c2 is not None: plt.legend(['cdf', label], loc = 'upper left')
    else: plt.legend(label, loc = 'upper left')
    plt.show()

def histo_plot(img, cor, label):
    """
        img - imagem
        cor - cor
        plt - matplotlib.pyplot object

    """
    plt.figure(figsize=(10,5))
    make_hist(img, cor, label)
    plt.show()
    plt.figure(figsize=(10,5))
    plt.imshow(img, cmap="Greys_r")#, vmin=0, vmax=255)    
    plt.title(label)