import cv2
import matplotlib.pyplot as plt

from numpy.linalg import norm
import numpy as np
import math

def mostra_imagem(img, title=None, ticks=False, subfig=False):
    '''
    Função para mostrar a imagem colorida ou em tons de cinza,
    removendo as escalas
    '''
    if len(img.shape) < 3:
        plt.imshow(img, cmap='gray')
    else:
        plt.imshow(img[:,:,::-1])
    
    if not ticks:
        plt.yticks([])
        plt.xticks([])
    
    if title is not None: plt.title(title)
    if not subfig:
        plt.show()

def acumulador_hough_retas(image, rho, theta):
    '''
    Gera a imagem do acumulador da transformada de Hough para retas
    '''
    max_rho = int( norm(image.shape) )
    min_rho = -max_rho
    max_theta = np.pi
    min_theta = 0 

    rows = int((max_rho-min_rho)/rho)
    cols = int((max_theta-min_theta)/theta)
    votes = np.zeros((rows, cols), dtype=int)
    
    # Invoca a transformada de Hough
    for v in range(100):
        lines = cv2.HoughLines(image, rho, theta, v)
        if lines is not None:
            for line in lines:
                r, t = line[0]
                votes[int((r-min_rho)/rho), int((t-min_theta)/theta)] += 1
    
    return votes

def desenha_retas(image, lines):
    '''
    Desenha as retas encontradas pela transformada de Hough
    '''
    if len(image.shape) < 3:
        imout = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        imout = image.copy()

    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(imout, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)
    
    return imout

def desenha_circulos(image, circles):
    '''
    Desenha as circunferências encontradas pela transformada de Hough
    As circunferências são preenchidas (círculos)
    '''
    if len(image.shape) < 3:
        imout = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        imout = image.copy()

    if circles is not None:
        for circle in circles[0]:
            cv2.circle(imout, (int(circle[0]), int(circle[1])), int(circle[2]), (0,0,255), thickness=-1 )
    
    return imout

def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

def non_max_suppression(lines, rho_interval=40, theta_interval=np.radians(20)):
    good_lines = []
    for line in lines:
        candidate = True
        rho, theta= line[0]
            
        for good_line in good_lines:
            rho2, theta2 = good_line[0]
            if abs(abs(rho2)-abs(rho)) < rho_interval and min(abs(theta-theta2),abs(abs(theta-theta2)-math.pi) ) < theta_interval:
                candidate = False
                break
        if candidate:
            good_lines.append([[rho, theta]])

    return good_lines

