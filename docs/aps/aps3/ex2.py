#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np
from module import ImageModule

class ClassificaDominoes(None):# Essa classe deve herdar da classe ImageModule
    def __init__(self):
        # Inicializar a classe pai

        # Configure o kernel para operações morfológicas

        self.lower = None
        self.upper = None


    def run(self, bgr: np.ndarray):
        """Recebe o frame atual em BGR e verifica o valor do domino no frame. 
        Retorna o frame com o valor do domino e o valor do domino em texto.

        Args:
            bgr (np.ndarray): Frame atual em BGR

        Returns:
            bgr: Imagem com o valor do domino
            texto: Valor do domino em texto no formato "x por y"
        """
        texto = f"{None} por {None}"
    
        return bgr, texto


def rodar_video():
    Dominoes = ClassificaDominoes()

    cap = cv2.VideoCapture('img/dominoes.mp4') # Confira se o video esta na pasta img

    while(cap.isOpened()):
        ret, frame = cap.read()

        if ret == True:
            frame = Dominoes.run(frame)

            cv2.imshow('Frame',frame)

            if cv2.waitKey(10) & 0xFF == ord('q'): # !!! Pressione q para sair
                break
        else:
            break
            
def main():
    # Selecione se deseja rodar seu codigo com uma imagem ou um video:
    rodar_video()

if __name__ == "__main__":
    main()