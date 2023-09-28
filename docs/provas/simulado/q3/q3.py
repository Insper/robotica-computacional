import cv2
import numpy as np
import math

class IdentificarPerigo():
    def __init__(self) -> None:
        self.animais = {
            'passaros': [],
            'cachorros': [],
            'gatos': [],
        }
        self.perigo = {
            'passaros': [],
            'cachorros': [],
            'gatos': [],
        }
    
    def run(self,bgr: np.ndarray) -> dict:
        '''A funcao recebe uma imagem bgr e devolve um dicionario com as coordenadas dos animais perigosos
        Adicione as funcoes auxiliares descritas no enunciado e quantas outras achar necessario'''
        
        return bgr, self.perigo

if __name__ == '__main__':
    bgr = cv2.imread("img/teste2.png")
    q3 = IdentificarPerigo()
    bgr, perigo = q3.run(bgr)
    print(perigo)

    cv2.imshow("Resultado", bgr)
    cv2.waitKey(0)