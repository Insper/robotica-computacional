import cv2
import numpy as np
import math
from module_net_ import MobileNetDetector

class IdentificarPerigo(MobileNetDetector):
    def __init__(self) -> None:
        super().__init__(CONFIDENCE=0.5)
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
    
    def processar_animais(self, results: list) -> None:
        for r in results:
            if r[0] == 'bird':
                self.animais['passaros'].append(r[2] + r[3])
            elif r[0] == 'dog':
                self.animais['cachorros'].append(r[2] + r[3])
            elif r[0] == 'cat':
                self.animais['gatos'].append(r[2] + r[3])
            
        print(self.animais)

    def distancia(self, a: tuple, b: tuple) -> float:
        def centroide(a: tuple) -> tuple:
            x1,y1,x2,y2 = a
            return ((x1+x2)/2, (y1+y2)/2)
        a = centroide(a)
        b = centroide(b)
        print(math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2))
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def checar_perigos(self, bgr) -> None:
        for passaro in self.animais['passaros']:
            for gato in self.animais['gatos']:
                if self.distancia(passaro, gato) < 300:
                    self.perigo['passaros'].append(passaro)
                    bgr = cv2.putText(bgr, 'PERIGO', (passaro[0], passaro[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        for gato in self.animais['gatos']:
            for cachorro in self.animais['cachorros']:
                if self.distancia(gato, cachorro) < 300:
                    self.perigo['gatos'].append(gato)
                    bgr = cv2.putText(bgr, 'PERIGO', (gato[0], gato[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return bgr

    def run(self, bgr: np.ndarray) -> dict:
        '''A funcao recebe uma imagem bgr e devolve um dicionario com as coordenadas dos animais em perigo.
        Adicione as funcoes auxiliares descritas no enunciado e quantas outras achar necessario'''
        
        perigo = {}
        bgr, results = self.detect(bgr)
        self.processar_animais(results)
        bgr = self.checar_perigos(bgr)

        print(self.perigo)

        return bgr, perigo

if __name__ == '__main__':
    img = cv2.imread("img/teste2.png")
    q3 = IdentificarPerigo()
    bgr, perigo = q3.run(img)


    cv2.imshow("Result", bgr)
    cv2.waitKey(0)

    