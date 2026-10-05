import cv2
import numpy as np

class IdentificadorBandeiras():
    def __init__(self) -> None:
        self.bandeiras = []

    def run(self,bgr):
        '''
        Essa função deverá identificar as bandeiras na imagem passada como argumento
        e devolver uma lista de tuplas no formato

        ('pais', (x1, y1), (x2, y2))
        '''

        return self.bandeiras


if __name__ == '__main__':
    bgr = cv2.imread('img/teste1.png')
    q1 = IdentificadorBandeiras()
    items = q1.run(bgr)

    for it in items:
        print(it)
        cv2.rectangle(bgr, it[1], it[2], (255, 0, 0), 5)
    
    cv2.imshow("Resultado", bgr)
    cv2.waitKey(0)
