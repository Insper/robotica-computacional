import cv2
import numpy as np


class Atividade1():
    def __init__(self):
        # Definir aqui os atributos da classe
        self.ciano = {
            'lower': (0,0,0),
            'upper': (0,0,0)
        }

        self.magenta = {
            'lower': (0,0,0),
            'upper': (0,0,0)
        }

        self.f = 0 # Distância focal da câmera [px]
        self.H = 0 # Distancia real entre os circulos [cm]

    def encontrar_foco(self, D: float, h: float):
        """Não mude ou renomeie esta função
        Entradas:
        D - distancia real da câmera até o objeto [m]
        h - a distancia virtual entre os circulos [px]
        Saída:
        f - a distância focal da câmera [px]
        """
        f = 0

        return f
    
    def encontrar_D(self, h: float):
        """Não mude ou renomeie esta função
        Entrada:
            f - a distância focal da câmera [px]
            h - a distancia virtual entre os circulos [px]
        Saída:
            D - distancia real da câmera até o objeto [m]
        """
        D = 0
        return D

    def run(self, bgr: np.ndarray):
        """Não mude ou renomeie esta função
        Entrada:
            bgr (np.ndarray): Frame de entrada

        Returns:
            bgr (np.ndarray): Frame com o exercicio 1 desenhado
            D (float): Distancia real da câmera até o objeto [m]
            h (float): a distancia virtual entre os circulos [px]
        """
        # Esta função deve ser implementada para executar o exercicio 1
        D = 0

        return bgr, D


def rodar_frame():
    RodaAtividade = Atividade1()
    
    # bgr = cv2.imread("img/q1/calib01.jpg") # Use esta imagem para calibrar a câmera
    # bgr = cv2.imread("img/q1/teste01.jpg") # Ditancia esperada: ~ 41 cm

    bgr, D = RodaAtividade.run(bgr)
    cv2.imshow("Imagem", bgr)
    cv2.waitKey(0)


def rodar_webcam():
    RodaAtividade = Atividade1()
    cap = cv2.VideoCapture(0)

    while True:
        ret, bgr = cap.read()
        bgr, D = RodaAtividade.run(bgr)

        cv2.imshow("Imagem", bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
def main():
    # Selecione se deseja rodar seu codigo com uma imagem ou um video:

    rodar_frame()
    # rodar_webcam()


if __name__ == "__main__":
    main()