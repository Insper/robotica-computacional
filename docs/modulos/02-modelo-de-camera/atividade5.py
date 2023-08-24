from module import ImageModule
import cv2
import numpy as np

class Atividade5(ImageModule): # TODO adicionar comentario para criar Herança
    def __init__(self):
        super().__init__()
        # Definir aqui os atributos da classe
    
    def run(self):
        # Esta função deve ser implementada para executar a atividade 5
        pass

def rodar_frame():
    RodaAtividade = Atividade5()
    
    bgr = cv2.imread("img/frame01.jpg")
    # bgr = cv2.imread("img/frame02.jpg")
    # bgr = cv2.imread("img/frame03.jpg")

    brg = RodaAtividade.run(bgr)

    cv2.imshow("Imagem", bgr)
    # cv2.imshow("Mascara", mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def rodar_video():
    RodaAtividade = Atividade5()

    cap = cv2.VideoCapture('img/video.mp4')

    while(cap.isOpened()):
        ret, frame = cap.read()

        if ret == True:
            brg = RodaAtividade.run(frame)

            cv2.imshow('Frame',brg)

            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
            

if __name__ == "__main__":
    # Selecione se deseja rodar seu codigo com a imagem ou um video:

    rodar_frame()
    # rodar_video()
