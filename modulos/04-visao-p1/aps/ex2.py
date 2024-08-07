import cv2
import numpy as np


class Atividade2():
    def __init__(self):
        # Filtro amarelo HSV
        self.lower = (0, 0, 0)
        self.upper = (0, 0, 0)

    def run(self, bgr: np.ndarray) -> np.ndarray:
        """Esta função deve processar o frame de entrada chamando as funções necessárias. 
        Crie quantas funções auxiliares achar necessário dentro dessa classe ou dentro da classe ImageModule.

        Se desejar pode retornar uma máscara durante o desenvolvimento para facilitar a visualização

        Args:
            bgr (np.ndarray): Frame de entrada

        Returns:
            bgr (np.ndarray): Frame com a atividade 5 desenhada
        """
        # Esta função deve ser implementada para executar a atividade 5

        return bgr


def rodar_frame():
    RodaAtividade = Atividade2()
    
    bgr = cv2.imread("img/q2/frame01.png") # Escolha aqui a imagem que deseja usar para testar
    # bgr = cv2.imread("img/q2/frame02.png")
    # bgr = cv2.imread("img/q2/frame03.png")
    # bgr = cv2.imread("img/q2/frame04.png")
    # bgr = cv2.imread("img/q2/cruz01.png")
    # bgr = cv2.imread("img/q2/cruz02.png")

    bgr = RodaAtividade.run(bgr)

    cv2.imshow("Imagem", bgr)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def rodar_video():
    RodaAtividade = Atividade2()

    cap = cv2.VideoCapture('img/q2/pista.mp4') # Confira se o video esta na pasta img/q2

    while(cap.isOpened()):
        ret, bgr = cap.read()

        if ret == True:
            bgr = RodaAtividade.run(bgr)
            cv2.imshow('Frame', bgr)

            if cv2.waitKey(1) & 0xFF == ord('q'): # !!! Pressione q para sair
                break
            
def main():
    # Selecione se deseja rodar seu codigo com uma imagem ou um video:

    rodar_frame()
    # rodar_video()

if __name__ == "__main__":
    main()