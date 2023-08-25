from module import ImageModule
import cv2
import numpy as np

class Atividade5(): # Essa classe deve herdar da classe ImageModule
    def __init__(self):
        pass # Podem apagar o pass depois de implementar a classe

        # Inicializar a classe pai

        # Definir aqui os atributos da classe
    
    def run(self, bgr: np.ndarray) -> np.ndarray:
        """Esta função deve ser processar o frame de entrada chamando as funções necessárias para executar a atividade 5. 
        Crie quantas funções auxiliares achar necessário dentro dessa classe ou dentro da classe ImageModule.

        Se desejar pode retornar uma máscara durante o desenvolvimento para facilitar a visualização

        Args:
            bgr (np.ndarray): Frame de entrada

        Returns:
            bgr (np.ndarray): Frame com a atividade 5 desenhada
        """
        # Esta função deve ser implementada para executar a atividade 5
        pass

def rodar_frame():
    RodaAtividade = Atividade5()
    
    bgr = cv2.imread("img/frame01.jpg") # Escolha aqui a imagem que deseja usar para testar
    # bgr = cv2.imread("img/frame02.jpg")
    # bgr = cv2.imread("img/frame03.jpg")

    img = RodaAtividade.run(bgr)

    cv2.imshow("Imagem", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def rodar_video():
    RodaAtividade = Atividade5()

    cap = cv2.VideoCapture('img/pista_simulado.mp4') # Confira se o video esta na pasta img

    while(cap.isOpened()):
        ret, frame = cap.read()

        if ret == True:
            brg = RodaAtividade.run(frame)

            cv2.imshow('Frame',brg)

            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
            

if __name__ == "__main__":
    # Selecione se deseja rodar seu codigo com uma imagem ou um video:

    rodar_frame()
    # rodar_video()
