from module import ImageModule
import cv2
import numpy as np
from sklearn import linear_model


class Atividade2(): # Essa classe deve herdar da classe ImageModule
    def __init__(self):
        # Inicializar a classe pai

        # Definir aqui os atributos da classe
        self.f = 0
        self.lower = None
        self.upper = None

        # Configure o kernel para operações morfológicas

    @staticmethod
    def plot_regression_xfy(bgr: np.ndarray, y: np.ndarray, m: np.ndarray, h: float, color=tuple) -> np.ndarray:
        """Plota a reta de regressão no gráfico yxf

        Args:
            bgr (np.ndarray): Image em BGR
            y (np.ndarray (N,)): Vetor com as posições y dos centros de massa
            m (np.ndarray): Coeficiente angular da reta de regressão
            h (float): Coeficiente linear da reta de regressão
            color (tuple): Cor da reta

        Returns:
            bgr (np.ndarray): Imagem com a reta de regressão desenhada
        """
        def f(y):
            return m*y + h
        y_min = int(min(y)) # precisa ser int para plotar na imagem
        y_max = int(max(y)) 

        x_min = int(f(y_min))
        x_max = int(f(y_max))    
        cv2.line(bgr, (x_min, y_min), (x_max, y_max), color, thickness=3)
        return bgr

    def regressao_por_centro(self, bgr: np.ndarray, x: np.ndarray, y: np.ndarray) -> (np.ndarray, np.ndarray, float):
        """Regressão linear por centro de massa

        Args:
            bgr (np.ndarray): Image em BGR
            x (np.ndarray (N,)): Vetor com as posições x dos centros de massa
            y (np.ndarray (N,)): Vetor com as posições y dos centros de massa

        Returns:
            bgr (np.ndarray): Imagem com a reta de regressão desenhada
            m (np.ndarray): Coeficiente angular da reta de regressão
            h (float): Coeficiente linear da reta de regressão
            xr (np.ndarray): Vetor com as posições x dos centros de massa ordenados
            yr (np.ndarray): Vetor com as posições y dos centros de massa ordenados
        """
        # Ajuste a entrada para o modelo de regressão
        yr = y.reshape(-1,1)
        xr = x.reshape(-1,)
        m = 0
        h = 0

        # Crie o modelo de regressão linear
        
        # Plotando a reta de regressão

        return bgr, m, h, xr, yr

    def run(self, bgr: np.ndarray) -> np.ndarray:
        """Esta função deve ser processar o frame de entrada chamando as funções necessárias para executar a atividade 5. 
        Crie quantas funções auxiliares achar necessário dentro dessa classe ou dentro da classe ImageModule.

        Se desejar pode retornar uma máscara durante o desenvolvimento para facilitar a visualização

        Args:
            bgr (np.ndarray): Frame de entrada

        Returns:
            bgr (np.ndarray): Frame com a atividade 5 desenhada
            m (np.ndarray): Coeficiente angular da reta de regressão
            h (float): Coeficiente linear da reta de regressão
            xr (np.ndarray): Vetor com as posições x dos centros de massa ordenados
            yr (np.ndarray): Vetor com as posições y dos centros de massa ordenados
        """
        # Esta função deve ser implementada para executar a atividade 5

        return bgr, m, h, xr, yr


def rodar_frame():
    RodaAtividade = Atividade2()
    
    bgr = cv2.imread("img/frame01.png") # Escolha aqui a imagem que deseja usar para testar
    # bgr = cv2.imread("img/frame02.png")
    # bgr = cv2.imread("img/frame03.png")
    # bgr = cv2.imread("img/frame04.png")
    # bgr = cv2.imread("img/cruz01.png")
    # bgr = cv2.imread("img/cruz02.png")

    bgr, m, h, xr, yr = RodaAtividade.run(bgr)

    print("Coeficiente angular [m]: ", m)
    print("Coeficiente linear [h]: ", h)
    print("Posições x dos centros de massa ordenados [xr]: ", xr)
    print("Posições y dos centros de massa ordenados [yr]: ", yr)

    cv2.imshow("Imagem", bgr)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def rodar_video():
    RodaAtividade = Atividade2()

    cap = cv2.VideoCapture('img/pista.mp4') # Confira se o video esta na pasta img

    while(cap.isOpened()):
        ret, bgr = cap.read()

        if ret == True:
            bgr, m, h, xr, yr = RodaAtividade.run(bgr)
            cv2.imshow('Frame', bgr)

            if cv2.waitKey(1) & 0xFF == ord('q'): # !!! Pressione q para sair
                break
            
def main():
    # Selecione se deseja rodar seu codigo com uma imagem ou um video:

    rodar_frame()
    # rodar_video()

if __name__ == "__main__":
    main()