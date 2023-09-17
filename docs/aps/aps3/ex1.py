from module import ImageModule
import cv2
import numpy as np


class LinhaBranca(ImageModule): # Essa classe deve herdar da classe ImageModule
    def __init__(self):
        pass # Podem apagar o pass depois de implementar a classe

        # Inicializar a classe pai

        # Configure o kernel para operações morfológicas

        # Definir aqui os atributos da classe
        self.lower = None
        self.upper = None

    def estimar_linha_nas_faixas(self, bgr: np.ndarray) -> np.ndarray:
        """Esta função deve receber uma imagem, converte-la para preta e branca e retornar dois pontos que formen APENAS uma linha em cada faixa.
        Desenhe cada uma dessas linhas na imagem.
        
        Args:
            gray (np.ndarray): Imagem de entrada em BGR

        Returns:
            bgr (np.ndarra)y: Imagem de saída em BGR com uma linha em cada faixa
            lines (list): formato: [(x1, y1, x2, y2), (x1, y1, x2, y2)]
        """
        # Segmenta a linha branca

        # Usa o o filtro Canny para detectar as bordas
        # Usa HouhLines ou HouhLinesP para detectar as linhas

        # Filtra as linhas obtendo uma na faixa da esquerda e outra na faixa da direita
        lines = None#[esquerda, direita]

        # Desenhar as linhas na imagem

        return bgr, lines
    
    def calcular_ponto_de_fuga(self,lines):
        """Esta função deve receber duas linhas e retornar o ponto de fuga (encontro) entre elas.

        Args:
            lines (list): formato: [(x1, y1, x2, y2), (x1, y1, x2, y2)]

        Returns:
            ponto (tuple): Pontos de fuga (x, y)
        """
        ponto = None
        return ponto


    def run(self, bgr: np.ndarray):
        """Esta função deve ser processar o frame de entrada chamando as funções necessárias para executar a atividade 5. 
        Crie quantas funções auxiliares achar necessário dentro dessa classe ou dentro da classe ImageModule.

        Se desejar pode retornar uma máscara durante o desenvolvimento para facilitar a visualização

        Args:
            bgr (np.ndarray): Frame de entrada

        Returns:
            bgr (np.ndarray): Frame com a atividade 5 desenhada
            ponto (tuple): Pontos de fuga (x, y)
        """
        # Esta função deve ser implementada para executar a atividade 5
        
        # Calcula o ponto de fuga
        ponto = None

        # Desenha o ponto de fuga na imagem
        return bgr, ponto

def rodar_frame():
    RodaAtividade = LinhaBranca()
    
    bgr = cv2.imread("img/frame01.jpg") # Escolha aqui a imagem que deseja usar para testar

    img, point = RodaAtividade.run(bgr)

    cv2.imshow("Imagem", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    rodar_frame()

if __name__ == "__main__":
    main()
