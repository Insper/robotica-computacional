import cv2
import numpy as np
import math
import os
from module_net import MobileNetDetector

class DogTracker(MobileNetDetector):
    def __init__(self):
        """Inicializa os atributos e configurações do rastreador de cachorros."""

        super().__init__(CONFIDENCE = 0.5,
                         args_prototxt = "./config/MobileNetSSD_deploy.prototxt.txt",
                         args_model = "./config/MobileNetSSD_deploy.caffemodel")
        
        self.tracking = {
            'dogA': [], # [(xmin, ymin, xmax, ymax)]
            'dogB': [],
        }
        self.last_center = {
            'dogA': (0, 0),
            'dogB': (0, 0),
        }

        self.colors = {
            'dogA': (0, 0, 255),  # Red for dogA
            'dogB': (255, 0, 0),  # Blue for dogB
        }

        self.threshold_distance = 100
    
    def bbox_center(self, bbox: tuple) -> tuple:
        """Não mude ou renomeie esta função
        Calcula o centro de um bounding box.
        
        Args:
            bbox (tuple): Uma tupla contendo as coordenadas (x1, y1, x2, y2) do bounding box.
        
        Returns:
            center (tuple): As coordenadas (x, y) do centro do bounding box.
        """
        center = None
        return center

    def bbox_distance(self, center1: tuple, center2: tuple) -> float:
        """Não mude ou renomeie esta função
        Calcula a distância entre dois centros de bounding boxes.
        
        Args:
            center1, center2 (tuple): As coordenadas (x, y) dos centros dos bounding boxes.
        
        Returns:
            distance (float): A distância entre os dois centros.
        """
        distance = None
        return distance

    def plot_last_detections(self, bgr: np.ndarray) -> np.ndarray:
        """Não mude ou renomeie esta função
        Desenha as últimas 3 detecções de cada cachorro na imagem.
        
        Args:
            bgr (numpy.ndarray): Imagem de entrada em BGR.
        
        Returns:
            numpy.ndarray: A imagem com as detecções desenhadas.
        """
        for dog, detections in self.tracking.items():
            # Take last 3 detections for this dog
            lenght = len(detections[-3:])
            for i, bbox in enumerate(detections[-3:]):
                # Draw a rectangle for each detection
                cv2.rectangle(bgr, bbox[:2], bbox[2:], self.colors[dog], 2)
                # Connect the detections with a line
                if i > 0:  # If it's not the first detection, connect it to the previous
                    prev_center = np.array(self.bbox_center(detections[i - lenght - 1])).astype(int)
                    current_center = np.array(self.bbox_center(bbox)).astype(int)
                    print(prev_center, current_center)
                    cv2.line(bgr, prev_center, current_center, self.colors[dog], 2)
        
        return bgr

    def first_detection(self, dogs: list):
        """Não mude ou renomeie esta função
        Trata os casos onde um ou ambos os cachorros ainda não foram detectados.
        
        Args:
            dogs (list): Lista de detecções feitas pela rede neural.
        """
        for dog in dogs:
            if len(self.tracking['dogA']) == 0:
                None
            elif len(self.tracking['dogB']) == 0:
                center = None # Calcula o center do dog atual.
                # (if)  Verifica se o dog atual está perto do dogA.
                # (else:) # Se não, atualiza o dogB.
                

    def update_2dogs(self, dogs: list, key: str):
        """Não mude ou renomeie esta função
        Atualiza o rastreamento quando dois cachorros são detectados no frame.
        
        Args:
            dogs (list): Lista de detecções feitas pela rede neural.
            key (str): A chave ('dogA' ou 'dogB') que indica qual cachorro está sendo atualizado.
        """
        if len(dogs) == 2:
            pass# Então atualiza o cachorro na variável 'key'
    def update_tracking(self, dogs: list):
        """Não mude ou renomeie esta função
        Atualiza o rastreamento dos cachorros com base nas detecções.
        
        Args:
            dogs (list): Lista de detecções feitas pela rede neural.
        """
        # Segue o Caso 3, encontrando o centro do dogs[0] e verificando se ele está perto do dogA ou do dogB.
        # Depois chama a função update_2dogs para atualizar o rastreamento no caso de dois cachorros detectados no frame.


    def run(self, bgr: np.ndarray) -> np.ndarray:
        """Executa a detecção e o rastreamento dos cachorros em um frame da imagem.
        
        Args:
            bgr (numpy.ndarray): A imagem de entrada em BGR.
        
        Returns:
            numpy.ndarray: A imagem processada com as detecções e rastreamentos.
        """
        bgr, out = self.detect(bgr)
        dogs = [det for det in out if det[0] != 'tvmonitor']
        if len(dogs) == 0:
            return bgr

        if len(self.tracking['dogA']) == 0 or len(self.tracking['dogB']) == 0:
            self.first_detection(dogs)
        else:
            self.update_tracking(dogs)

        bgr = self.plot_last_detections(bgr)

        return bgr


def rodar_video():
    Tracker = DogTracker()

    cap = cv2.VideoCapture('img/dogs.wmv') # Confira se o video esta na pasta img

    while(cap.isOpened()):
        ret, frame = cap.read()

        if ret == True:
            frame = Tracker.run(frame)

            cv2.imshow('Frame',frame)

            if cv2.waitKey(10) & 0xFF == ord('q'): # !!! Pressione q para sair
                break
        else:
            break
            
def main():
    # Selecione se deseja rodar seu codigo com uma imagem ou um video:

    # rodar_frame()
    rodar_video()

if __name__ == "__main__":
    main()