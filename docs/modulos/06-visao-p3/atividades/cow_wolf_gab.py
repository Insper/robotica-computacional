import cv2
import numpy as np
import argparse
from module_net import MobileNetDetector

class DangerDetector(MobileNetDetector):
    def __init__(self, CONFIDENCE=0.7):
        super().__init__()
        # Ajuste o valor de confiança para o valor que você achar melhor
        self.CONFIDENCE = CONFIDENCE
        
    def separar_caixa_entre_animais(self, img: np.ndarray, resultados: list):
        """Não mude ou renomeie esta função
        Combina as caixas dos lobos em uma unica grande caixa e organiza as caixas das vacas em uma lista.
        Isso facilitará a verificação de perigo para cada vaca.

        Args:
            img (np.ndarray): Imagem de entrada
            results ( list(dict) ): Lista de dicionários com as detecções (classe, confidence, bbox(x1, y1, x2, y2))

        Returns:
            img: Imagem de saída
            animais: Dicionário com as caixas dos animais no formato {'vaca': [[(xmin, ymin, xmax, ymax)],[...],...], 'lobo': [xmin, ymin, xmax, ymax]}
        """
        img = img.copy()
        animais = {'vaca': [], 'lobo': []}
        for resultado in resultados:
            if resultado['classe'] == 'cow':
                x1, y1, x2, y2 = resultado['bbox']
                cv2.rectangle(img, (x1, y1), (x2, y2), [0,255,0], 2)
                animais['vaca'].append([x1, y1, x2, y2])
            else:
                x1, y1, x2, y2 = resultado['bbox']
                animais['lobo'].append([x1, y1, x2, y2])

        xmin = min([i[0] for i in animais['lobo']])
        ymin = min([i[1] for i in animais['lobo']])
        xmax = max([i[2] for i in animais['lobo']])
        ymax = max([i[3] for i in animais['lobo']])
        animais['lobo'] = [xmin, ymin, xmax, ymax]
        cv2.rectangle(img, (xmin, ymin), (xmax, ymax), [0,0,255], 2)

        return img, animais

    def calcula_iou(self, boxA: list, boxB: list) -> float:
        """Calcula a intersecção sobre a união (Intersection over Union - IoU) entre duas caixas.

        Args:
            boxA (list): Caixa 1 no formato: [xmin, ymin, xmax, ymax]
            boxB (list): Caixa 2 no formato: [xmin, ymin, xmax, ymax]

        Returns:
            iou: Retorna o valor da IoU entre as duas caixas
        """
        xA = min(boxA[0], boxB[0])
        yA = min(boxA[1], boxB[1])
        xB = max(boxA[2], boxB[2])
        yB = max(boxA[3], boxB[3])

        # compute the area of intersection rectangle
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
        # compute the area of both the prediction and ground-truth
        # rectangles
        boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = interArea / float(boxAArea + boxBArea - interArea)

        return iou

    def checar_perigo(self, image: np.ndarray, animais: dict) -> np.ndarray:
        """Não mude ou renomeie esta função
            Recebe as coordenadas das caixas, se a caixa de uma vaca tem intersecção com as do lobo, ela esta em perigo.
            Se estiver em perigo, deve escrever na imagem com a cor vermlha, se não, escreva com a cor azul.
            *Importante*: nesta função, não faça cópia da imagem de entrada!!
            
            Repita para cada vaca na imagem.

        Args:
            image (np.ndarray): Imagem de entrada
            animais (dict): Dicionário com as caixas dos animais no formato {'vaca': [[(xmin, ymin, xmax, ymax)],[...],...], 'lobo': [xmin, ymin, xmax, ymax]}

        Returns:
            image: Imagem de saída com as caixas desenhadas e o texto indicando se está em perigo ou não
        """

        for i in animais['vaca']:
            iou = self.calcula_iou(animais['lobo'], i)
            if iou > 0.5:
                image = cv2.putText(image,"Esta em perigo",(i[0], i[1]),cv2.FONT_HERSHEY_SIMPLEX,1.5,[0,0,255],3)
            else:
                image = cv2.putText(image,'Esta sem perigo', (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, [255, 0, 0], 3)
            
        return image


def main():
    import time
    bgr = cv2.imread("img/cow_wolf05.png")

    Detector = DangerDetector()
    image, results = Detector.detect(bgr)
    image, animais = Detector.separar_caixa_entre_animais(image, results)
    print(animais)
    image = Detector.checar_perigo(image, animais)

    cv2.imshow("Result_MobileNet", image)
    cv2.waitKey(0)



    

if __name__ == "__main__":
    main()