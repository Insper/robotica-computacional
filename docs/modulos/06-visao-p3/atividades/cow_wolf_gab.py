import cv2
import numpy as np
import argparse
from module_net import MobileNetDetector

class DangerDetector(MobileNetDetector): # Esta classe deve herdar da classe MobileNetDetector
    def __init__(self, CONFIDENCE=0.7):
        # Inicialize a classe Pai
        super().__init__()

        # Ajuste o valor de confiança para o valor que você achar melhor
        self.CONFIDENCE = CONFIDENCE
        
    def separar_caixa_entre_animais(self, img: np.ndarray, resultados: list) -> (np.ndarray, dict):
        """Não mude ou renomeie esta função
        Combina as caixas dos lobos em uma unica grande caixa e organiza as caixas das vacas em uma lista.
        Isso facilitará a verificação de perigo para cada vaca.

        Args:
            img (np.ndarray): Imagem de entrada
            resultados (list): Lista com as detecções no formato [classe, confiança, (xmin, ymin), (xmax, ymax)]

        Returns:
            img: Imagem de saída
            animais: Dicionário com as caixas dos animais no formato {'vaca': [[(xmin, ymin, xmax, ymax)],[...],...], 'lobo': [xmin, ymin, xmax, ymax]}
        """
        img = img.copy()
        animais = {'vaca': [], 'lobo': []}

        x_lobos = []
        y_lobos = []
        for resultado in resultados:
            if resultado['classe'] == 'cow':
                animais['vaca'].append(resultado['bbox'])
                x1, y1, x2, y2 = resultado['bbox']
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            else:
                x_lobos.append(resultado['bbox'][0])
                y_lobos.append(resultado['bbox'][1])
                x_lobos.append(resultado['bbox'][2])
                y_lobos.append(resultado['bbox'][3])
        
        x1, y1, x2, y2 = (min(x_lobos), min(y_lobos), max(x_lobos), max(y_lobos))
        animais['lobo'] = [x1, y1, x2, y2]
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        return img, animais

    def calcula_iou(self, boxA: list, boxB: list) -> float:
        """Calcula a intersecção sobre a união (Intersection over Union - IoU) entre duas caixas.

        Args:
            boxA (list): Caixa 1 no formato: [xmin, ymin, xmax, ymax]
            boxB (list): Caixa 2 no formato: [xmin, ymin, xmax, ymax]

        Returns:
            iou: Retorna o valor da IoU entre as duas caixas
        """
        # Coordenadas de interseção
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])

        # Área de interseção
        interArea = max(0, xB - xA) * max(0, yB - yA)

        # Área das caixas
        boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
        boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

        # Cálculo do IoU
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
        for vaca in animais['vaca']:
            iou = self.calcula_iou(vaca, animais['lobo'])
            if iou > 0:
                cv2.rectangle(image, (vaca[0], vaca[1]), (vaca[2], vaca[3]), (0, 0, 255), 2)
                cv2.putText(image, 'PERIGO', (vaca[0], vaca[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                cv2.rectangle(image, (vaca[0], vaca[1]), (vaca[2], vaca[3]), (255, 0, 0), 2)
                cv2.putText(image, 'Seguro', (vaca[0], vaca[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
        return image



def main():
    import time
    bgr = cv2.imread("img/cow_wolf_5.png")

    Detector = DangerDetector()
    image, results = Detector.detect(bgr)
    image, animais = Detector.separar_caixa_entre_animais(image, results)
    print(animais)
    image = Detector.checar_perigo(image, animais)

    cv2.imshow("Result_MobileNet", image)
    cv2.waitKey(0)



    

if __name__ == "__main__":
    main()