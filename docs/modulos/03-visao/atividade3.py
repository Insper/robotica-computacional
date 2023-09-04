import cv2
import numpy as np
import argparse
from module_net import MobileNetDetector

class DangerDetector(MobileNetDetector):
    def __init__(self):
        super().__init__()
        
    def separar_caixa_entre_animais(self, img, resultados):
        img = img.copy()
        animais = {'vaca': [], 'lobo': []}
        for i in range(len(resultados)):
            if resultados[i][0] == 'cow':
                cv2.rectangle(img, (resultados[i][2][0], resultados[i][2][1]), (resultados[i][3][0], resultados[i][3][1]), [255,0,0], 2)
                animais['vaca'].append([resultados[i][2][0], resultados[i][2][1], resultados[i][3][0], resultados[i][3][1]])
            else:
                animais['lobo'].append([resultados[i][2][0], resultados[i][2][1], resultados[i][3][0], resultados[i][3][1]])

        retang_lobo = []
        if len(animais['lobo']) == 2:
            if animais['lobo'][0][0] > animais['lobo'][1][0]: #pegando o menor xmin
                retang_lobo.append(animais['lobo'][1][0])
            else:
                retang_lobo.append(animais['lobo'][0][0])

            if animais['lobo'][0][1] > animais['lobo'][1][1]: #pegando o menor ymin
                retang_lobo.append(animais['lobo'][1][1])
            else:
                retang_lobo.append(animais['lobo'][0][1])

            if animais['lobo'][0][2] > animais['lobo'][1][2]: #pegando o maior xmax
                retang_lobo.append(animais['lobo'][0][2])
            else:
                retang_lobo.append(animais['lobo'][1][2])

            if animais['lobo'][0][3] > animais['lobo'][1][3]: #pegando o maior ymax
                retang_lobo.append(animais['lobo'][0][3])
            else:
                retang_lobo.append(animais['lobo'][1][3])
        
            animais['lobo'] = retang_lobo
            cv2.rectangle(img, (retang_lobo[0], retang_lobo[1]),(retang_lobo[2], retang_lobo[3]), [0,0,255], 2)
        else:
            cv2.rectangle(img, (animais['lobo'][0][0], animais['lobo'][0][1]), (animais['lobo'][0][2], animais['lobo'][0][3]), [0,0,255], 2)

        return img, animais

    def calcula_iou(self, boxA, boxB):
        """Não mude ou renomeie esta função
            Calcula o valor do "Intersection over Union" para saber se as caixa se encontram
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

    def checar_perigo(self, image, animais):
        """Não mude ou renomeie esta função
            Recebe as coordenadas das caixas, se a caixa de uma vaca tem intersecção com as do lobo, ela esta em perigo.
            Se estiver em perigo, deve escrever na imagem com a cor vermlha, se não, escreva com a cor azul.
            *Importante*: nesta função, não faça cópia da imagem de entrada!!
            
            Repita para cada vaca na imagem.
        """

        for i in animais['vaca']:
            iou = self.calcula_iou(animais['lobo'][0], i)
            if iou > 0.5:
                image = cv2.putText(image,"Esta em perigo",(i[0], i[1]),cv2.FONT_HERSHEY_SIMPLEX,1.5,[0,0,255],3)
            else:
                image = cv2.putText(image,'Esta sem perigo', (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, [255, 0, 0], 3)
            
        return image


def main():
    import time
    bgr = cv2.imread("img/cow_wolf03.png")

    Detector = DangerDetector()
    image, results = Detector.detect(bgr)
    image, animais = Detector.separar_caixa_entre_animais(image, results)
    image = Detector.checar_perigo(image, animais)

    cv2.imshow("Result_MobileNet", image)
    cv2.waitKey(0)



    

if __name__ == "__main__":
    main()