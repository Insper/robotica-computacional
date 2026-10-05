# https://codeshare.io/XLvlAY
import cv2
import numpy as np

class ProcessImage:
    def __init__(self):
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        self.bgr = None
        self.cores = {
            "marron": [np.array([10, 63, 177]),  np.array([20, 116, 250])],
            "azul": [np.array([60, 50, 50]), np.array([100, 255, 250])],
            "vermelho": [(0,255,255), (15,255,255)],
            "amarelo": [(8,255,60), (28,255,255)],
            "preto":[0,50],
        }
        
    def filter_bw(self,img,cor):
        latinhas_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        latinhas_gray = cv2.inRange(latinhas_gray, self.cores[cor][0], self.cores[cor][1])

        # realiza a abertura
        mask = cv2.morphologyEx(latinhas_gray, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contornos, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    
        return contornos
    
    def filter_hsv(self,img,cor):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_hsv = cv2.inRange(img_hsv, self.cores[cor][0], self.cores[cor][1])

        mask = cv2.morphologyEx(img_hsv, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contornos, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
        return contornos
    def filter_hsv_area(self,img,cor):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, self.cores[cor][0], self.cores[cor][1])

        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return np.sum(mask) / 255
    def filter_bw_area(self,img,cor):
        latinhas_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(latinhas_gray, self.cores[cor][0], self.cores[cor][1])

        # realiza a abertura
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        return np.sum(mask) / 255
        
    def get_center(self,contornos):
        
        centros = []
        for contour in contornos:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centros.append((cx, cy))
        return centros

    def run_image(self,img):
        self.bgr = img
        contornos = self.filter_hsv(img,"marron")
        maior = max(contornos, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(maior)
        self.tabuleiro = self.bgr[y:y+h, x:x+w]

        lado = w // 8
        resultado = []
        print(resultado)
        for i in range(8):
            for j in range(8):
                print("i=", i, "j=", j)
                casa = self.tabuleiro[lado*i:lado*(i+1),lado*j:lado*(j+1)]
                preto = self.filter_bw_area(casa, "preto")
                if preto == 0:
                    resultado.append("vazio")
                else:
                    print(preto/(w*h))
                    if preto/(w*h) < 0.0030:
                        peca = "W"
                    else:
                        peca = "B"

                    amarelo = self.filter_hsv_area(casa, "amarelo") == 0
                    azul = self.filter_hsv_area(casa, "azul") == 0
                    vermelho = self.filter_hsv_area(casa, "vermelho") == 0
                    cores = (amarelo, azul, vermelho)
                    print(cores)

                    if cores == (True,False,False):
                        peca=peca+"T"
                    elif cores == (False, False, True):
                        peca=peca+"K"
                    elif cores == (False, True, False):
                        peca=peca+"Q"
                    elif cores == (False, True, True):
                        peca=peca+"H"
                    elif cores == (True, True, False):
                        peca=peca+"B"
                    else:
                        peca=peca+"P"

                    resultado.append(f"{peca}")

        resultado = np.array(resultado).reshape((8,8))
        print(resultado)

        
                


    def show_image(self):
        cv2.imshow("Imagem Processada", self.tabuleiro)
        cv2.waitKey(0)  
        cv2.destroyAllWindows()

def main():
    img_path = '/home/borg/colcon_ws/src/simulado_ai/simulado_ai/q2/exemplo3.png'
    img = cv2.imread(img_path)
    processor = ProcessImage()
    processor.run_image(img)
    processor.show_image()

if __name__ == '__main__':
    main()