import cv2
import numpy as np
from module import ImageModule

class IdentificadorBandeiras(ImageModule):
    def __init__(self) -> None:
        super().__init__()

        self.configure_kernel(5, "rect")
        self.colors = {
            'orange': ((0, 100, 100), (20, 255, 255)),
            'red': ((170, 100, 100), (180, 255, 255)),
            'green': ((70, 100, 100), (80, 255, 255)),
            'white': ((0, 0, 200), (180, 20, 255)),
        }
        self.bandeiras = []

    def draw_bandeiras(self, bgr):
        for bandeira in self.bandeiras:
            cv2.rectangle(bgr, bandeira[1], bandeira[2], (255, 0, 0), 5)
            cv2.putText(bgr, bandeira[0], bandeira[1], cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

    def checar_bandeiras(self, hsv, rects):
        for rect in rects:
            x, y, w, h = rect
            crop = hsv[y:y+h, x:x+w]
            cv2.imshow('crop', cv2.cvtColor(crop, cv2.COLOR_HSV2BGR))

            # Irlanda
            if np.sum(self.color_filter(crop, self.colors['orange'][0], self.colors['orange'][1])) > 0:
                self.bandeiras.append(('irlanda', (x, y), (x+w, y+h)))
            elif np.sum(self.color_filter(crop, self.colors['green'][0], self.colors['green'][1])) > 0:
                self.bandeiras.append(('italia', (x, y), (x+w, y+h)))
            else:
                area = w * h
                print(area)
                mask_r = self.color_filter(crop, self.colors['red'][0], self.colors['red'][1])
                cv2.imshow('mask_r', mask_r)
                red = np.sum(mask_r) / 255
                mask_w = self.color_filter(crop, self.colors['white'][0], self.colors['white'][1])
                cv2.imshow('mask_w', mask_w)
                white = np.sum(mask_w) / 255
                if white > red:
                    self.bandeiras.append(('singapura', (x, y), (x+w, y+h)))
                elif red > white:
                    self.bandeiras.append(('peru', (x, y), (x+w, y+h)))
                else:
                    self.bandeiras.append(('monaco', (x, y), (x+w, y+h)))

    def get_all_contours(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        mask = self.color_filter(gray, 20, 255)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        rects = []
        for contour in contours:
            rect = cv2.boundingRect(contour)
            rects.append(rect)
        
        return bgr, rects

    def run(self,bgr):
        '''
        Essa função deverá identificar as bandeiras na imagem passada como argumento
        e devolver uma lista de tuplas no formato

        ('pais', (x1, y1), (x2, y2))
        '''
        bgr,  rects = self.get_all_contours(bgr)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        self.checar_bandeiras(hsv, rects)

        bgr = self.draw_bandeiras(bgr)

        return self.bandeiras


if __name__ == '__main__':
    bgr = cv2.imread('img/teste1.png')
    q1 = IdentificadorBandeiras()
    items = q1.run(bgr)

    print(items)
    
    cv2.imshow("Resultado", bgr)
    cv2.waitKey(0)
