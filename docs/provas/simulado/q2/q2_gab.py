import cv2
from module_ import ImageModule

class PrateleiraArrumada(ImageModule):
    def __init__(self) -> None:
        super().__init__()
        self.prateleira_cima = 0
        self.prateleira_baixo = 0
        self.prateleira_cima_arrumada = 0
        self.prateleira_baixo_arrumada = 0

        self.color = {
            'magenta': ((150, 100, 100), (160, 255, 255)),
            'yellow':  ((30, 100, 100), (40, 255, 255)),
        }

        self.configure_kernel(3, 'rect')

    def get_rectangles(self, hsv, color):
        mask = self.color_filter(hsv, self.color[color][0], self.color[color][1])
        mask = self.morphological_transform(mask, 'open')
        contours = self.find_contours(mask)

        rects = []
        for c in contours:
            rect = cv2.boundingRect(c)
            rects.append(rect)
        return rects      

    def verificar_prateleiras(self, rects_m, rects_y):
        cima = []
        baixo = []
        for rect in rects_m:
            x, y, w, h = rect
            cy = y + h // 2
            
            if cy < self.middle:
                cima.append((rect, 'magenta', w/h < 1))
            else:
                baixo.append((rect, 'magenta', w/h > 1))
        
        for rect in rects_y:
            x, y, w, h = rect
            cy = y + h // 2
            if cy < self.middle:
                cima.append((rect, 'yellow', h/w > 1))
            else:
                baixo.append((rect, 'yellow', h/w < 1))

        return cima, baixo
        
    def run(self,bgr):
        '''
        Devolve 4 valores:

        - número de produtos na prateleira de cima
        - número de produtos que estão arrumados (prateleira correta e orientação correta) na prateleira de cima
        - número de produtos na prateleira de baixo
        - número de produtos que estão arrumados (prateleira correta e orientação correta) na prateleira de baixo
        '''
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        self.middle = bgr.shape[0] // 2
        print(self.middle)

        rects_m = self.get_rectangles(hsv, 'magenta')
        rects_y = self.get_rectangles(hsv, 'yellow')

        cima, baixo = self.verificar_prateleiras(rects_m, rects_y)
        self.prateleira_cima = len(cima)
        self.prateleira_baixo = len(baixo)

        for rect, color, orientacao in cima:
            if orientacao and color == 'yellow':
                self.prateleira_cima_arrumada += 1
        
        for rect, color, orientacao in baixo:
            if orientacao and color == 'magenta':
                self.prateleira_baixo_arrumada += 1

        return self.prateleira_cima, self.prateleira_cima_arrumada, self.prateleira_baixo, self.prateleira_baixo_arrumada

if __name__ == '__main__':
    bgr = cv2.imread("img/teste4.png")
    q2 = PrateleiraArrumada()
    v = q2.run(bgr)

    print(f'''
    - {v[0]} produtos na prateleira de cima
    - {v[1]} estão arrumados (prateleira correta e orientação correta) na prateleira de cima
    - {v[2]} na prateleira de baixo
    - {v[3]} estão arrumados (prateleira correta e orientação correta) na prateleira de baixo
''')
    cv2.imshow('img', bgr)
    cv2.waitKey(0)