import cv2
class PrateleiraArrumada():
    def __init__(self) -> None:
        self.prateleira_cima = 0
        self.prateleira_baixo = 0
        self.prateleira_cima_arrumada = 0
        self.prateleira_baixo_arrumada = 0

    def run(self,bgr):
        '''
        Devolve 4 valores:

        - número de produtos na prateleira de cima
        - número de produtos que estão arrumados (prateleira correta e orientação correta) na prateleira de cima
        - número de produtos na prateleira de baixo
        - número de produtos que estão arrumados (prateleira correta e orientação correta) na prateleira de baixo
        '''

        return 0,0,0,0

if __name__ == '__main__':
    bgr = cv2.imread("img/teste1.png")
    q2 = PrateleiraArrumada()
    v = q2.run(bgr)

    print(f'''
    - {v[0]} produtos na prateleira de cima
    - {v[1]} estão arrumados (prateleira correta e orientação correta) na prateleira de cima
    - {v[2]} na prateleira de baixo
    - {v[3]} estão arrumados (prateleira correta e orientação correta) na prateleira de baixo
''')