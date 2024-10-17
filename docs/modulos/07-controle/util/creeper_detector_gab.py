from robcomp_util.module_aruco import Aruco3d
import cv2
import numpy as np

class CreeperDetector(Aruco3d):
    def __init__(self):
        Aruco3d.__init__(self)
        self.kernel = np.ones((5,5),np.uint8)

        self.filters = {
            'blue': {
                'lower': np.array([100, 50, 50]),
                'upper': np.array([140, 255, 255])
            },
            'green': {
                'lower': np.array([40, 50, 20]),
                'upper': np.array([80, 255, 255])
            },}
        
    def find_creeper(self, bgr: np.ndarray, color: str) -> list:
        """
        Encontra o creeper na imagem com base na cor fornecida.
        
        Args:
            bgr (numpy.array): imagem no espaço de cor BGR.
            color (str): cor do creeper para identificação.

        Returns:
            list: lista de centros dos creepers detectados e a cor atual. No formato list( [(cx,cy), color], [...]... ).
        """
        creepers = []
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.filters[color]['lower'], self.filters[color]['upper'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1000:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w//2, y + h//2
            creepers.append([(cx, cy), color])

        return creepers

    def distance(self, x1, x2):
        """
        Calcula a distância horizontal entre duas coordenadas.
        """
        distance = abs(x1 - x2)
        return distance

    def match_aruco(self, bgr, creepers, results):
        """
        Combina os marcadores Aruco com os creepers mais próximos.
        
        Args:
            bgr (numpy.array): imagem no espaço de cor BGR.
            creepers (list): lista de centros dos creepers detectados através da função `find_creeper`.
                list( [(cx,cy), color], [...]... ).
            results (list(dicts)): resultados da detecção Aruco.
                dict_keys(['id', 'rvec', 'tvec', 'distancia', 'corners', 'centro'])

        Returns:
            bgr (numpy.array): imagem com linhas desenhadas e pares combinados.
            matched_pairs (dict): detecções Aruco combinadas com o centro do "corpo" do creeper mais próximo, na chave "body_center".
            e a cor do creeper na chave "color". Remova creepers sem correspondência.
        """
        matched_pairs = []
        for cabeca in results:
            
            # 1. Use a função min para ordenar os corpinhos (creepers) por distância com base na função self.distance.
                #   Dica: Utilize a função lambda para acessar a chave 'centro' do dicionário `results`.
            closest = min(creepers, key=lambda creeper: self.distance(cabeca['centro'][0], creeper[0][0]))

            # 2. Remove da lista `creepers` o corpinho mais próximo da cabeca `creeper` para evitar que ele seja combinado novamente.
                #   Dica: Pode ser feito utilizando list comprehension.
            creepers = [creeper for creeper in creepers if creeper[0] != closest[0]]

            # 3. Adiciona na variável o centro do creeper mais próximo na chave "body_center" do dicionário `cabeca`.
            cabeca['body_center'] = closest[0]
            # 4. Adiciona a cor do creeper mais próximo na chave "color" do dicionário `cabeca`.
            cabeca['color'] = closest[1]
            
            # 5. Desenha uma linha entre o centro do creeper e o centro do marcador Aruco.
            cv2.line(bgr, tuple(cabeca['centro']), (cabeca['body_center']), (0, 0, 255), 2)
            
            # 6. Adiciona o par combinado na lista `matched_pairs`.
            matched_pairs.append(cabeca)

            if len(results) == 0:
                break
        
        return bgr, matched_pairs

    def run(self, bgr):
        """
        Executa a detecção de Aruco e correspondência com creepers.
        
        Args:
            bgr (numpy.array): imagem no espaço de cor BGR.

        Returns:
            bgr (numpy.array): imagem no espaço de cor BGR com linhas desenhadas entre Arucos e creepers.
            ranked_arucos (list(dicts)): imagem processada e Arucos classificados por distância.
                dict_keys(['id', 'rvec', 'tvec', 'distancia', 'corners', 'centro', 'body_center', 'color'])
        """
        # 1. Chame a função self.detect_aruco e armazene os resultados em uma variável.
        _, results = self.detectaAruco(bgr) 
        
        creepers = []
        creepers += self.find_creeper(bgr, "green")
        creepers += self.find_creeper(bgr, "blue")

        if len(creepers) == 0 or len(results) == 0:
            return bgr, []

        # 2. Desenvolva a função `match_aruco` para combinar os marcadores Aruco com os corpos dos creepers.
        bgr, matched_pairs = self.match_aruco(bgr, creepers, results)

        for result in matched_pairs:
            bgr = self.drawAruco(bgr, result)

        
        # 5. Passe novamente por creepers e adicione cores sem correspondência caso não exista um marcador Aruco visível.
            # Deixe a chave 'id' como '0'
        for creeper in creepers:
            matched_pairs.append({
                'body_center' : creeper[0],
                'color' : creeper[1],
                'id': 0,
            })

        return bgr, matched_pairs
    
def main():
    Arucos = CreeperDetector()
    
    bgr = cv2.imread("/home/borg/Documents/robotica-computacional/docs/modulos/07-controle/atividades/img/aruco.jpg")
    # bgr = cv2.imread("img/aruco2.jpg")

    bgr, ranked_arucos = Arucos.run(bgr)
    cv2.imshow("Imagem", bgr)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()