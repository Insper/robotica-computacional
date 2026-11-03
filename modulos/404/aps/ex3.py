from module_aruco import Aruco3d
from module import ImageModule
import cv2
import numpy as np

class DistanceEstimator(Aruco3d, ImageModule):
    def __init__(self):
        Aruco3d.__init__(self)
        ImageModule.__init__(self)

        # configure o kernel

        self.filters = {
            'blue': {
                'lower': np.array([0, 0, 0]),
                'upper': np.array([180, 255, 255])
            },
            'green': {
                'lower': np.array([0, 0, 0]),
                'upper': np.array([180, 255, 255])
            },}
        
    def find_creeper(self, hsv: np.ndarray, color: str) -> list:
        """
        Encontra o creeper na imagem com base na cor fornecida.
        
        Args:
            hsv (numpy.array): imagem no espaço de cor HSV.
            color (str): cor do creeper para identificação.

        Returns:
            list: lista de centros dos creepers detectados e a cor atual. No formato list( [(cx,cy), color], [...]... ).
        """
        creepers = []
        
        return creepers

    def distance(self, p1, p2):
        """
        Calcula a distância euclidiana entre dois pontos.
        
        Args:
            p1 (tuple): primeiro ponto.
            p2 (tuple): segundo ponto.

        Returns:
            distance float: distância entre os dois pontos.
        """
        distance = None
        return distance

    def match_aruco(self, bgr, creepers, results):
        """
        Combina os marcadores Aruco com os creepers mais próximos.
        
        Args:
            bgr (numpy.array): imagem no espaço de cor BGR.
            creepers (list): lista de centros dos creepers detectados através da função find_creeper.
            results (list(dicts)): resultados da detecção Aruco.
                dict_keys(['id', 'rvec', 'tvec', 'distancia', 'corners', 'centro'])

        Returns:
            bgr (numpy.array): imagem com linhas desenhadas e pares combinados.
            matched_pairs (dict): detecções Aruco combinadas com o centro do "corpo" do creeper mais próximo, na chave "body_center".
            e a cor do creeper na chave "color". Remova creepers sem correspondência.
        """
        matched_pairs = []
        for creeper in creepers:
            closest = None# Use a função sorted para ordenar os resultados por distância com base na função self.distance.

            # Remove o creeper mais próximo dos resultados para evitar que ele seja combinado com outro creeper.
            results = [res for res in results if not np.array_equal(res['distancia'], closest['distancia'])]

            # Adiciona o centro do creeper mais próximo na chave "body_center" do dicionário closest.
            # Adiciona a cor do creeper mais próximo na chave "color" do dicionário closest.
            
            # Desenha uma linha entre o centro do creeper e o centro do marcador Aruco.
            
            # Adiciona o par combinado na lista matched_pairs.
        
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
            closest_aruco (list): cor e id do Aruco mais próximo.
        """
        _, results = None# Chame a função self.detect_aruco e armazene os resultados em uma variável.
        
        hsv = None# Converta a imagem BGR para HSV.
        creepers = []
        creepers += self.find_creeper(hsv, "green")
        creepers += self.find_creeper(hsv, "blue")

        matched_pairs = None# Faz match entre os marcadores Aruco e os creepers.

        for result in matched_pairs:
            bgr = self.drawAruco(bgr, result)
            
        ranked_arucos = None# Classifica os Arucos por distância.

        closest_aruco = None# Retorna o Aruco mais próximo.

        return bgr, ranked_arucos, closest_aruco
    

def rodar_webcam():
    Arucos = DistanceEstimator()
    # cap = cv2.VideoCapture(0) # webcam
    cap = cv2.VideoCapture('img/aruco.mp4') # Confira se o video esta na pasta img

    while True:
        ret, bgr = cap.read()
        bgr, ranked_arucos, closest_aruco = Arucos.run(bgr)

        # Imprime o id e a cor do Aruco mais próximo.


        cv2.imshow("Imagem", bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
def main():
    # Selecione se deseja rodar seu codigo com uma imagem ou um video:

    rodar_webcam()


if __name__ == "__main__":
    main()