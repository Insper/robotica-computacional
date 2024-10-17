from robcomp_util.module_aruco import Aruco3d
import cv2
import numpy as np

class CreeperDetector(): # Importe a classe Aruco3d
    def __init__(self):
        # Inicialize a classe Aruco3d
        self.kernel = np.ones((5,5),np.uint8)

        self.filters = {
            'blue': {
                'lower': (0,0,0),
                'upper': (255,255,255)
            },
            'green': {
                'lower': (0,0,0),
                'upper': (255,255,255)
            },
            'red': {
                'lower': (0,0,0),
                'upper': (255,255,255)
            },
            }
        
    def find_creeper(self, bgr: np.ndarray, color: str) -> list:
        """
        Encontra o creeper na imagem com base na cor fornecida.
        
        Args:
            bgr (numpy.array): imagem no espaço de cor BGR.
            color (str): cor do creeper para identificação.

        Returns:
            list: lista de centros dos creepers detectados e a cor atual.
                   No formato list( [(cx,cy), color], [...]... ).
        """
        creepers = []
        # 1. Converter para hsv e filtrar a partir da chave `color` do dicionário `self.filters`.
        # 2. Utilize a função cv2.morphologyEx para aplicar a operação de abertura e fechamento.
        # 3. Encontre os contornos e adicione o centro do creeper e o nome da cor na lista `creepers`.
        
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
            closest = min(...)

            # 2. Remove da lista `creepers` o corpinho mais próximo da cabeca `creeper` para evitar que ele seja combinado novamente.
                #   Dica: Pode ser feito utilizando list comprehension.
            creepers = ...

            # 3. Adiciona na variável o centro do creeper mais próximo na chave "body_center" do dicionário `cabeca`.
            cabeca['body_center'] = ...
            # 4. Adiciona a cor do creeper mais próximo na chave "color" do dicionário `cabeca`.
            cabeca['color'] = ...
            
            # 5. Desenha uma linha entre o centro do creeper e o centro do marcador Aruco.
            cv2.line()
            
            # 6. Adiciona o par combinado na lista `matched_pairs`.
            matched_pairs...

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
        _, results = ...

        # 2. Chame a função self.find_creeper para encontrar os creepers na imagem de cada cor.        
        creepers = []
        creepers += ...
        creepers += ...
        creepers += ...

        if len(creepers) == 0 or len(results) == 0: # Verifica se não há creepers ou arucos na imagem.
            return bgr, []

        # 3. Desenvolva a função `match_aruco` para combinar os marcadores Aruco com os corpos dos creepers.
        bgr, matched_pairs = ...


        # 4. Desenha os marcadores Aruco na imagem utilizando a função `drawAruco`.
        for result in matched_pairs:
            bgr = ...

        # 5. Passe novamente por creepers e adicione cores sem correspondência caso não exista um marcador Aruco visível.
            # Deixe a chave 'id' como '0'
        matched_pairs = ...

        return bgr, matched_pairs
    
def main():
    Arucos = CreeperDetector()
    
    bgr = cv2.imread("img/aruco.jpg")
    # bgr = cv2.imread("img/aruco2.jpg")

    bgr, ranked_arucos = Arucos.run(bgr)
    cv2.imshow("Imagem", bgr)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()