import cv2.aruco as aruco
import cv2
import numpy as np
import os

#definindo a classe Aruco3d
class Aruco3d():
    def __init__(self):
            #Definindo a variavel ids para armazenar o Id do aruco detectado
            self.ids = None 

            #Capturando o caminho da pasta local em que o codigo esta
            calibra_path  = os.path.dirname(os.path.abspath(__file__))
            
            # Carregando os arquivos de calibracao da camera
            self.camera_matrix   = np.loadtxt(calibra_path+'/config/cameraMatrix_realsense.txt', delimiter=',')
            self.camera_distortion   = np.loadtxt(calibra_path+'/config/cameraDistortion_realsense.txt', delimiter=',')

    def detectaAruco(self,bgr):
        # Gera a mascara em escalas de Cinza apartir da copia da imagem em BGR
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        #Define o Dicionario para o Aruco que vamos utilizar
        dicionarioAruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
        
        #Detecta os Arucos e Carrega os pontos de translação e de rotação do aruco em relação ao robo
        cornersList, ids, _ = aruco.detectMarkers(gray, dicionarioAruco)
        
        results = []
        # Se um Id foi detectado, verifica se ele esta dentro da range de 0 a 99 e calcula os valores de rotação e translação 
        if ids is not None:
            for i in range(len(ids)):
                ret = aruco.estimatePoseSingleMarkers(cornersList[i], 6, self.camera_matrix, self.camera_distortion)
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    
                results.append({
                    'id': ids[i],
                    'rvec': rvec,
                    'tvec': tvec,
                    'distancia':np.linalg.norm(tvec),
                    'corners': cornersList[i],
                    'centro': np.mean(cornersList[i], axis=1).astype("int").flatten()
                })
        
        #retorna os ids e as coordenadas de centro e de distancia do aruco em relação ao robo
        return bgr, results

    def drawAruco(self, bgr, result):
            # Desenha a linha referencia em X
            cv2.line(bgr, (bgr.shape[1]//2,bgr.shape[0]//2), ((bgr.shape[1]//2 + 50),(bgr.shape[0]//2)), (0,0,255), 5) 
            # Desenha a linha referencia em Y
            cv2.line(bgr, (bgr.shape[1]//2,bgr.shape[0]//2), (bgr.shape[1]//2,(bgr.shape[0]//2 + 50)), (0,255,0), 5) 

            #-- Desenha um retanculo e exibe Id do marker encontrado
            cv2.drawFrameAxes(bgr, self.camera_matrix, self.camera_distortion, result['rvec'], result['tvec'] ,0.03)

            aruco.drawDetectedMarkers(bgr, np.array([result['corners']]), np.array([result['id']]))

            return bgr

    def writeDistance(self, bgr, distancia):
        cv2.putText(bgr, f"Distancia: {distancia:.2f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return bgr

    
def main():
    Arucos = Aruco3d()

    bgr = cv2.imread("img/aruco.png")
    #Chama a funcao detectaAruco
    bgr, results = Arucos.detectaAruco(bgr)
    
    for result in results:
        bgr = Arucos.drawAruco(bgr, result)

    print(results[0])

    cv2.imshow("Aruco", bgr)
    cv2.waitKey(0)

if __name__ == "__main__":
    main()