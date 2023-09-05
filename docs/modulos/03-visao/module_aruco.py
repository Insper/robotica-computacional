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
            
            # =================================== APENAS PARA O ROBO FISICO ===========================================================
            # ========================================= IMPORTANTE!!!! ================================================================
            # ===== Se estiver usando o robo fisico, e o seu robo está com a RealSense, troque o arquivo de calibração =====
            # ===== Verifique se o arquivo cameraDistortion_realsense.txt e cameraMatrix_realsense.txt esta disponivel na pasta arucopath ====
            # ===== Troque o nome do arquivo nas variaveis camera_matrix e camera_distortion para cameraDistortion_realsense.txt e cameraMatrix_realsense.txt respectivamente. 
            # ===== SE O SEU ROBO ESTA USANDO A RASPICAM, NAO PRECISA ALTERAR NADA!
            # ===== Caso não saiba do que eu estou falando, por favor, jogue no google imagens "Camera RealSense D400", essa é a camera RealSense.
            
            
            #Carregando os arquivos de calibracao da camera
            #Exemplo de calibracao para a raspcam, funciona no robo simulado e no robo real que está com a Raspcam
            self.camera_matrix   = np.loadtxt(calibra_path+'/config/cameraMatrix_realsense.txt', delimiter=',')
            self.camera_distortion   = np.loadtxt(calibra_path+'/config/cameraDistortion_realsense.txt', delimiter=',')

    def detectaAruco(self,bgr):
        # Gera a mascara em escalas de Cinza apartir da copia da imagem em BGR
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        #Define o Dicionario para o Aruco que vamos utilizar
        dicionarioAruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
        
        #Detecta os Arucos e Carrega os pontos de translação e de rotação do aruco em relação ao robo
        cornersList, ids, _ = aruco.detectMarkers(
            gray, dicionarioAruco)
        
        # Desenha a linha referencia em X
        cv2.line(bgr, (bgr.shape[1]//2,bgr.shape[0]//2), ((bgr.shape[1]//2 + 50),(bgr.shape[0]//2)), (0,0,255), 5) 
        # Desenha a linha referencia em Y
        cv2.line(bgr, (bgr.shape[1]//2,bgr.shape[0]//2), (bgr.shape[1]//2,(bgr.shape[0]//2 + 50)), (0,255,0), 5) 

        aruco.drawDetectedMarkers(bgr, cornersList, ids)

        results = []
        # Se um Id foi detectado, verifica se ele esta dentro da range de 0 a 99 e calcula os valores de rotação e translação 
        if ids is not None:
            for i in range(len(ids)):
                if ids[i]>99:
                    ret = aruco.estimatePoseSingleMarkers(cornersList[i], 19, self.camera_matrix, self.camera_distortion)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                else: 
                    ret = aruco.estimatePoseSingleMarkers(cornersList[i], 6, self.camera_matrix, self.camera_distortion)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    
            #-- Desenha um retanculo e exibe Id do marker encontrado
            cv2.drawFrameAxes(bgr, self.camera_matrix, self.camera_distortion, rvec, tvec ,0.03)

            results.append({
                'id': ids[i],
                'rvec': rvec,
                'tvec': tvec,
                'distancia':np.linalg.norm(tvec),
                'centro': np.mean(cornersList[i], axis=1).astype("int").flatten()
            })
        
        #retorna os ids e as coordenadas de centro e de distancia do aruco em relação ao robo
        return bgr, results
            
                    
if __name__ == "__main__":
    #inicializa a classe Aruco3d
    Arucos = Aruco3d()

    bgr = cv2.imread("img/aruco.png")
    #Chama a funcao detectaAruco
    bgr, results = Arucos.detectaAruco(bgr)

    print(results[0])

    cv2.imshow("Result_MobileNet", bgr)
    cv2.waitKey(0)