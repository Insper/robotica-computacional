#importando as bibliotecas utilizadas
import cv2.aruco as aruco
import cv2
import numpy as np
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import rospy
import os

#definindo a classe Aruco3d
class Aruco3d:
    def __init__(self):
            
            self.bridge = CvBridge()
            #Criando o callback do topico da camera do robo, para receber cada frame transmitido
            self.image_sub = rospy.Subscriber('/camera/image/compressed',CompressedImage,self.image_callback,queue_size=1,buff_size = 2**24)
            
            #Definindo centros_aruco para armazenar as coordenadas [x,y] do ponto central do aruco detectado
            self.centros_aruco = []
            #Definindo a variavel ids para armazenar o Id do aruco detectado
            self.ids = None 
            #Definindo a lista que vai armazenar os pontos contendo a distancia dos arucos detectados em relacao ao robo
            self.distancias = []

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
            self.camera_matrix   = np.loadtxt(calibra_path+'/cameraMatrix_realsense.txt', delimiter=',')
            self.camera_distortion   = np.loadtxt(calibra_path+'/cameraDistortion_realsense.txt', delimiter=',')

    def image_callback(self, msg: CompressedImage) -> None:
        #converte a msg do ROS para OpenCV 
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        #chama a função de leitura dos Arucos
        self.ids, self.centros_aruco, self.distancias =self.geraAruco(cv_image)
        #carrega uma copia da imagem ja decodificada para a variavel img
        img = cv_image.copy()      

    def geraAruco(self,cv_image):
        centros = []
        distancia_aruco=[]
        # Gera a mascara em escalas de Cinza apartir da copia da imagem la da função image_callback
        grayColor = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #Define o Dicionario para o Aruco que vamos utilizar
        dicionarioAruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        
        #Detecta os Arucos e Carrega os pontos de translação e de rotação do aruco em relação ao robo
        cornersList, ids, _ = aruco.detectMarkers(
            grayColor, dicionarioAruco)
        # Se um Id foi detectado, verifica se ele esta dentro da range de 0 a 99 e calcula os valores de rotação e translação 
        if ids is not None:
            for i in range(len(ids)):
                if ids[i]>99:
                    ret = aruco.estimatePoseSingleMarkers(cornersList[i], 19, self.camera_matrix, self.camera_distortion)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    distancia_aruco.append(np.linalg.norm(tvec))
                else: 
                    ret = aruco.estimatePoseSingleMarkers(cornersList[i], 6, self.camera_matrix, self.camera_distortion)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    distancia_aruco.append(np.linalg.norm(tvec))
                
            #Captura as coordenadas dos cantos do Aruco detectado
            for corners in cornersList:
                for corner in corners:
                    centros.append(np.mean(corner, axis=0))

            #-- Desenha um retanculo e exibe Id do marker encontrado
            aruco.drawDetectedMarkers(cv_image, cornersList, ids) 
            cv2.drawFrameAxes(cv_image, self.camera_matrix, self.camera_distortion, rvec, tvec ,0.03)
            #-- Exibe tvec --> vetor de tanslação em x y z
            str_position = "Marker x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            #-- Desenha na imagem o vetor de tanslação em x y z
            cv2.putText(cv_image, str_position, (0, 100), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1, cv2.LINE_AA)
            # Desenha a linha referencia em X
            cv2.line(cv_image, (cv_image.shape[1]//2,cv_image.shape[0]//2), ((cv_image.shape[1]//2 + 50),(cv_image.shape[0]//2)), (0,0,255), 5) 
            # Desenha a linha referencia em Y
            cv2.line(cv_image, (cv_image.shape[1]//2,cv_image.shape[0]//2), (cv_image.shape[1]//2,(cv_image.shape[0]//2 + 50)), (0,255,0), 5) 

            # Exibe a imagem na tela
            cv2.imshow("Camera", cv_image)
            cv2.waitKey(1)

        #retorna os ids e as coordenadas de centro e de distancia do aruco em relação ao robo
        return ids, centros, distancia_aruco


       
    def main(self):
        print('id {}\n\ncentro{}\n\ndistancia{}'.format(self.ids,self.centros_aruco,self.distancias))
            
                    
if __name__ == "__main__":
    #inicializa o node de conexao com o ROS
    rospy.init_node("Aruco")
    #inicializa a classe Aruco3d
    arucos = Aruco3d()

    
    while not rospy.is_shutdown():
        # Chama a função main da classe Aruco3d
        arucos.main()


