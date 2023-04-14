import cv2.aruco as aruco
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import rospy
import os

#documentação openv para o aruco:  https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

class aruco3d:
    def __init__(self):
            
            self.bridge = CvBridge()
            #topico da camera do robo
            self.image_sub = rospy.Subscriber("/camera/image/compressed",
                                                CompressedImage, 
                                                self.image_callback, 
                                                queue_size=4, 
                                                buff_size = 2**24)
            
            self.centro_aruco = 0 
            self.ids = None 
            self.dista = []

            #--- Get the camera calibration path
            calib_path  = os.path.abspath(os.getcwd())
            self.camera_matrix   = np.loadtxt(calib_path+'/arucopath/cameraMatrix_raspi.txt', delimiter=',')
            self.camera_distortion   = np.loadtxt(calib_path+'/arucopath/cameraDistortion_raspi.txt', delimiter=',')


    def image_callback(self, msg):

        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.centro_aruco, self.ids, self.dista = self.geraAruco(self.cv_image)
        print('Dã certo {}'.format(self.centro_aruco, self.ids, self.dista))
        cv2.imshow("Camera", self.cv_image)
        cv2.waitKey(1)


    def geraAruco(self,img):


        #converte a msg do ROS para OpenCV 
        # Criamos uma lista vazia chamada centros para armazenar os centros dos marcadores ArUco detectados.
        centros = []
        #Criamos outra lista vazia chamada dista para armazenar as distâncias calculadas a partir dos vetores de translação dos marcadores ArUco.
        dista=[]
        # Recebe o frame comprimido via ROS
#É realizada a conversão da imagem colorida de entrada (img) para uma imagem em tons de cinza utilizando a função cv2.cvtColor() da biblioteca OpenCV. 
        grayColor = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# É criado um dicionário contendo os marcadores ArUco pré-definidos, utilizando a função aruco.getPredefinedDictionary() da biblioteca OpenCV
        '''
        A função cv2.aruco.getPredefinedDictionary() é usada para criar um dicionário contendo os marcadores ArUco pré-definidos, que são 
        conjuntos de marcadores com IDs únicos e padrões específicos. Esses dicionários são usados para detecção e estimativa de posição dos 
        marcadores em imagens de entrada. O dicionário é criado com base em uma constante passada como parâmetro, que especifica o tipo de
        dicionário ArUco desejado.

        Exemplo: Por exemplo, no código abaixo, a linha dicionarioAruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) cria um
        dicionário ArUco pré-definido com 250 marcadores de 6x6 bits. 
        '''
        dicionarioAruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        
        '''
        cv2.aruco.detectMarkers(grayColor, dicionarioAruco): Essa função realiza a detecção dos marcadores ArUco em uma imagem em tons de
        cinza. O primeiro parâmetro, grayColor, é a imagem em tons de cinza onde a detecção será realizada. O segundo parâmetro, dicionarioAruco,
        é o dicionário ArUco pré-definido que será utilizado para a detecção. Essa função retorna três valores: cornersList, que é uma lista contendo
        os pontos dos cantos dos marcadores detectados; ids, que é uma lista contendo os IDs dos marcadores detectados; e _, que é uma variável descartada neste caso.
        '''

        cornersList, ids, _ = aruco.detectMarkers(
            grayColor, dicionarioAruco)
        
        '''
        Nesta etapa, é feita uma verificação se foram encontrados marcadores Aruco na imagem, verificando se a lista ids não está vazia.
        '''
        if ids is not None:
            print(ids)
            '''
            Fazemos um for nos ids detectados na lista ids, onde i é o índice do marcador atual.
            '''
            for i in range(len(ids)):
                '''É verificado se o ID do marcador atual é maior do que 99. Se for, é considerado um marcador grande e são utilizados 
                parâmetros diferentes para estimar sua pose.'''
                if ids[i]>99:
                    '''
                    A função cv2.aruco.estimatePoseSingleMarkers() é usada para estimar a pose (orientação e posição) de um marcador Aruco
                    individual a partir dos pontos dos cantos (corners) detectados desse Aruco na imagem. Vamos analisar os parâmetros dessa função:
                    - cornersList[i]: É a lista de pontos dos cantos (corners) do marcador Aruco. [[x1,y1],[x2,y2]]
                    -19: É o tamanho do lado do Aruco, ou seja, o comprimento do lado do quadrado preto que o compõe. 
                    Esse valor é fornecido manualmente, e é importante que seja o mesmo valor utilizado durante a calibração da câmera 
                    para obter uma estimativa de pose precisa.
                    -Camera_matrix: É a matriz de câmera que contém os parâmetros intrínsecos da câmera, como a distância focal e o 
                    centro óptico. Essa matriz é usada para corrigir as distorções da imagem e projetar os pontos 3D no espaço da câmera.
                    -Camera_distortion: É o vetor de distorção da câmera que contém os coeficientes de distorção radial e tangencial da câmera. 
                     Esse vetor é usado em conjunto com a matriz de câmera para corrigir as distorções da imagem.
                    '''
                    ret = aruco.estimatePoseSingleMarkers(cornersList[i], 19, self.camera_matrix, self.camera_distortion)
                    '''
                    O resultado da função cv2.aruco.estimatePoseSingleMarkers() é armazenado na variável ret, que é uma tupla contendo as 
                    informações de posição estimadas do Aruco. Essas informações incluem a matriz de rotação (rvec) e o vetor de 
                    translação (tvec) que representam a orientação e a posição do marcador no espaço 3D da câmera, respectivamente. 
                    Esses valores podem ser utilizados posteriormente para realizar a renderização de objetos virtuais ou para outros 
                    fins de aplicação. Mas a criterios de código, retorna uma tupla ret que contém duas informações: ret[0] é uma matriz 
                    de rotação 3x1 (rvec) e ret[1] é um vetor de translação 3x1 (tvec). 

                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] está pegando os valores de rotação (rvec) e translação (tvec)
                    Em ret[0][0,0,:] está acessando o valor da matriz de rotação (rvec) na posição [0,0,:] e ret[1][0,0,:] 
                    está acessando o valor do vetor de translação (tvec) na posição [0,0,:]. 
                    Obs: A notação ":" é usada para indicar todas as posições na última dimensão da matriz ou vetor, que no caso é 
                    a dimensão 3.                    
                    '''

                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    print(rvec,tvec)

                    '''
                    dista.append(np.linalg.norm(tvec)) está calculando a norma Euclidiana do vetor de translação tvec 
                    usando a função np.linalg.norm() e adicionando o resultado à lista dista usando o método append().

                    A função np.linalg.norm() é uma função da biblioteca NumPy  que calcula 
                    a norma Euclidiana de um vetor em um espaço n-dimensional. A norma Euclidiana é a medida da magnitude 
                    de um vetor, calculada como a raiz quadrada da soma dos quadrados dos componentes do vetor. 
                    ex: v = ai+bj , norma = sqrt(a²+b²)

                    tvec é um vetor de translação estimado para um Aruco na posição 3D da câmera.
                    A norma Euclidiana desse vetor representa a magnitude ou distância desse vetor em relação à origem do sistema de 
                    coordenadas 3D da câmera. Portanto, np.linalg.norm(tvec) está calculando a distância do aruco em relação 
                    à câmera no espaço 3D.

                    Depois esses valores são armazenados na lista de distancias (dista)
                    '''
                    dista.append(np.linalg.norm(tvec))



                else: 
                    #mesma ideia do que foi explicado no if se aplica aqui, porém para ids menores que 99
                    ret = aruco.estimatePoseSingleMarkers(cornersList[i], 6, self.camera_matrix, self.camera_distortion)
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    dista.append(np.linalg.norm(tvec))
                

            for corners in cornersList:
                for corner in corners:
                    centros.append(np.mean(corner, axis=0))


        return centros, ids, dista

    def control(self):
        print('centros{},id{},distancia{}'.format(self.centro_aruco,self.ids,self.dista))
if __name__ == "__main__":
    #inicializa o node de conexao com o ROS
    rospy.init_node("aruco")
    arucos = aruco3d()

    while not rospy.is_shutdown():
        arucos.control()