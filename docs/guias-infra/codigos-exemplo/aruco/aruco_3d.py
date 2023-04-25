import cv2.aruco as aruco
import cv2
import numpy as np
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import rospy
import os


class Aruco3d:
    def __init__(self):
            
            self.bridge = CvBridge()
            #topico da camera do robo
            self.image_sub = rospy.Subscriber('/camera/image/compressed',CompressedImage,self.image_callback,queue_size=1,buff_size = 2**24)
            
            # Publishers
            self.image_pub = rospy.Publisher("/image_publisher/", CompressedImage, queue_size=1)
            self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

            # Subscribers
            self.laser_subscriber = rospy.Subscriber('/scan',LaserScan, self.laser_callback)

            self.twist = Twist()
            self.centros_aruco = 0
            self.ids = None 
            self.distancias = 0


            calibra_path  = os.path.abspath(os.getcwd())
            self.camera_matrix   = np.loadtxt(calibra_path+'/arucopath/cameraMatrix_raspi.txt', delimiter=',')
            self.camera_distortion   = np.loadtxt(calibra_path+'/arucopath/cameraDistortion_raspi.txt', delimiter=',')



    def laser_callback(self, msg):
        leitura = np.array(msg.ranges).round(decimals=2)
        


    def image_callback(self, msg: CompressedImage) -> None:

        #converte a msg do ROS para OpenCV 
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        #chama a função de leitura dos Arucos
        self.ids, self.centros_aruco, self.distancias =self.geraAruco(cv_image)
        img = cv_image.copy()
      


    def geraAruco(self,cv_image):
        centros = []
        distancia_aruco=[]
        # Gera mask Cinza
        grayColor = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #Gera Dicionario com Arucos
        dicionarioAruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        #Detecta Arucos e Gera variaveis
    
        cornersList, ids, _ = aruco.detectMarkers(
            grayColor, dicionarioAruco)
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

            # Exibe tela
            cv2.imshow("Camera", cv_image)
            cv2.waitKey(1)

        return ids, centros, distancia_aruco


       
    def main(self):
        print('id {}\n\ncentro{}\n\ndistancia{}'.format(self.ids,self.centros_aruco,self.distancias))
            
                    
if __name__ == "__main__":
    #inicializa o node de conexao com o ROS
    rospy.init_node("Aruco")
    arucos = Aruco3d()

    while not rospy.is_shutdown():
        arucos.main()

