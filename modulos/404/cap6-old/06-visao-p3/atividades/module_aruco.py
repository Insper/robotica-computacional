import cv2
import cv2.aruco as aruco
import numpy as np
import os

class Aruco3d():
    def __init__(self, camera_matrix=None, camera_distortion=None):
        self.ids = None
        calibra_path  = os.path.dirname(os.path.abspath(__file__))

        if camera_matrix is None:
            self.camera_matrix = np.loadtxt(calibra_path+'/config/cameraMatrix_realsense.txt', delimiter=',')
            self.camera_distortion = np.loadtxt(calibra_path+'/config/cameraDistortion_realsense.txt', delimiter=',')
        else:
            self.camera_matrix = np.loadtxt(camera_matrix, delimiter=',')
            self.camera_distortion = np.loadtxt(camera_distortion, delimiter=',')

    def detectaAruco(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        dicionarioAruco = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dicionarioAruco, parameters)
        cornersList, ids, _ = detector.detectMarkers(gray)

        results = []
        if ids is not None:
            for i in range(len(ids)):
                results.append({
                    'id': ids[i],
                    'corners': cornersList[i],
                    'centro': np.mean(cornersList[i], axis=1).astype("int").flatten()
                })

        return bgr, results

    def drawAruco(self, bgr, result):
        cv2.line(bgr, (bgr.shape[1]//2, bgr.shape[0]//2), ((bgr.shape[1]//2 + 50), (bgr.shape[0]//2)), (0,0,255), 5)
        cv2.line(bgr, (bgr.shape[1]//2, bgr.shape[0]//2), (bgr.shape[1]//2, (bgr.shape[0]//2 + 50)), (0,255,0), 5)
        aruco.drawDetectedMarkers(bgr, np.array([result['corners']]), np.array([result['id']]))
        return bgr

    def writeDistance(self, bgr, distancia):
        cv2.putText(bgr, f"Distancia: {distancia:.2f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return bgr

def main():
    Arucos = Aruco3d()
    bgr = cv2.imread("img/aruco.png")
    bgr, results = Arucos.detectaAruco(bgr)
    for result in results:
        bgr = Arucos.drawAruco(bgr, result)
    print(results[0])
    cv2.imshow("Aruco", bgr)
    cv2.waitKey(0)

if __name__ == "__main__":
    main()
