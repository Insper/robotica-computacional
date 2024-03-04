import cv2
import numpy as np

class ImageModule():
    def __init__(self):
        self.kernel = None
    
    def color_filter(self, img: np.ndarray, lower: tuple, upper: tuple) -> np.ndarray:
        result = cv2.inRange(img, lower, upper)
        return result

    def run(self, img: np.ndarray, lower: tuple, upper: tuple) -> np.ndarray:
        result = self.color_filter(img, lower, upper)

        ## Pode fazer mais coisas aqui

        return result

def main():
    img = cv2.imread('image.jpg')
    image_module = ImageModule()
    lower = (0, 0, 0)
    upper = (255, 255, 255)
    result = image_module.run(img, lower, upper)
    cv2.imshow('result', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()