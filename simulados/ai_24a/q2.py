import numpy as np
import cv2

class DominoDetector:
    def __init__(self):
        self.kernel = np.ones((10, 10), np.uint8)
    
    def filter_contours(self, gray, lower, upper):
        mask = cv2.inRange(gray, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return sorted(contours, key=cv2.contourArea, reverse=True)
    
    def run(self, bgr):
        img_gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        contours = self.filter_contours(img_gray, 200, 255)[:2]
        # sort by y position
        contours = sorted(contours, key=lambda x: cv2.boundingRect(x)[1])

        value = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(bgr, (x, y), (x+w, y+h), (0, 255, 0), 2)
            # cut the domino
            domino = img_gray[y:y+h, x:x+w]
            contours = self.filter_contours(domino, 0, 100)

            count = 0
            for contour in contours:
                area = cv2.contourArea(contour) / (w * h)

                print('Area: ', area)
                if area > 0.02:
                    count += 1
            value.append(count)

        cv2.putText(bgr, f'{value[0]} por {value[1]}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return bgr

def main():
    detector = DominoDetector()

    # bgr = cv2.imread('domino.jpg')
    # output = detector.run(bgr)

    # cv2.imshow('output', output)
    # cv2.waitKey(0)

    cap = cv2.VideoCapture('dominoes.mp4')
    print('Press q to quit')

    while(cap.isOpened()):
        ret, bgr = cap.read()
        if not ret:
            break
        output = detector.run(bgr)
        cv2.imshow('output', output)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    main()