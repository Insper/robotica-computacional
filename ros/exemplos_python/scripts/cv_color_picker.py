import argparse
import cv2


class ImageCapture:
    def __init__(self, src):
        self.img = cv2.imread(src)

    def read(self):
        if self.img is None:
            return False, None
        return True, self.img.copy()

    def close(self):
        pass


def draw_circle(img, center):
    LAYERS = 4
    for i in range(LAYERS):
        c = 255 * (i % 2)
        cv2.circle(img, center, 10+i, (c, c, c))


def make_str(code, color):
    s = []
    for cd, cl in zip(list(code), color):
        s.append('{}: {}'.format(cd, cl))
    return ', '.join(s)


def cvtColor(color, transf):
    img = color.reshape((1, 1, -1))
    return cv2.cvtColor(img, transf).reshape((-1))


def print_colors(color_bgr):
    sep = '#' * 10
    print(sep)
    print(make_str('BGR', color_bgr))
    print(make_str('HSV', cvtColor(color_bgr, cv2.COLOR_BGR2HSV)))
    print('GRAY: {}'.format(cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)[0]))
    print(sep)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Color picker for OpenCV images')
    parser.add_argument('--image', help='image filename')
    parser.add_argument('--video', help='video filename', default=0)
    args = parser.parse_args()

    if args.image:
        cap = ImageCapture(args.image)
    else:
        cap = cv2.VideoCapture(args.video)

    mouse = None
    mouse_down = False
    # mouse callback function
    def update_mouse(event, x, y, flags, param):
        global mouse
        global mouse_down

        mouse = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_down = True
        elif event == cv2.EVENT_LBUTTONUP:
            mouse_down = False
            print_colors(frame[mouse[1], mouse[0]])

    winname = 'OpenCV Color Picker'
    cv2.namedWindow(winname)
    cv2.setMouseCallback(winname, update_mouse)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if mouse:
            draw_circle(frame, mouse)
        cv2.imshow(winname, frame)
        if cv2.waitKey(1) == 27:
            break
