import cv2
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSignal, QObject
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QSlider, QLabel, QVBoxLayout, QWidget, QPushButton, QRadioButton, QHBoxLayout, QFileDialog, QFrame, QCheckBox

class ImageModule():
    def __init__(self):
        self.kernel = None
    
    def color_filter(self, img: np.ndarray, lower: tuple, upper: tuple) -> np.ndarray:
        result = cv2.inRange(img, lower, upper)
        return result

    def run(self, img: np.ndarray, lower: tuple, upper: tuple) -> np.ndarray:
        result = self.color_filter(img, lower, upper)

        # Convert to 
        return result

class ImageUpdateSignal(QObject):
    # Define a custom signal
    signal = pyqtSignal(np.ndarray)

class ImageTuner(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.image_module = ImageModule()
        self.image_size = 400
        self.bridge = CvBridge()
        self.image_update_signal = ImageUpdateSignal()
        self.image_update_signal.signal.connect(self.update_image_from_ros_slot)

    def closeEvent(self, event):
        cv2.destroyAllWindows()
        event.accept()

    def init_ui(self):
        self.init_variables()
        self.init_widgets()
        self.set_layouts()

    def init_variables(self):
        self.img = None
        self.hsv_img = None
        self.slider_names_rgb = ['Rmin', 'Gmin', 'Bmin', 'Rmax', 'Gmax', 'Bmax']
        self.slider_names_hsv = ['Hmin', 'Smin', 'Vmin', 'Hmax', 'Smax', 'Vmax']

    def init_widgets(self):
        self.open_button = QPushButton('Open Image', self)
        self.open_button.clicked.connect(self.open_image)

        self.rgb_button, self.hsv_button = self.create_radio_buttons()
        self.sliders, self.slider_labels = self.create_sliders_and_labels()

    def create_radio_buttons(self):
        rgb_button = QRadioButton("RGB", self)
        hsv_button = QRadioButton("HSV", self)
        rgb_button.setChecked(True)
        rgb_button.toggled.connect(self.update_ui)
        hsv_button.toggled.connect(self.update_ui)
        return rgb_button, hsv_button

    def create_sliders_and_labels(self):
        sliders, slider_labels = [], []
        for i in range(6):
            slider = QSlider(Qt.Horizontal, self)
            slider.setRange(0, 255)
            if i > 2:
                slider.setValue(255)
            slider.valueChanged.connect(self.update_image)
            slider.valueChanged.connect(self._make_slider_callback(i))
            sliders.append(slider)

            slider_label = QLabel(self)
            slider_label.setText(f"{self.slider_names_rgb[i]}: {slider.value()}")
            slider_labels.append(slider_label)
        return sliders, slider_labels

    def set_layouts(self):
        main_layout = QHBoxLayout()

        ui_layout = QVBoxLayout()
        ui_layout.addWidget(self.open_button)
        ui_layout.addLayout(self.create_radio_rossub_layout())
        ui_layout.addLayout(self.create_radio_buttons_layout())
        ui_layout.addLayout(self.create_sliders_layout())
        ui_layout.addWidget(self.create_horizontal_line())

        main_layout.addLayout(ui_layout)
        main_layout.addWidget(self.create_vertical_line())

        self.setLayout(main_layout)

    def create_horizontal_line(self):
        horizontal_line = QFrame()
        horizontal_line.setFrameShape(QFrame.HLine)
        horizontal_line.setFrameShadow(QFrame.Sunken)
        return horizontal_line

    def create_vertical_line(self):
        vertical_line = QFrame()
        vertical_line.setFrameShape(QFrame.VLine)
        vertical_line.setFrameShadow(QFrame.Sunken)
        return vertical_line

    def create_radio_rossub_layout(self):
        rossub_layout = QHBoxLayout()
        self.use_rossub = QCheckBox("Use Subscribers", self)
        rossub_layout.addWidget(self.use_rossub)
        return rossub_layout
    
    def create_radio_buttons_layout(self):
        radio_layout = QHBoxLayout()
        radio_layout.addWidget(self.rgb_button)
        radio_layout.addWidget(self.hsv_button)
        return radio_layout

    def create_sliders_layout(self):
        sliders_layout = QVBoxLayout()
        for i in range(6):
            slider_layout = QVBoxLayout()
            slider_layout.addWidget(self.slider_labels[i])
            slider_layout.addWidget(self.sliders[i])
            sliders_layout.addLayout(slider_layout)
        return sliders_layout

    def _make_slider_callback(self, i):
        return lambda value: self.update_slider_label(i, value)

    def open_image(self):
        options = QFileDialog.Options()
        fileName, _ = QFileDialog.getOpenFileName(self, "Open Image", "", "Image Files (*.jpg *.jpeg *.png *.bmp)", options=options)
        if fileName:
            self.img = cv2.imread(fileName)
            if self.img.shape[0] > self.image_size or self.img.shape[1] > self.image_size:
                scale = self.image_size / max(self.img.shape[:2])
                self.img = cv2.resize(self.img, None, fx=scale, fy=scale)
            self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
            self.show_image(self.img, "Original Image")
        self.update_ui()

    def show_image(self, img, window_name):
        cv2.imshow(window_name, img)
        cv2.waitKey(1)

    def update_image(self):
        if self.img is None:
            return
        low = np.array([self.sliders[i].value() for i in range(3)])[[2, 1, 0]]
        high = np.array([self.sliders[i].value() for i in range(3, 6)])[[2, 1, 0]]
        if self.rgb_button.isChecked():
            result = self.image_module.run(self.img, low, high)
        else:
            result = self.image_module.run(self.hsv_img, low, high)
        self.show_image(result, "Filtered Image")

    def update_slider_label(self, index, value):
        if self.rgb_button.isChecked():
            self.slider_labels[index].setText(f"{self.slider_names_rgb[index]}: {value}")
        else:
            self.slider_labels[index].setText(f"{self.slider_names_hsv[index]}: {value}")

    def update_ui(self):
        is_rgb = self.rgb_button.isChecked()
        for i in range(3):
            if is_rgb or i != 0:
                self.sliders[i].setRange(0, 255)
                self.sliders[i+3].setRange(0, 255)
            else:
                self.sliders[i].setRange(0, 180)
                self.sliders[i+3].setRange(0, 180)
            self.slider_labels[i].setText(f"{self.slider_names_rgb[i] if is_rgb else self.slider_names_hsv[i]}: {self.sliders[i].value()}")
            self.slider_labels[i+3].setText(f"{self.slider_names_rgb[i+3] if is_rgb else self.slider_names_hsv[i+3]}: {self.sliders[i+3].value()}")
        self.update_image()

    def update_image_from_ros_slot(self, cv_image):
        """
        Update the displayed image from a ROS callback only if use_rossub is checked.
        """
        if not self.use_rossub.isChecked():
            return  # Do not update if ROS subscriber usage is not selected

        self.img = cv_image
        if self.img.shape[0] > self.image_size or self.img.shape[1] > self.image_size:
            scale = self.image_size / max(self.img.shape[:2])
            self.img = cv2.resize(self.img, None, fx=scale, fy=scale)
        self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.show_image(self.img, "Original Image")
        self.update_ui()

def main():
    app = QApplication([])
    window = ImageTuner()
    window.show()
    app.exec_()

if __name__ == "__main__":
    main()
