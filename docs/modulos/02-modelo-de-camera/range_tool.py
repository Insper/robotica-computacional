import cv2
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QSlider, QLabel, QVBoxLayout, QWidget, QPushButton, QRadioButton, QHBoxLayout, QFileDialog
from PyQt5.QtGui import QImage, QPixmap


class ImageTuner(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

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

        self.image_label = QLabel(self)
        self.mask_label = QLabel(self)

    def create_radio_buttons(self):
        rgb_button = QRadioButton("RGB", self)
        hsv_button = QRadioButton("HSV", self)

        # RGB selected as default
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
        ui_layout.addLayout(self.create_radio_buttons_layout())
        ui_layout.addLayout(self.create_sliders_layout())
        main_layout.addLayout(ui_layout)

        image_layout = QVBoxLayout()
        image_layout.addWidget(self.image_label)
        image_layout.addWidget(self.mask_label)
        main_layout.addLayout(image_layout)

        self.setLayout(main_layout)

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
        fileName, _ = QFileDialog.getOpenFileName(self, "QFileDialog.getOpenFileName()", "",
                                                  "All Files (*);;Image Files (*.jpg *.png)", options=options)
        if fileName:
            self.img = cv2.imread(fileName)
            self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
            self.show_image(self.img, self.image_label)

    def show_image(self, img, label):
        qformat = QImage.Format_Indexed8
        if len(img.shape) == 3:
            qformat = QImage.Format_RGBA8888 if img.shape[2] == 4 else QImage.Format_RGB888

        img = QImage(img, img.shape[1], img.shape[0], img.strides[0], qformat)
        img = img.rgbSwapped()
        label.setPixmap(QPixmap.fromImage(img))

    def update_image(self):
        if self.img is None:
            return

        low = np.array([self.sliders[i].value() for i in range(3)])
        high = np.array([self.sliders[i].value() for i in range(3, 6)])

        if self.rgb_button.isChecked():
            mask = cv2.inRange(self.img, low, high)
        else:
            mask = cv2.inRange(self.hsv_img, low, high)

        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        self.show_image(self.img, self.image_label)
        self.show_image(mask, self.mask_label)

    def update_slider_label(self, index, value):
        if self.rgb_button.isChecked():
            self.slider_labels[index].setText(f"{self.slider_names_rgb[index]}: {value}")
        else:
            self.slider_labels[index].setText(f"{self.slider_names_hsv[index]}: {value}")

    def update_ui(self):
        if self.rgb_button.isChecked():
            self.sliders[0].setRange(0, 255)
            self.sliders[3].setRange(0, 255)
            for i in range(6):
                self.slider_labels[i].setText(f"{self.slider_names_rgb[i]}: {self.sliders[i].value()}")
        else:
            self.sliders[0].setRange(0, 180)
            self.sliders[3].setRange(0, 180)
            for i in range(6):
                self.slider_labels[i].setText(f"{self.slider_names_hsv[i]}: {self.sliders[i].value()}")
        self.update_image()


def main():
    app = QApplication([])
    window = ImageTuner()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
