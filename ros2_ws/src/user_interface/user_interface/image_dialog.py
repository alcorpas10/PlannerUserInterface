import numpy as np
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QDialog, QLabel, QHBoxLayout
#import matplotlib.pyplot as plt

class ImageDialog(QDialog):
    def __init__(self):
        super().__init__()

        # Crear un layout vertical para el diálogo
        self.setLayout(QHBoxLayout(self))

    def show_image(self, img_data):
        height, width, _ = img_data.shape

        # Crear un QLabel para mostrar la imagen
        self.image_label = QLabel()
        self.image_label.setPixmap(QPixmap.fromImage(QImage(img_data, width, height, 3 * width, QImage.Format_RGB888)))

        # Add the QLabel to the dialog layout 
        self.layout().addWidget(self.image_label)

        # Adjust the dialog size to the image size
        #self.image_label.adjustSize()

        self.show()