import numpy as np
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QDialog, QLabel, QVBoxLayout

class ImageDialog(QDialog):
    def __init__(self):
        super().__init__()

        # Crear un layout vertical para el diálogo
        layout = QVBoxLayout(self)

        # Crear un QLabel para mostrar la imagen
        self.image_label = QLabel(self)
        layout.addWidget(self.image_label)

    def show_image(self, img_data):
        # Crear una instancia de QImage a partir de la matriz NumPy
        #qimg = QImage(img_data.data, img_data.shape[1], img_data.shape[0], img_data.strides[0], QImage.Format_RGB888)

        # Crear una instancia de QPixmap a partir de la QImage
        #pixmap = QPixmap.fromImage(qimg)

        # Mostrar la imagen en el QLabel
        #self.image_label.setPixmap(pixmap)
        height, width, _ = img_data.shape
        bytes_per_line = 3 * width
        qimg = QPixmap.fromImage(QImage(img_data.data, width, height, bytes_per_line, QImage.Format_RGB888))
        self.image_label.setPixmap(qimg)

        # Ajustar el tamaño del QLabel para que coincida con el tamaño de la imagen
        self.image_label.adjustSize()
