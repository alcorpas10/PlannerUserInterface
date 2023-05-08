import numpy as np
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QDialog, QLabel, QHBoxLayout


class ImageDialog(QDialog):
    def __init__(self):
        super().__init__()

        # Create a horizontal layout for the dialog
        self.setLayout(QHBoxLayout(self))

        # Set the dialog size
        self.setFixedSize(800, 250)

    def show_image(self, img_data):
        height, width, _ = img_data.shape

        # Create a QLabel to show the images
        self.image_label = QLabel()
        self.image_label.setPixmap(QPixmap.fromImage(QImage(img_data, width, height, 3 * width, QImage.Format_RGB888)))

        # Add the QLabel to the dialog layout 
        self.layout().addWidget(self.image_label)

        # Adjust the dialog size to the image size
        #self.image_label.adjustSize()

        self.show()