import rclpy
from geometry_msgs.msg import Point

from point_task.point_action_client import PointActionClient
from homebase_serv_cli.homebase_client import HomebaseClient

from .main_window import MainWindow
from .image_dialog import ImageDialog
from PySide6.QtCore import (QMetaObject, QRect, Signal)
from PySide6.QtWidgets import (QApplication, QTabBar, QMainWindow, QTextEdit, QWidget)

from PySide6.QtGui import QImage, QPixmap

import numpy as np

import sys

from .action_client_class import ActionClientClass


class UserInterface(MainWindow):
    signal_str = Signal(str)
    signal_img = Signal(np.ndarray)

    def __init__(self):
        MainWindow.__init__(self)
        self.action_list = []
        self.id = 0


    def setupUi(self, MainWindow):
        super().setupUi(MainWindow)
        
        self.signal_str.connect(self.update_text)
        self.signal_img.connect(self.show_image)
        self.pushButton.clicked.connect(self.read_input)
        #self.tabWidget.tabCloseRequested.connect(self.close_action_tab)

    def read_input(self):
        text = self.textEdit.toPlainText().split(' ')

        cmd = text[0]
        
        if cmd == 'point_task':
            self.point_task(text)
        elif cmd == 'go_homebase' or cmd == 'abort':
            self.homebase_task(cmd)
        #elif cmd == 'inspect':
            #self.previous()
        elif cmd == 'exit':
            print("Exiting...")
            exit()
        else:
            self.textEdit.append("Invalid input, please try again.")

    def update_text(self, text):
        id = text.split(' ')[0]
        self.action_list[int(id)].append(text[len(id)+1:])

    def show_image(self, img):
        dialog = ImageDialog()
        dialog.show()

        dialog.show_image(img)

    def close_action_tab(self, index):
        if index == 0:
            return
        self.action_list.pop(index-1)
        self.tabWidget.removeTab(index)

    def point_task(self, text):
        x = float(text[1])
        y = float(text[2])
        z = float(text[3])

        rclpy.init()
        pt_action_client = PointActionClient(self.id, self.signal_str, self.signal_img)

        self.new_action_tab()
        action = ActionClientClass(pt_action_client, Point(x=x, y=y, z=z))
        action.daemon = True
        action.start()

    def homebase_task(self, order):
        rclpy.init()
        hb_action_client = HomebaseClient(self.id, self.signal_str)

        self.new_action_tab()
        action = ActionClientClass(hb_action_client, order)
        action.daemon = True
        action.start()
    
    def new_action_tab(self):
        tab = QWidget()
        tab.setObjectName(u"tab_"+str(len(self.action_list)))

        textEdit = QTextEdit(tab)
        textEdit.setObjectName(u"textEdit_"+str(len(self.action_list)))
        textEdit.setGeometry(QRect(20, 20, 751, 481))
        textEdit.setReadOnly(True)

        self.tabWidget.addTab(tab, "")
        self.action_list.append(textEdit)
        self.tabWidget.tabBar().setTabButton(len(self.action_list)+1, QTabBar.RightSide, None)
        self.tabWidget.setTabText(self.tabWidget.indexOf(tab), "Action "+str(len(self.action_list)))
        self.tabWidget.setCurrentIndex(len(self.action_list))

        QMetaObject.connectSlotsByName(self.MainWindow)

        self.id += 1


def main(args=None):
    app = QApplication([])
    window = QMainWindow()
    ui = UserInterface()
    ui.setupUi(window)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()