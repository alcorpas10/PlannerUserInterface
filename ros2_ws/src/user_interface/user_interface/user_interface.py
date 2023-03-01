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
        self.action_dict = {}
        self.ids_list = []
        self.id = 0
        self.dialog = ImageDialog()


    def setupUi(self, MainWindow):
        super().setupUi(MainWindow)
        
        self.signal_str.connect(self.update_text)
        self.signal_img.connect(self.show_image)
        self.pushButton.clicked.connect(self.read_input)
        self.tabWidget.tabCloseRequested.connect(self.close_action_tab)

    def read_input(self):
        text = self.textEdit.toPlainText().split(' ')

        cmd = text[0]
        
        if cmd == 'point_task' or cmd == 'cancel':
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
        self.action_dict[int(id)].append(text[len(id)+1:])

    def show_image(self, img):
        self.dialog.show_image(img)

    def close_action_tab(self, index):
        print("Closing tab: " + str(index))
        if index == 0:
            return
        print("Action list: " + str(self.action_dict.keys()))
        self.action_dict.pop(self.ids_list[index-1])
        self.ids_list.pop(index-1)
        print("Action list: " + str(self.action_dict.keys()))
        self.tabWidget.removeTab(index)

    def point_task(self, text):
        """if text == 'cancel':
            self.action_client.send_goal(self.input)
            self.signal_str.emit("cancel")
            return"""
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
        tab.setObjectName(u"tab_"+str(self.id))

        textEdit = QTextEdit(tab)
        textEdit.setObjectName(u"textEdit_"+str(self.id))
        textEdit.setGeometry(QRect(20, 20, 751, 481))
        textEdit.setReadOnly(True)

        self.tabWidget.addTab(tab, "")
        self.action_dict[self.id] = textEdit
        self.ids_list.append(self.id)
        self.tabWidget.tabBar().setTabButton(len(self.action_dict)+1, QTabBar.RightSide, None)
        self.tabWidget.setTabText(self.tabWidget.indexOf(tab), "Action "+str(self.id+1))
        self.tabWidget.setCurrentIndex(len(self.action_dict))

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