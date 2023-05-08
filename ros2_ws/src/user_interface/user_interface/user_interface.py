import rclpy
from rclpy import Node
from mutac_msgs.msg import Generation, Sweep
from mutac_msgs.srv import GeneratePlan

from geometry_msgs.msg import Point, Point32, Polygon

# from point_task.point_action_client import PointActionClient
# from homebase_serv_cli.homebase_client import HomebaseClient

from user_interface.main_window import MainWindow
from user_interface.image_dialog import ImageDialog
from user_interface.inspect_node import InspectNode
from user_interface.info_node import InfoNode
from PySide6.QtCore import (QMetaObject, QRect, Signal)
from PySide6.QtWidgets import (QApplication, QTabBar, QMainWindow, QTextEdit, QWidget)

import numpy as np

import sys


class UserInterface(MainWindow):
    signal_str = Signal(str)
    signal_img = Signal(np.ndarray)

    def __init__(self):
        MainWindow.__init__(self)
        self.action_tab_dict = {}
        self.action_client_dict = {}
        self.ids_list = []
        self.id = 0
        self.dialog = ImageDialog()

        rclpy.init()


    def setupUi(self, MainWindow):
        super().setupUi(MainWindow)
        
        self.signal_str.connect(self.update_text)
        # self.signal_img.connect(self.show_image)
        self.pushButton.clicked.connect(self.read_input)
        self.tabWidget.tabCloseRequested.connect(self.close_action_tab)

    def read_input(self):
        text = self.textEdit.toPlainText().split(' ')

        cmd = text[0]
        
        if cmd == 'inspect':
            self.inspect()
        elif cmd == 'swarm_state':
            self.swarm_state(text[1])
        # if cmd == 'point_task':
        #     self.point_task(text)
        # elif cmd == 'go_homebase':
        #     self.homebase_task(cmd)
        # elif cmd == 'cancel_mission':
        #     self.cancel_tasks(text[1])
        # elif cmd == 'cancel':
        #     self.cancel_tasks()
        #elif cmd == 'inspect':
            #self.previous()
        elif cmd == 'exit':
            print("Exiting...")
            exit()
        else:
            self.textEdit.append("Invalid input, please try again.")

    def update_text(self, text):
        id = text.split(' ')[0]
        if int(id) in self.ids_list:
            self.action_tab_dict[int(id)].append(text[len(id)+1:])

    # def show_image(self, img):
    #     self.dialog.show_image(img)

    def close_action_tab(self, index):
        print("Closing tab: " + str(index))
        if index == 0:
            return
        print("Action list: " + str(self.action_tab_dict.keys()))
        self.action_tab_dict.pop(self.ids_list[index-1])
        self.ids_list.pop(index-1)
        print("Action list: " + str(self.action_tab_dict.keys()))
        self.tabWidget.removeTab(index)

    def inspect(self):
        polygon1 = Polygon(points=[
            Point32(x=-2.0 , y= 2.0, z=0.0),
            Point32(x=-0.5 , y= 1.0, z=0.0),
            Point32(x=-0.5 , y=-2.0, z=0.0),
            Point32(x=-2.0 , y=-2.0, z=0.0)])
        polygon2 = Polygon(points=[
            Point32(x= 0.0 , y= 2.0, z=0.0),
            Point32(x= 2.0 , y= 2.0, z=0.0),
            Point32(x= 2.0 , y=-1.0, z=0.0),
            Point32(x= 0.0 , y= 0.0, z=0.0)])

        sweeps = [
            Sweep(polygon=polygon1,
                orientation=Point(x=0.0, y=-0.5, z=0.0)),
            Sweep(polygon=polygon2,
                orientation=Point(x=0.0, y=-0.5, z=0.0))]

        generation = Generation(sweeps=sweeps)

        genPlan = GeneratePlan.Request(generation=generation)

        inspect_node = InspectNode(self.id, self.signal_str, genPlan)
        self.action_client_dict[self.id] = inspect_node
        
        self.new_action_tab()
        inspect_node.start()

    def swarm_state(self, text):
        info_node = InfoNode(self.id, self.signal_str, text)
        self.action_client_dict[self.id] = info_node

        self.new_action_tab()
        info_node.start()


    # def point_task(self, text):
    #     x = float(text[1])
    #     y = float(text[2])
    #     z = float(text[3])

    #     pt_action_client = PointActionClient(self.id, self.signal_str, self.signal_img, Point(x=x, y=y, z=z))
    #     self.action_client_dict[self.id] = pt_action_client

    #     self.new_action_tab()
    #     pt_action_client.start()

    # def homebase_task(self, order):
    #     hb_action_client = HomebaseClient(self.id, self.signal_str, order)
    #     self.action_client_dict[self.id] = hb_action_client

    #     self.new_action_tab()
    #     hb_action_client.start()

    # def cancel_tasks(self, id=None):
    #     if id is None:
    #         for key in self.action_client_dict.keys():
    #             self.action_client_dict[key].cancel_goal()
    #     else:
    #         self.action_client_dict[int(id)].cancel_goal()
    
    def new_action_tab(self):
        tab = QWidget()
        tab.setObjectName(u"tab_"+str(self.id))

        textEdit = QTextEdit(tab)
        textEdit.setObjectName(u"textEdit_"+str(self.id))
        textEdit.setGeometry(QRect(20, 20, 751, 481))
        textEdit.setReadOnly(True)

        self.tabWidget.addTab(tab, "")
        self.action_tab_dict[self.id] = textEdit
        self.ids_list.append(self.id)
        self.tabWidget.tabBar().setTabButton(len(self.action_tab_dict)+1, QTabBar.RightSide, None)
        self.tabWidget.setTabText(self.tabWidget.indexOf(tab), "Action "+str(self.id+1))
        self.tabWidget.setCurrentIndex(len(self.action_tab_dict))

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