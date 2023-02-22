from PySide6.QtCore import (QCoreApplication, QMetaObject, QRect)
from PySide6.QtWidgets import (QMainWindow, QTextEdit, QVBoxLayout, QPlainTextEdit, QScrollArea, QStatusBar, QWidget)


class ActionWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Crea el QTextEdit
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)

        # Agrega el QTextEdit al layout
        layout = QVBoxLayout()
        layout.addWidget(self.text_edit)
        self.setLayout(layout)
    """def __init__(self):
        QMainWindow.__init__(self)
        if not self.objectName():
            self.setObjectName(u"MainWindow")
        self.resize(800, 600)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName(u"centralwidget")
        self.scrollArea = QScrollArea(self.centralwidget)
        self.scrollArea.setObjectName(u"scrollArea")
        self.scrollArea.setGeometry(QRect(-1, -1, 801, 601))
        self.scrollArea.setWidgetResizable(True)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setObjectName(u"scrollAreaWidgetContents")
        self.scrollAreaWidgetContents.setGeometry(QRect(0, 0, 799, 599))
        self.plainTextEdit = QPlainTextEdit(self.scrollAreaWidgetContents)
        self.plainTextEdit.setObjectName(u"plainTextEdit")
        self.plainTextEdit.setGeometry(QRect(3, 9, 791, 571))
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(self)
        self.statusbar.setObjectName(u"statusbar")
        self.setStatusBar(self.statusbar)

        self.retranslateUi()

        #QMetaObject.connectSlotsByName()
    # setupUi

    def retranslateUi(self):
        self.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
    # retranslateUi

    def set_text(self, text):
        self.plainTextEdit.setPlainText(self.plainTextEdit.toPlainText() + text)"""

    def update_text(self, text):
        # Agrega el texto al QTextEdit
        self.text_edit.append(text)