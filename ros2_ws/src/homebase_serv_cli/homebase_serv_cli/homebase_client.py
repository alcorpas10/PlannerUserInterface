import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_action_msgs.action import Homebase
from .action_window import ActionWindow

from PySide6.QtWidgets import QApplication

"""import sys
from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QTextEdit


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        # Crea el QTextEdit
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)

        # Agrega el QTextEdit al layout
        layout = QVBoxLayout()
        layout.addWidget(self.text_edit)
        self.setLayout(layout)

    def update_text(self, text):
        # Agrega el texto al QTextEdit
        self.text_edit.append(text)


class WindowThread(QThread):
    window_created = Signal(QWidget)

    def run(self):
        # Crea la ventana
        window = MainWindow()
        # Emite la señal para que se pueda acceder a la ventana creada
        self.window_created.emit(window)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Crea el hilo para la ventana
    window_thread = WindowThread()

    # Conecta la señal del hilo con la función que muestra la ventana
    window_thread.window_created.connect(lambda window: window.show())

    # Inicia el hilo
    window_thread.start()

    # Ejecuta la aplicación
    sys.exit(app.exec())"""

class HomebaseClient(Node):

    def __init__(self):
        super().__init__('homebase_action_client')
        self._action_client = ActionClient(self, Homebase, 'homebase')

        app = QApplication([])
        app.setApplicationName("Homebase client viewer")

        self.window = ActionWindow()
        self.window.show()

        app.exec()


    def send_goal(self, order):
        goal_msg = Homebase.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        # Cuando se manda la goal se define el callback para el feedback
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, self.feedback_callback)

        # Despues se especifica el callback para el resultado
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Goal aceptada o rechazada
    def goal_response_callback(self, future):
        goal_handle = future.result()
        # Si rechazada, return
        if not goal_handle.accepted:
            self.window.set_text('Goal rejected :(\n')
            return

        # Si aceptada, informar y pedir resultado
        self.window.set_text('Goal accepted :)\n')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Callback para cuando llegue el resultado
    def get_result_callback(self, future):
        result = future.result().result
        self.window.set_text('Result: ' + result.result + '\n')
        rclpy.shutdown()

    # Callback para cuando llegue el feedback
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.distance
        i = 0
        for p in feedback:
            self.window.set_text('DRON ' + str(i) + ' ' + str(p) + '\n')
            i = i + 1

def main(args=None):
    rclpy.init(args=args)

    action_client = HomebaseClient()

    order = input('Introduce una orden para el swarm \n')

    future = action_client.send_goal(order)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()