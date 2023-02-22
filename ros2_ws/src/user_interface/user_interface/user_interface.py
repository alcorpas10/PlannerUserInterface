import rclpy
from geometry_msgs.msg import Point

from task_point.point_action_client import PointActionClient
from homebase_serv_cli.homebase_client import HomebaseClient

from .action_client_class import ActionClientClass

class UserInterface():

    def __init__(self):
        print("Welcome to the user interface.")

        rclpy.init()

    def execute(self):
        while True:
            self.print_menu()
            self.read_input()

    def print_menu(self):
        print("Available commands:")
        print("point_task: send a point task to the drone")
        print("go_homebase: send a go_homebase task to the drone")
        print("abort: abort the go_homebase task")
        print("exit: exit the program")

    def read_input(self):
        cmd = input("Insert command: ")
        
        if cmd == 'point_task':
            self.alex()
        elif cmd == 'go_homebase' or cmd == 'abort':
            self.guille(order=cmd)
        #elif cmd == 'inspect':
            #self.previous()
        elif cmd == 'exit':
            print("Exiting program...")
            exit()
        else:
            print("Invalid input, please try again.")

    def alex(self):
        print('Insert the coordinates of the point:')

        x = float(input("Insert X coordinate: "))
        y = float(input("Insert Y coordinate: "))
        z = float(input("Insert Z coordinate: "))

        pt_action_client = PointActionClient()
        action = ActionClientClass(pt_action_client)
        action.daemon = True
        action.start(Point(x=x, y=y, z=z))

    def guille(self, order=None):
        hb_action_client = HomebaseClient()
        action = ActionClientClass(hb_action_client)
        action.daemon = True
        action.start(order)


def main(args=None):
    ui = UserInterface()
    ui.execute()

    # pt_action_client = PointActionClient()
    # hb_action_client = HomebaseClient()
    # ui = UserInterface(pt_action_client, hb_action_client)

    # executor = MultiThreadedExecutor()
    # executor.add_node(pt_action_client)
    # executor.add_node(hb_action_client)
    # executor.add_node(ui)

    # executor.spin()


if __name__ == '__main__':
    main()