import sys
from PyQt6.QtWidgets import QApplication
from rclpy.node import Node
import rclpy

from turtlesim_py import turtle_frame

class TurtleApp(QApplication):
    def __init__(self, argv) -> None:
        super().__init__(argv)
        rclpy.init(args=argv)
        self.nh_ = Node("turtlesim")
    
    def exec(self):
        frame = turtle_frame.TurtleFrame(self.nh_)
        frame.show()
        return QApplication.exec()
    
def main():
    app = TurtleApp(sys.argv)
    sys.exit(app.exec())
