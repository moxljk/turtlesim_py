from turtlesim_py.turtle import Turtle
import rclpy
from rclpy.node import Node

from PyQt6.QtWidgets import QWidget, QFrame
from PyQt6.QtGui import QImage, QPaintEvent, QPainter, qRgb
from PyQt6.QtCore import Qt, QTimer, QPointF, QPoint

import math
import random
import os
from turtlesim_py.path_manager import IMAGES_DIR


DEFAULT_BG_R = 0x45
DEFAULT_BG_G = 0x56
DEFAULT_BG_B = 0xff

class TurtleFrame(QFrame):
    def __init__(self, nh:Node) -> None:
        super().__init__()

        # 类属性
        self.nh_           :Node              = nh
        self.update_timer_ :QTimer            = QTimer(self)
        self.path_image_   :QImage            = QImage(500, 500, QImage.Format.Format_ARGB32)
        self.path_painter_ :QPainter          = QPainter(self.path_image_)  # 好像是背景，但不知道为什么要叫“path image”
        self.frame_count_  :int               = (0)
        self.turtles_      :dict[str,Turtle]  = {}  # 乌龟表，由乌龟名索引乌龟对象实例。
        self.id_counter_   :int               = 0
        self.turtle_images_:list[QImage]      = []
        # self.meter_           :float
        # self.width_in_meters_ :float
        # self.height_in_meters_:float
        
        # 初始化UI
        self.setFixedSize(500, 500)
        self.setWindowTitle("Turtlesim")

        # 设置QT计时器
        self.update_timer_.setInterval(16)
        self.update_timer_.start()
        self.update_timer_.timeout.connect(self.__onUpdate)
        

        # 载入乌龟图像
        for ImagePath in os.listdir(IMAGES_DIR):
            img:QImage = QImage(IMAGES_DIR + ImagePath)
            self.turtle_images_.append(img)
        

        # 设置meter（我不知道这是什么东西，可能是一个比例单位？（在turtlesim世界中的一米长））
        self.meter_ = self.turtle_images_[0].height()

        self.__clear()
        
        self.width_in_meters_ = (self.width() - 1) / self.meter_
        self.height_in_meters_ = (self.height() - 1) / self.meter_
        self.spawnTurtle(self.width_in_meters_/2.0, self.height_in_meters_/2.0, 0)
        
        # 如果把False改成True，则重新运行可以看到所有的乌龟（好像只有七个位置）
        if False:
            for index in range(len(self.turtle_images_)):
                self.spawnTurtle(1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), math.pi / 2.0, index=index)
                
        
    def spawnTurtle(self, x:float, y:float, angle:float, name:str = "", index:int=None) -> str:
        """ 以指定的姿态产生一只乌龟并返回新产生的乌龟名。若指定的乌龟名已被使用则返回空字符串，不产生新乌龟。

        Args:
            x (float): 产生的乌龟的横坐标，以self.meter_为单位
            y (float): 产生的乌龟的纵坐标，以self.meter_为单位
            angle (float): 产生的乌龟的角度
            name (str): 产生的乌龟的名字。 默认传入空字符串，产生的乌龟名字为“turtle”加一个数字
            index (int, optional): 乌龟照片序号，用于指定产生的乌龟的照片。默认传入None，在给定照片中随机产生。

        Returns:
            str: 乌龟名称或空字符串
        """
        if index is None:
            index = random.randint(0, len(self.turtle_images_) - 1)
        real_name = name
        if not real_name:
            # 如果指定乌龟名为空，则给一个默认值。
            while True:
                self.id_counter_ += 1
                real_name = "turtle" + str(self.id_counter_)
                if not self.__has_turtle(real_name):
                    break
        else:
            # 如果已指定了一个名字且已经有这只乌龟了，则无需产生新乌龟，返回空字符串。
            if self.__has_turtle(real_name):
                return ""
        
        # 使用名称产生一只乌龟
        t = Turtle(self.nh_, real_name, self.turtle_images_[index], QPointF(x, self.height_in_meters_ - y), angle)
        self.turtles_[real_name] = t
        self.update()

        return real_name

    def paintEvent(self, event:QPaintEvent) -> None:
        """ 重写qt paintEvent方法
            1. 绘制背景图。
            2. 绘制所有乌龟（由乌龟自己负责绘制）。
        """
        painter = QPainter(self)
        painter.drawImage(QPoint(0, 0), self.path_image_)
        
        for turtle in self.turtles_.values():
            turtle.paint(painter)
    
    def __onUpdate(self) -> None:
        # todo: 不太明白这玩意是啥
        rclpy.spin_once(self.nh_, timeout_sec = 0.01)
        self.__updateTurtles()
        if not rclpy.ok():
            self.close()
    
    def __updateTurtles(self):
        """更新名单内每一只乌龟
        """
        modified:bool = False
        for turtle in self.turtles_.values():
            modified |= turtle.update(0.001 * self.update_timer_.interval(), self.path_painter_, self.path_image_, self.width_in_meters_, self.height_in_meters_)
        if modified:
            self.update()
            print("frame updates!")
        self.frame_count_ += 1
    
    def __clear(self) -> None:
        """ 使用设定值填满背景（即清屏）
        """
        self.path_image_.fill(qRgb(DEFAULT_BG_R,
                                   DEFAULT_BG_G,
                                   DEFAULT_BG_G)
                              )
        self.update()
    
    def __has_turtle(self, name:str) -> bool:
        """ 检查名单中是否存在某名字的乌龟

        Args:
            name (str): 要检查的乌龟名。

        Returns:
            bool: 是否存在
        """
        return name in self.turtles_.keys()