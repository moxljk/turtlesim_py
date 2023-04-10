from rclpy.node import Node
from turtlesim_interfaces.msg import Pose, Color
from turtlesim_interfaces.srv import SetPen
from geometry_msgs.msg import Twist

from PyQt6.QtGui import QImage, QColor, QPen, QTransform, QPainter, qRed, qGreen, qBlue
from PyQt6.QtCore import QPointF

from datetime import datetime, timedelta
import math

DEFAULT_PEN_R = 0xb3
DEFAULT_PEN_G = 0xb8
DEFAULT_PEN_B = 0xff

class Turtle:
    def __init__(self, nh:Node, name:str, turtle_image:QImage, pos:QPointF, orient:float) -> None:
        self.nh_:Node = nh                          # ROS2 结点
        self.name_:str = name                       # 乌龟名字
        self.turtle_image_:QImage = turtle_image    # 乌龟图片
        self.pos_:QPointF = pos                     # 乌龟的位置
        self.orient_:float = orient                 # 乌龟的方向
        self.lin_vel_:float = (0.0)                 # 乌龟的线速度
        self.ang_vel_:float = (0.0)                 # 乌龟的角速度
        self.pen_on_:bool = True                    # 画笔是否开启
        self.pen_ = QPen(QColor(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))  # 画笔
        self.last_command_time_ = datetime.now()
        
        self.pen_.setWidth(3)

        # 速度信息 subscriber
        self.velocity_sub_ = self.nh_.create_subscription(Twist,
                                                          self.name_+"/cmd_vel",
                                                          callback = self.__velocity_sub_callback,
                                                          qos_profile = 0
                                                        )
        # 姿态信息 publisher
        self.pose_pub_ = self.nh_.create_publisher(Pose, "pose", 1)
        # 颜色信息 publisher
        self.color_pub_ = self.nh_.create_publisher(Color, "color_sensor", 1)
        self.nh_.create_service(SetPen, "set_pen", self.__set_pen_serv_callback)
        
        self.meter_ = self.turtle_image_.height()
        self.__rotateImage()

        self.teleport_requests_:list[Turtle.__TeleportRequest] = []
    # public:

    def update(self, dt:float, path_painter:QPainter, path_image:QImage, canvas_width:float, canvas_height:float) -> bool:
        """ 进行乌龟的更新。

        Args:
            dt (float): δt,用来与速度相乘得到总位移的时间
            path_painter (QPainter): 用于绘图的QPainter
            path_image (QImage): 背景图像，用于提取当前乌龟位置的颜色
            canvas_width (float): 场地宽度，用于限制乌龟的运动范围
            canvas_height (float): 场地高度，用于限制乌龟的运动范围

        Returns:
            bool: _description_
        """
        modified = False # 记录是否有新的变化
        old_orient = self.orient_
        
        # 从列表里获得运动的控制发布。
        for req in self.teleport_requests_:
            # 分为相对运动和绝对运动。
            if req.relative:
                self.orient_ += req.theta
                self.pos_.setX(self.pos_.x() + math.sin(self.orient_ + math.pi/2.0) * req.linear)
                self.pos_.setY(self.pos_.y() + math.cos(self.orient_ + math.pi/2.0) * req.linear)
            else:
                self.pos_.setX(req.pos[0])
                self.pos_.setY(max(0.0, float(canvas_height - req.pos[1])))
                self.orient_ = req.theta

            if self.pen_on_:
                path_painter.setPen(self.pen_)
                path_painter.drawLine(self.pos_ * self.meter_, old_pos * self.meter_)
            modified = True
        
        self.teleport_requests_.clear()
        
        # 如果当前的速度设置时间超过了500ms，则停止（？）
        if datetime.now() - self.last_command_time_ > timedelta(milliseconds=1000):
        # if True:
            self.lin_vel_ = 0.0
            self.ang_vel_ = 0.0
        
        old_pos = QPointF(self.pos_)
        # 设置移动
        self.orient_ = math.fmod(self.orient_ + self.ang_vel_ * dt, 2*math.pi)
        self.pos_.setX(self.pos_.x() + math.sin(self.orient_ + math.pi/2.0) * self.lin_vel_ * dt)
        self.pos_.setY(self.pos_.y() + math.cos(self.orient_ + math.pi/2.0) * self.lin_vel_ * dt)
        
        print("x:",math.sin(self.orient_ + math.pi/2.0) * self.lin_vel_ * dt)
        print("y:",math.cos(self.orient_ + math.pi/2.0) * self.lin_vel_ * dt)
        print(self.last_command_time_)
        print(datetime.now())
        # 控制移动范围
        if (self.pos_.x() < 0 or self.pos_.x() > canvas_width or self.pos_.y() < 0 or self.pos_.y() > canvas_height):
            print("Oh no! I hit the wall!")
        
        self.pos_.setX(min(max(self.pos_.x(), 0.0), canvas_width))
        self.pos_.setY(min(max(self.pos_.y(), 0.0), canvas_height))
        
        # Publish 位置信息
        p = Pose()
        p.x = self.pos_.x()
        p.y = canvas_height - self.pos_.y()
        p.theta = self.orient_
        p.linear_velocity = self.lin_vel_
        p.angular_velocity = self.ang_vel_
        self.pose_pub_.publish(p)
    
        # Figure out (and publish) the color underneath the turtle
        color = Color()
        pixel = path_image.pixel(QPointF(self.pos_ * self.meter_).toPoint())
        color.r = qRed(pixel)
        color.g = qGreen(pixel)
        color.b = qBlue(pixel)
        self.color_pub_.publish(color)
        
        # 如果角度变了就要重新设置一下图片旋转了。
        if self.orient_ != old_orient:
            self.__rotateImage()
            modified = True
        # 如果位置变了就要重新绘制一下轨迹了。
        if self.pos_ != old_pos:
            if self.pen_on_:
                path_painter.setPen(self.pen_)
                path_painter.drawLine(self.pos_ * self.meter_, old_pos * self.meter_)
            modified = True
        return modified
    
    def paint(self, painter:QPainter):
        """绘制乌龟。
        """
        p:QPointF = self.pos_ * self.meter_
        p.setX(p.x() - 0.5 * self.turtle_rotated_image_.width())
        p.setY(p.y() - 0.5 * self.turtle_rotated_image_.height())
        painter.drawImage(p, self.turtle_rotated_image_)

    # private:

    def __velocity_sub_callback(self, vel: Twist):
        """`cmd_vel`话题的订阅回调
        """
        self.last_command_time_ = datetime.now()
        self.lin_vel_ = vel.linear.x
        self.ang_vel_ = vel.angular.z
    
    def __set_pen_serv_callback(self, req:SetPen.Request, res:SetPen.Response):
        """`set_pen`服务的回调函数，根据req中的信息设置画笔的颜色、宽度、开关。
        """
        self.pen_on_ = not req.off
        if req.off: return True
        
        pen = QPen(QColor(req.r, req.g, req.b))
        if req.width != 0:
            pen.setWidth(req.width)

        self.pen_ = pen
        return True
    
    def __rotateImage(self):
        """用于处理乌龟图片的旋转
        """
        transform = QTransform().rotate(-self.orient_ * 180.0 / math.pi + 90.0)
        self.turtle_rotated_image_ = self.turtle_image_.transformed(transform)
        
    class __TeleportRequest:
        def __init__(self, x:float, y:float, _theta:float, _linear:float, _relative:bool) -> None:
            self.pos:QPointF = QPointF(x, y)
            self.theta:float = (_theta)
            self.linear:float = (_linear)
            self.relative:bool = (_relative)    
