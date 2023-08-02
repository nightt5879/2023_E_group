# 这里使用面向对象写所有的相关代码

import RPi.GPIO as GPIO
import cv2
import time
import numpy as np

class KeyInput:
    """
    这里是按键输入的类
    """
    def __init__(self, pin_set=18):
        """
        初始化
        :param pin_set: 设置的按键GPIO
        """
        GPIO.setmode(GPIO.BCM)  # BCM编码
        self.pin = pin_set
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def read_input(self):
        """
        读取按键输入
        :return: 返回按键输入的状态
        """
        input_val = None
        if GPIO.input(self.pin) == GPIO.LOW:
            time.sleep(0.1)  # 按键消除抖动
            if GPIO.input(self.pin) == GPIO.LOW:
                input_val = 0
        elif GPIO.input(self.pin) == GPIO.HIGH:
            time.sleep(0.1)  # 按键消除抖动
            if GPIO.input(self.pin) == GPIO.HIGH:
                input_val = 1
        return input_val

    def wait_press(self):
        """
        等待按键按下
        简单写了一个死循环，如果按下才会跳出这个循环
        :return:
        """
        while self.read_input() != 0:
            pass
        print("key pressed")

class Video:
    """
    这里是视频的类
    """
    def __init__(self, camera_num=0, camera=None):
        self.camera_num = camera_num
        self.cap = camera if camera else cv2.VideoCapture(self.camera_num)  # GYJ_高端修改 适应各种使用情况
        self.frame = None
        self.copy = None
        self.dst = None
        self.edges = None

    def read_frame(self):
        """
        读取视频帧
        :return: 返回视频帧
        """
        ret, self.frame = self.cap.read()
        if not ret:
            print("read frame failed")  # 说明摄像头读取有问题

    def show_frame(self,window_name="frame",img_show=None):
        """
        显示视频帧
        :return:
        """
        if img_show is None:  # 如果不输入图片，就显示读取原始的视频帧
            img_show = self.frame
        cv2.imshow(window_name, img_show)
        cv2.waitKey(1)

    def draw_circle(self):
        self.copy = self.frame.copy()
        # 圆心坐标 (320, 240)，半径为 50，红色，线条宽度为 -1，即填充整个圆
        center = (320, 240)
        radius = 2
        color = (0, 0, 255)  # 红色 (B, G, R)
        thickness = -1
        # 绘制圆圈(中心的小红点）
        cv2.circle(self.copy, center, radius, color, thickness)
        radius = 8
        color = (0, 255, 0)  # 绿色
        thickness = 1
        # 绘制圆圈(外侧的大圆圈）
        cv2.circle(self.copy, center, radius, color, thickness)

        # 矩形框的左上角和右下角坐标
        start_point = (130, 50)
        end_point = (510, 430)
        # 矩形框的颜色为蓝色 (B, G, R)，线条宽度为 2
        color = (255, 0, 0)
        thickness = 2
        # 绘制矩形框
        cv2.rectangle(self.copy, start_point, end_point, color, thickness)

    def hsv_frame(self):
        """
        HSV遮罩
        :return:
        """
        # 转换到HSV颜色空间
        blurred = cv2.GaussianBlur(self.frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # 裁剪成框图内的图像
        hsv = hsv[50:430, 130:510]
        lower_black = np.array([0, 40, 0])
        upper_black = np.array([180, 255, 75])
        # 使用HSV阈值过滤黑色
        mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.bitwise_not(mask)  # 反转一下图像
        self.dst = cv2.GaussianBlur(mask, (5, 5), 0)  # 我需要的就是遮罩之后的图像,高斯模糊一些

    def edge_frame(self):
        """
        边缘检测
        :return:
        """
        self.edges = cv2.Canny(self.dst, threshold1=30, threshold2=60)
        cv2.imshow('Edge Detection', self.edges)

