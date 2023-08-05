import sys
sys.path.insert(0, "/home/pi/.local/lib/python3.7/site-packages")
import os
os.chdir("/home/pi/Desktop/main_2023")
import raspberry_king
from util.get_map import show_lcd
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# 摄像头反复调用
exit_loop = False
while True:
    cam = cv2.VideoCapture(0)  # -1就是使用默认摄像头 防止报错
    break_flag = 0
    # 读五次图像，如果都成功就跳出
    for i in range(5):
        success, img = cam.read()
        if success:
            show_lcd(img)
            break_flag += 1
            if break_flag >= 1:
                print(break_flag)
                print("摄像头来咯")
                exit_loop = True
                break
        else:
            print("摄像头出不来哦")
            cam.release()  # 释放摄像头
            cam = cv2.VideoCapture(0) # 重新获取摄像头

    if exit_loop:
        break # 跳出外部循环
# 全局变量
pid_controller = raspberry_king.IncrementalPID(kp_x=0.02, ki_x=0.00, kd_x=0.0, kp_y=0.02, ki_y=0.00, kd_y=0.0)
cap = raspberry_king.Video(camera=cam)
servo_control = raspberry_king.ServoSTM32()
key = raspberry_king.KeyInput(pin_set=21)
pid_enabled = True  # 增加一个PID使能标志

