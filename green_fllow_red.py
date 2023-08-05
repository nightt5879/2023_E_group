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
key = raspberry_king.KeyInput(pin_set=21)
servo_control = raspberry_king.ServoSTM32()  # 串口发送给STM32控制舵机
pid_enabled = True  # 增加一个PID使能标志

def move_to_one_point(target_x, target_y,set=1):
    print("start a new point:",target_x,target_y)

    global pid_enabled


    print("Reached final target!",current_x,current_y)

def callback_function(channel):
    global pid_enabled
    print(pid_enabled)
    if pid_enabled:
        servo_control.control_stop()
    pid_enabled = not pid_enabled  # 切换PID使能标志的状态
if __name__ == '__main__':
    target_x = 320
    target_y = 240
    GPIO.add_event_detect(21, GPIO.FALLING, callback=callback_function, bouncetime=300)  # 开启事件检测
    pid_controller.target_x = target_x
    pid_controller.target_y = target_y
    servo_control.control_stop()
    while True:
        if pid_enabled:
            cap.read_frame()
            cap.show_frame(wait_set=10)
            cap.show_frame(window_name="init", img_show=cap.copy)
            cap.gray_find_red_light()
            current_x, current_y = cap.cX, cap.cY
            if pid_enabled and (current_x != 0) and (current_y != 0):
                pid_controller.calculate_pid_increment(current_x, current_y, flag_set=set)
                if (-int(pid_controller.pid_output_y * 10) != 0 or -int(pid_controller.pid_output_x * 10) != 0):
                    servo_control.control_servo(-int(pid_controller.pid_output_y * 10),
                                                int(pid_controller.pid_output_x * 10))
                # print("x:", pid_controller.pid_output_x, "y:", pid_controller.pid_output_y)
                print("x:", int(pid_controller.pid_output_y * 10), "y:", -int(pid_controller.pid_output_x * 10))
            # 追踪