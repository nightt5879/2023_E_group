import sys
sys.path.insert(0, "/home/pi/.local/lib/python3.7/site-packages")
import os
os.chdir("/home/pi/Desktop/main_2023")
import raspberry_king
from util.get_map import show_lcd
import cv2
import numpy as np
import time


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

if __name__ == "__main__":
    cap = raspberry_king.Video(camera=cam)
    pid_controller = raspberry_king.IncrementalPID(kp_x=0.03, ki_x=0.0, kd_x=0.0, kp_y=0.3, ki_y=0.0, kd_y=0.0)
    servos = raspberry_king.DualServo(17, 27)
    # 以每秒15度和20度的速度将两个舵机分别旋转到90度和60度
    servos.move_to_angle(105, 90, 135, 90)
    servos.freeze()
    # servos.stop()
    while True:

        cap.read_frame()
        cap.show_frame(wait_set=10)
        cap.gray_find_red_light()
        # 获取当前的x和y坐标（例如，从摄像头）
        current_x, current_y = cap.cX, cap.cY

        # 计算PID输出
        if (current_x != 0) and (current_y != 0):
            pid_output_x, pid_output_y = pid_controller.calculate(current_x, current_y)
            # 根据PID输出计算舵机的方向和速度
            direction_x = 1 if pid_output_x > 0 else -1
            direction_y = 1 if pid_output_y > 0 else -1

            # 这里你可以调整速度系数来控制舵机的响应速度
            speed_x = abs(pid_output_x) * 1
            speed_y = abs(pid_output_y) * 1

            # 控制舵机
            # servos.move_by_direction(direction_x=direction_x, speed_x=10, direction_y=direction_y, speed_y=10)
            print("x:", pid_output_x, "y:", pid_output_y)

            print("x:", pid_output_x, "y:", pid_output_y)
        # cap.hsv_frame_red()
        # cap.show_frame(window_name="hsv_frame_red", img_show=cap.dst)
        #转换为灰度图像


