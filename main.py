import sys
sys.path.insert(0, "/home/pi/.local/lib/python3.7/site-packages")  # 插入包的路径
import os
os.chdir("/home/pi/Desktop/main_2023")  # 切换工作区
import raspberry_king  # 自己的库
from util.get_map import show_lcd
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# 摄像头反复调用
exit_loop = False
while True:
    cam = cv2.VideoCapture(0)  # 尝试初始化摄像头
    break_flag = 0
    # 读五次图像，如果都成功就跳出
    for i in range(5):
        success, img = cam.read()
        if success:
            show_lcd(img)
            break_flag += 1
            if break_flag >= 1:
                # print(break_flag)
                print("摄像头来咯")
                exit_loop = True
                break
        else:
            print("摄像头出不来哦")
            cam.release()  # 释放摄像头
            cam = cv2.VideoCapture(0) # 重新获取摄像头

    if exit_loop:
        break # 跳出外部循环
# 实例化对象
six_key = raspberry_king.SixKeyInput()  # 用于选择功能的面包板按键，总计6个对应关系如下：
"""
0号按键：程序开始进行校准 并不断读取这个值，按下再松开表示校准完成跳出第一个校准程序（校准一共是5个点进行校准）
1号按键：四个点校准，程序不断进行五个点的移动，进行微调校准
2号按键：回到中心点，考虑是否要在所有的移动中加入读取 可以随时打断移动回到中点，暂时先做单独的一个复位
3号按键：50cm✖50cm的边框移动，这里直接利用校准得到的值 就走的准确了
4号按键：读取靶子并且顺时针沿着靶子黑线旋转一周
"""
key = raspberry_king.KeyInput(pin_set=21)  # 板子上的按键（用于随时暂停移动）
cap = raspberry_king.Video(camera=cam)  # 摄像头的函数
servo_control = raspberry_king.ServoSTM32()  # 串口发送给STM32控制舵机
pid_controller = raspberry_king.IncrementalPID(kp_x=0.015, ki_x=0.00, kd_x=0.0, kp_y=0.015, ki_y=0.00, kd_y=0.0)  # pid对象
# 全局参数
pid_enabled = True  # 移动暂停与否的标志
# 函数
def init_location():
    """
    初始化图像大概位置
    :return:
    """
    while True:
        six_key.read_input()
        if six_key.pin_pressed[0] == 1:  # 0号按键按下
            break
        cap.read_frame()
        cap.draw_circle()
        cap.show_frame(window_name="init", img_show=cap.copy)
        show_lcd(cap.copy)
    print("初始化图像结束")
    cv2.destroyAllWindows()  # 关闭所有窗口
    show_lcd(cap.frame)  # 展示原图 就知道跳出来了

def move_to_one_point(target_x, target_y,set=1):
    """
    移动到一个点，增量式PID 移动某个点（不保证直线，保证稳定）
    :param target_x: 目标点的x坐标
    :param target_y: 目标点的y坐标
    :param set: 连续稳态的次数，越高越精准但是越慢，默认设置为1
    :return:
    """
    global pid_enabled
    print("start a new point:",target_x,target_y)
    pid_controller.target_x = target_x
    pid_controller.target_y = target_y
    pid_controller.reached = 0
    for i in range (5):
        cap.read_frame(cut=1)  # 清空缓存
    while pid_controller.reached == 0:
        six_key.read_input()  # 只要是移动一个点都读取6个按键值
        cap.read_frame(cut=1)
        cap.show_frame(wait_set=10)
        cap.draw_circle_find()  # 用于看是不是在四个角点内移动的
        cap.show_frame(window_name="init", img_show=cap.copy)
        cap.gray_find_red_light()
        current_x, current_y = cap.cX, cap.cY
        if pid_enabled and (current_x != 0) and (current_y != 0):
            pid_controller.calculate_pid_increment(current_x, current_y,flag_set=set)
            if (-int(pid_controller.pid_output_y * 10) != 0 or -int(pid_controller.pid_output_x * 10) != 0):
                servo_control.control_servo(-int(pid_controller.pid_output_y * 10),
                                            -int(pid_controller.pid_output_x * 10))
            # print("x:", pid_controller.pid_output_x, "y:", pid_controller.pid_output_y)
            # print("x:", -int(pid_controller.pid_output_y * 10), "y:", -int(pid_controller.pid_output_x * 10))
    # cv2.destroyAllWindows()  # 关闭所有窗口
    servo_control.control_stop()
    print("Reached final target!",current_x,current_y)  # 顺便打印一下到的位置是多少
    show_lcd(cap.frame)  # 展示原图 就知道跳出来了

def four_point_calibration():
    while True:
        if six_key.pin_pressed[1] == 1:  # 1号按键已经被按下
            six_key.flash_all_key() # 清空按键值
            break
        move_to_one_point(65, 65)
        move_to_one_point(415, 65)
        move_to_one_point(415, 415)
        move_to_one_point(65, 415)
        move_to_one_point(65, 65)
def four_point():
    move_to_one_point(65, 65)
    move_to_one_point(415, 65)
    move_to_one_point(415, 415)
    move_to_one_point(65, 415)
    move_to_one_point(65, 65)
def back_to_center():
    """
    回到中心点
    :return:
    """
    move_to_one_point(240, 240, set=5)
# 回调函数
def callback_function(channel):
    global pid_enabled
    print(pid_enabled)
    if pid_enabled:
        servo_control.control_stop()
    pid_enabled = not pid_enabled  # 切换PID使能标志的状态

if __name__ == '__main__':
    GPIO.add_event_detect(21, GPIO.FALLING, callback=callback_function, bouncetime=300)  # 开启事件检测
    init_location()
    # four_point_calibration()
    while True:
        six_key.read_input() # 循环读取6个key的值
        if six_key.pin_pressed[1] == 1:  # 1号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            four_point_calibration()
        elif six_key.pin_pressed[2]: # 2号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            back_to_center()
        elif six_key.pin_pressed[3]: # 3号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            four_point()