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
point_list_four = [[68.2652278177458, 67.20923261390887], [412.67540574282145, 55.89138576779026], [416.88634920634917, 412.3447619047619], [68.91631603553061, 413.3272557269752]]
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

def move_to_one_point(target_x, target_y,set=1,kp_x=0.030, ki_x=0.0, kd_x=0.003, kp_y=0.030, ki_y=0.00, kd_y=0.003):
    """
    移动到一个点，增量式PID 移动某个点（不保证直线，保证稳定）
    :param target_x: 目标点的x坐标
    :param target_y: 目标点的y坐标
    :param set: 连续稳态的次数，越高越精准但是越慢，默认设置为1
    :return:
    """
    # 传不同的PID参量进来
    pid_controller.kp_x = kp_x
    pid_controller.ki_x = ki_x
    pid_controller.kd_x = kd_x
    pid_controller.kp_y = kp_y
    pid_controller.ki_y = ki_y
    pid_controller.kd_y = kd_y
    global pid_enabled
    print("start a new point:",target_x,target_y)
    pid_controller.target_x = target_x
    pid_controller.target_y = target_y
    pid_controller.reached = 0
    for i in range (2):
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
                servo_control.control_servo(int(pid_controller.pid_output_y * 10),
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

def four_point_by_eyes():
    move_x = 0
    move_y = 0
    save_point = 0
    while True:
        six_key.read_input()
        cap.read_frame(cut=1)
        cap.show_frame(wait_set=10)
        cap.draw_circle_find()  # 用于看是不是在四个角点内移动的
        cap.show_frame(window_name="init", img_show=cap.copy)
        cap.gray_find_red_light()
        if six_key.pin_pressed[0]:  # 0号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            move_x += 1
        elif six_key.pin_pressed[1]:  # 1号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            move_x -= 1
        elif six_key.pin_pressed[2]:  # 2号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            move_y -= 1
        elif six_key.pin_pressed[3]:  # 3号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            move_y += 1
        elif six_key.pin_pressed[4]: # 4号按键已经被按下
            six_key.flash_all_key()
            point_list_four[save_point][0] = cap.cX
            point_list_four[save_point][1] = cap.cY
            show_lcd(cap.frame)
            save_point += 1
        elif six_key.pin_pressed[5]: # 5号按键已经被按下
            six_key.flash_all_key()
            move_x = 0
            move_y = 0
        if save_point == 4:
            print("四个点的坐标是：",point_list_four)
            break
        servo_control.control_servo(move_y, move_x)
        # print(move_x, move_y)
def interpolate(start_x, start_y, end_x, end_y, num_points):
    x_values = [start_x + i * (end_x - start_x) / (num_points - 1) for i in range(num_points)]
    y_values = [start_y + i * (end_y - start_y) / (num_points - 1) for i in range(num_points)]
    return list(zip(x_values, y_values))


def move_to_point(start_x, start_y, end_x, end_y, num_set=6):
    num_points = num_set # 你可以根据你的需求调整这个
    points = interpolate(start_x, start_y, end_x, end_y, num_points)
    # print(points)
    for target_x, target_y in points:
        print(target_x, target_y)
        if target_x == end_x and target_y == end_y:  # 如果是最后一个点的位置放宽限制
            pid_controller.threshold = 12
            move_to_one_point(target_x, target_y,set=2)
            pid_controller.threshold = 8
            break
        pid_controller.threshold = 8
        move_to_one_point(target_x, target_y,set=1)

def move_to_point_small(start_x, start_y, end_x, end_y, num_set=6):
    num_points = num_set # 你可以根据你的需求调整这个
    points = interpolate(start_x, start_y, end_x, end_y, num_points)
    # print(points)
    for target_x, target_y in points:
        print(target_x, target_y)
        if target_x == end_x and target_y == end_y:  # 如果是最后一个点的位置放宽限制
            pid_controller.threshold = 10
            move_to_one_point(target_x, target_y,set=2,kp_x=0.030, ki_x=0.0, kd_x=0.003, kp_y=0.030, ki_y=0.00, kd_y=0.003)
            pid_controller.threshold = 8
            break
        pid_controller.threshold = 8
        move_to_one_point(target_x, target_y,set=1,kp_x=0.030, ki_x=0.0, kd_x=0.003, kp_y=0.030, ki_y=0.00, kd_y=0.003)

def four_point():
    move_to_one_point(point_list_four[0][0],point_list_four[0][1])  # 先去到左上角的点
    move_to_point(point_list_four[0][0],point_list_four[0][1],point_list_four[1][0],point_list_four[1][1])
    move_to_point(point_list_four[1][0],point_list_four[1][1],point_list_four[2][0],point_list_four[2][1])
    move_to_point(point_list_four[2][0],point_list_four[2][1],point_list_four[3][0],point_list_four[3][1])
    move_to_point(point_list_four[3][0],point_list_four[3][1],point_list_four[0][0],point_list_four[0][1])
    # move_to_point(65, 55, 415, 55)
    # move_to_point(415, 55, 415, 405)
    # move_to_point(415, 405, 65, 405)
    # move_to_point(65, 405, 65, 55)
def back_to_center():
    """
    回到中心点
    :return:
    """
    move_to_one_point(240, 240, set=5)
def find_the_target():
    servo_control.control_servo(0, -40)
    time.sleep(1)  # 先移开光斑以免影响检测
    servo_control.control_stop()
    for i in range(15):
        cap.read_frame()  # 清空缓冲
    while True:
        # serial_0.send_frame()
        rectangles_corners = []  # 重置矩形角点列表
        corner_list = []
        cap.read_frame()
        cap.draw_circle()
        cap.hsv_frame()
        cap.show_frame()
        cap.show_frame(window_name="init", img_show=cap.copy)
        cap.show_frame(window_name="hsv", img_show=cap.dst)
        # show_lcd(cap.copy)
        cap.edge_frame()
        cap.corner_detect()
        if cap.points_list != [[0,0],[0,0],[0,0],[0,0]]:  # 说明识别到了
            break
    print("我已经成功识别 识别到的坐标是：",cap.points_list)
    cv2.destroyAllWindows()  # 关闭所有窗口
    servo_control.control_servo(0, 40)
    time.sleep(1)  # 把光斑移动回来
    servo_control.control_stop()

def move_to_the_target():
    move_to_one_point(cap.points_list[2][0],cap.points_list[2][1],set=3)
    move_to_point_small(cap.points_list[2][0],cap.points_list[2][1],cap.points_list[1][0],cap.points_list[1][1])
    move_to_point_small(cap.points_list[1][0],cap.points_list[1][1],cap.points_list[0][0],cap.points_list[0][1])
    move_to_point_small(cap.points_list[0][0],cap.points_list[0][1],cap.points_list[3][0],cap.points_list[3][1])
    move_to_point_small(cap.points_list[3][0],cap.points_list[3][1],cap.points_list[2][0],cap.points_list[2][1])
    print("移动完毕")
# 回调函数
def callback_function(channel):
    global pid_enabled
    print(pid_enabled)
    if pid_enabled:
        servo_control.control_stop()
    pid_enabled = not pid_enabled  # 切换PID使能标志的状态

if __name__ == '__main__':
    GPIO.add_event_detect(21, GPIO.FALLING, callback=callback_function, bouncetime=300)  # 开启事件检测
    # init_location()
    # four_point_calibration()
    move_to_point(65, 65, 415, 65)
    # move_to_one_point(137.1,221,set=5)
    # four_point_by_eyes()
    target_flag = 0
    while True:
        six_key.read_input() # 循环读取6个key的值
        if six_key.pin_pressed[1]:  # 1号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            four_point_calibration()
        elif six_key.pin_pressed[2]: # 2号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            back_to_center()
        elif six_key.pin_pressed[3]: # 3号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            four_point()
        elif six_key.pin_pressed[4]: # 4号按键已经被按下
            six_key.flash_all_key()  # 清空所有的值
            if target_flag == 0:  # 第一次是只看不走
                find_the_target()
                target_flag += 1
            elif target_flag == 1: # 第二次是直接走
                move_to_the_target()
                target_flag = 0
