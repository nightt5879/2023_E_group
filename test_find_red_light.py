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
def move_to_one_point(target_x, target_y,set=1):
    print("start a new point:",target_x,target_y)
    pid_controller.target_x = target_x
    pid_controller.target_y = target_y
    pid_controller.reached = 0
    global pid_enabled
    while pid_controller.reached == 0:
        cap.read_frame(cut=1)
        cap.show_frame(wait_set=10)
        cap.draw_circle_find()
        cap.show_frame(window_name="init", img_show=cap.copy)
        cap.gray_find_red_light()
        current_x, current_y = cap.cX, cap.cY
        # if key.read_input() == 0:  # 如果检测到按键
        #     print(pid_enabled)
        #     # servo_control.control_stop()
        #     if pid_enabled:
        #         servo_control.control_stop()
                # servo_control.control_servo(1, 0)
            # pid_enabled = not pid_enabled  # 切换PID使能标志的状态
        if pid_enabled and (current_x != 0) and (current_y != 0):
            pid_controller.calculate_pid_increment(current_x, current_y,flag_set=set)
            # if (-int(pid_controller.pid_output_y * 10) != 0 or -int(pid_controller.pid_output_x * 10) != 0):
            #     servo_control.control_servo(-int(pid_controller.pid_output_y * 10),
            #                                 -int(pid_controller.pid_output_x * 10))
            # print("x:", pid_controller.pid_output_x, "y:", pid_controller.pid_output_y)
            # print("x:", -int(pid_controller.pid_output_y * 10), "y:", -int(pid_controller.pid_output_x * 10))
        time.sleep(0.01)
    servo_control.control_stop()
    print("Reached final target!",current_x,current_y)

def move_to_one_point_test(target_x, target_y,set=1,set_speed = 1):
    pid_controller.target_x = target_x
    pid_controller.target_y = target_y
    pid_controller.reached = 0
    while pid_controller.reached == 0:
        cap.read_frame(cut=1)
        cap.show_frame(wait_set=10)
        cap.gray_find_red_light()
        current_x, current_y = cap.cX, cap.cY
        if (current_x != 0) and (current_y != 0):
            pid_controller.calculate(current_x, current_y,flag_set=set)
            if pid_controller.pid_output_x > 0:
                x_speed = int(-set_speed-pid_controller.pid_output_x * 10)
            else:
                x_speed = int(set_speed-pid_controller.pid_output_x * 10)
            if pid_controller.pid_output_y > 0:
                y_speed = int(-set_speed-pid_controller.pid_output_y * 10)
            else:
                y_speed = int(set_speed-pid_controller.pid_output_y * 10)
            # y_speed = -set_speed-int(pid_controller.pid_output_y * 10)
            # y_speed = 0
            # x_speed = 0
            servo_control.control_servo(y_speed,x_speed)
            print("x:", pid_controller.pid_output_x, "y:", pid_controller.pid_output_y)
            print("x_speed:", x_speed, "y_speed:", y_speed)
    # pid_controller.reached = 0  #重置一下标志位
    servo_control.control_servo(0,0)
    print("Reached final target!",current_x,current_y)

def interpolate(start_x, start_y, end_x, end_y, num_points):
    x_values = [start_x + i * (end_x - start_x) / (num_points - 1) for i in range(num_points)]
    y_values = [start_y + i * (end_y - start_y) / (num_points - 1) for i in range(num_points)]
    return list(zip(x_values, y_values))

def move_to_point(start_x, start_y, end_x, end_y, num_set=5):
    num_points = num_set # 你可以根据你的需求调整这个
    points = interpolate(start_x, start_y, end_x, end_y, num_points)
    # print(points)
    for target_x, target_y in points:
        print(target_x, target_y)
        if target_x == end_x and target_y == end_y:  # 如果是最后一个点的位置放宽限制
            pid_controller.threshold = 10
            move_to_one_point(target_x, target_y,set=1)
            pid_controller.threshold = 8
            break
        move_to_one_point(target_x, target_y,set=1)


        # print(target_x,target_y)
    #     pid_controller.target_x = target_x
    #     pid_controller.target_y = target_y
    #     pid_controller.reached = 0
    #     while pid_controller.reached == 0:
    #         cap.read_frame(cut=1)
    #         cap.show_frame(wait_set=10)
    #         cap.gray_find_red_light()
    #         current_x, current_y = cap.cX, cap.cY
    #         if (current_x != 0) and (current_y != 0):
    #             pid_controller.calculate_pid_increment(current_x, current_y,flag_set=1)
    #             servo_control.control_servo(-int(pid_controller.pid_output_y * 10),-int(pid_controller.pid_output_x * 10))
    #             print("x:", pid_controller.pid_output_x, "y:", pid_controller.pid_output_y)
    # print("Reached final target!_______________________",current_x,current_y)

def callback_function(channel):
    global pid_enabled
    print(pid_enabled)
        # servo_control.control_stop()
    if pid_enabled:
        servo_control.control_stop()
    pid_enabled = not pid_enabled  # 切换PID使能标志的状态

if __name__ == "__main__":
    # move_to_point(65, 65, 415, 65)
    # move_to_point(415, 65, 415, 415)
    # move_to_point(415, 415, 65, 415)
    # move_to_point(65, 415, 65, 65)
    # move_to_point(65, 65, 415, 65)
    # move_to_point(415, 65, 65, 65)
    # move_to_one_point(240, 240)
    # move_to_one_point(65, 65)
    # move_to_one_point(415, 65)
    # move_to_one_point(415, 415)
    # move_to_one_point(65, 415)
    # move_to_one_point(65, 65)
    GPIO.add_event_detect(21, GPIO.FALLING, callback=callback_function, bouncetime=300)
    while True:
        pass
        # move_to_one_point(240, 240)
        # move_to_one_point(200, 200)
        # move_to_one_point(65, 65)
        # time.sleep(1)
        # move_to_one_point(415, 65)
        # time.sleep(1)
        # move_to_one_point(415, 415)
        # time.sleep(1)
        # move_to_one_point(65, 415)
        # time.sleep(1)
        # move_to_one_point(65, 65)
        # time.sleep(1)
