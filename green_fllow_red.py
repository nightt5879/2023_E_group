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
pid_controller = raspberry_king.IncrementalPID(kp_x=0.06, ki_x=0.00, kd_x=0.0, kp_y=0.06, ki_y=0.00, kd_y=0.00)
cap = raspberry_king.Video(camera=cam)
key = raspberry_king.KeyInput(pin_set=21)
servo_control = raspberry_king.ServoSTM32()  # 串口发送给STM32控制舵机
pid_enabled = True  # 增加一个PID使能标志

# 设置BCM引脚编号模式
GPIO.setmode(GPIO.BCM)

# 定义引脚编号
PWM_PIN = 25

# 设置引脚为输出模式
GPIO.setup(PWM_PIN, GPIO.OUT)

# 创建PWM实例，频率为2KHz
pwm = GPIO.PWM(PWM_PIN, 3000)

# 启动PWM，初始占空比为50%
pwm.start(0)
GPIO.setmode(GPIO.BCM)

# 定义引脚编号
PIN = 24

# 设置引脚为输出模式
GPIO.setup(PIN, GPIO.OUT)

def get_the_target():
    GPIO.output(PIN, GPIO.HIGH)  # 设置高电平
    pwm.ChangeDutyCycle(50)

def not_get_the_target():
    GPIO.output(PIN, GPIO.LOW)  # 设置低电平
    pwm.ChangeDutyCycle(0)

def callback_function(channel):
    global pid_enabled
    print(pid_enabled)
    if pid_enabled:
        servo_control.control_stop()
    pid_enabled = not pid_enabled  # 切换PID使能标志的状态
if __name__ == '__main__':
    target_x = 320
    target_y = 240
    thr = 10
    GPIO.add_event_detect(21, GPIO.FALLING, callback=callback_function, bouncetime=300)  # 开启事件检测
    pid_controller.target_x = target_x
    pid_controller.target_y = target_y
    servo_control.control_stop()
    while True:
        cap.read_frame()
        cap.show_frame(wait_set=10)
        cap.show_frame(window_name="init", img_show=cap.copy)
        cap.gray_find_red_light()
        current_x, current_y = cap.cX, cap.cY
        if pid_enabled:
            if abs(current_x - target_x) > thr or abs(current_y - target_y) > thr:
                not_get_the_target()
                if pid_enabled and (current_x != 0) and (current_y != 0):
                    pid_controller.calculate_pid_increment(current_x, current_y, flag_set=set,not_break=0)
                    if (-int(pid_controller.pid_output_y * 10) != 0 or -int(pid_controller.pid_output_x * 10) != 0):
                        servo_control.control_servo(-int(pid_controller.pid_output_y * 10),
                                                    int(pid_controller.pid_output_x * 10))
                    # print("x:", pid_controller.pid_output_x, "y:", pid_controller.pid_output_y)
                    # print("x:", int(pid_controller.pid_output_y * 10), "y:", -int(pid_controller.pid_output_x * 10))
                    print(current_x, current_y)
                # 追踪
            else:
                print("target done")
                servo_control.control_stop()
                # 说明到达了
                get_the_target()