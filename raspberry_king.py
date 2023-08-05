# 这里使用面向对象写所有的相关代码
import RPi.GPIO as GPIO
import cv2
import time
import numpy as np
import serial
import struct

GPIO.setmode(GPIO.BCM)  # BCM编码
class KeyInput:
    """
    这里是按键输入的类
    """
    def __init__(self, pin_set=18):
        """
        初始化
        :param pin_set: 设置的按键GPIO
        """
        self.pin = pin_set
        self.pin_pressed = 0
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def read_input(self):
        """
        读取按键输入
        :return: 返回按键输入的状态
        """
        input_val = None
        if GPIO.input(self.pin) == GPIO.LOW:
            time.sleep(0.001)  # 按键消除抖动
            if GPIO.input(self.pin) == GPIO.LOW:
                self.pin_pressed = 1
        elif GPIO.input(self.pin) == GPIO.HIGH:
            time.sleep(0.001)  # 按键消除抖动
            if GPIO.input(self.pin) == GPIO.HIGH and self.pin_pressed == 1:  # 说明之前按下了
                self.pin_pressed = 0 # 清空按键按下状态
                input_val = 0 # 这说明完成了按下到抬起的一个完整的按键动作
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

class SixKeyInput:
    """
    六个按键输入的类
    """
    def __init__(self):
        """
        初始化
        :param pin_set: 设置的按键GPIO
        """
        self.pin = [18, 23, 24, 25, 4, 5]
        for i in range(6):
            GPIO.setup(self.pin[i], GPIO.IN, pull_up_down=GPIO.PUD_UP) # 初始化六个按键
        self.pin_val = [1, 1, 1, 1, 1, 1]  # 初始化按键状态
        self.pin_pressed = [0, 0, 0, 0, 0, 0]  # 初始化按键按下状态

    def read_input(self):
        """
        读取按键输入
        :return: 返回按键输入的状态
        """
        for i in range(6):
            if GPIO.input(self.pin[i]) == GPIO.LOW:
                time.sleep(0.0001)  # 按键消除抖动
                if GPIO.input(self.pin[i]) == GPIO.LOW:
                    self.pin_val[i] = 0
            elif GPIO.input(self.pin[i]) == GPIO.HIGH:
                time.sleep(0.0001)
                if GPIO.input(self.pin[i]) == GPIO.HIGH and self.pin_val[i] == 0:  # 说明之前按下了
                    self.pin_val[i] = 1  # 清空按键按下状态
                    self.pin_pressed[i] = 1  # 这说明完成了按下到抬起的一个完整的按键动作

    def flash_all_key(self):
        """
        清空按键的状态
        """
        for i in range(6):
            self.pin_pressed[i] = 0

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
        self.out_img = None
        self.points_list = [[0,0],[0,0],[0,0],[0,0]]
        self.cX = 0
        self.cY = 0

    def read_frame(self,cut=0):
        """
        读取视频帧
        :return: 返回视频帧
        """
        ret, self.frame = self.cap.read()
        if cut:
            self.frame = self.frame[10:470, 80:560]  # 裁剪一下图像
        if not ret:
            print("read frame failed")  # 说明摄像头读取有问题

    def show_frame(self,window_name="frame",img_show=None,wait_set=1):
        """
        显示视频帧
        :return:
        """
        if img_show is None:  # 如果不输入图片，就显示读取原始的视频帧
            img_show = self.frame
        cv2.imshow(window_name, img_show)
        cv2.waitKey(wait_set)

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
        start_point = (145, 65)
        end_point = (495, 415)
        # 矩形框的颜色为蓝色 (B, G, R)，线条宽度为 2
        color = (255, 0, 0)
        thickness = 2
        # 绘制矩形框
        cv2.rectangle(self.copy, start_point, end_point, color, thickness)
        start_point = (80, 10)
        end_point = (560, 470)
        color = (0, 255, 0)
        cv2.rectangle(self.copy, start_point, end_point, color, thickness)

    def draw_circle_find(self):
        self.copy = self.frame.copy()
        # 圆心坐标 (320, 240)，半径为 50，红色，线条宽度为 -1，即填充整个圆
        center = (240, 240)
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
        start_point = (65, 65)
        end_point = (415, 415)
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
        hsv = hsv[10:470, 80:560]
        lower_black = np.array([0, 12, 200])
        upper_black = np.array([180, 255, 255])
        # 使用HSV阈值过滤黑色
        mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.bitwise_not(mask)  # 反转一下图像
        self.dst = cv2.GaussianBlur(mask, (5, 5), 0)  # 我需要的就是遮罩之后的图像,高斯模糊一些

    def hsv_frame_red(self):
        """
        HSV遮罩
        :return:
        """
        # 转换到HSV颜色空间
        blurred = cv2.GaussianBlur(self.frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # 裁剪成框图内的图像
        # hsv = hsv[50:430, 130:510]
        lower_black = np.array([86, 33, 159])
        upper_black = np.array([180, 208, 248])
        # 使用HSV阈值过滤黑色
        mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.bitwise_not(mask)  # 反转一下图像
        self.dst = cv2.GaussianBlur(mask, (5, 5), 0)  # 我需要的就是遮罩之后的图像,高斯模糊一些

    def find_red_light(self):
        # 找到所有轮廓
        edges = cv2.Canny(self.dst, threshold1=30, threshold2=60)
        cv2.imshow('Edge Detection', edges)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(len(contours))
        # 找到最大的轮廓
        max_contour = max(contours, key=cv2.contourArea) if contours else None

        # 如果找到了最大轮廓，绘制它
        if max_contour is not None:
            print("i in ")
            cv2.drawContours(self.frame, [max_contour], -1, (0, 255, 0), 2)

            # 计算轮廓的中心点
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                self.cX = int(M["m10"] / M["m00"])
                self.cY = int(M["m01"] / M["m00"])
                # 绘制中心点
                cv2.circle(self.frame, (self.cX, self.cY), 5, (0, 0, 255), -1)
            else:
                # 这说明没有找到点 直接返回0，0 用于PID计算和作用
                self.cX = 0
                self.cY = 0

        # 显示带有绘制的最大轮廓和中心点的图像
        cv2.imshow('Detected Contour', self.dst)
        cv2.imshow('Edge Detection', edges)
        cv2.imshow('Detected Contour', self.frame)

    def gray_find_red_light(self):
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # 应用高斯模糊
        blurred = cv2.GaussianBlur(gray, (15, 15), 0)
        cv2.imshow('Gray_blurred', blurred)

        # 设置阈值
        _, thresh = cv2.threshold(blurred, 220, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        thresh = cv2.dilate(thresh, kernel, iterations=2)
        thresh = cv2.erode(thresh, kernel, iterations=1)

        cv2.imshow('Threshold', thresh)

        # 找到轮廓
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 计算摄像头的总面积
        total_area = self.frame.shape[0] * self.frame.shape[1]

        # 找到面积小于0.3倍总摄像头面积的最大轮廓
        max_contour = None
        max_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area and area < total_area * 0.1:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            # 计算轮廓的矩
            M = cv2.moments(max_contour)

            # 使用矩计算中心点
            if M["m00"] != 0:
                self.cX = M["m10"] / M["m00"]
                self.cY = M["m01"] / M["m00"]
            else:
                self.cX, self.cY = 0.0, 0.0
            # 绘制轮廓
            cv2.drawContours(self.frame, [max_contour], -1, (0, 255, 0), 2)

            # 绘制中心点
            cv2.circle(self.frame, (int(self.cX), int(self.cY)), 5, (0, 0, 255), -1)  # 将坐标转为整数以便绘图

            # 输出中心点坐标
            # print(f"中心点坐标: ({self.cX}, {self.cY})")

        # 显示结果
        cv2.imshow('Detected Bright Spot', self.frame)
        cv2.imshow('Threshold', thresh)

    def edge_frame(self):
        """
        边缘检测
        :return:
        """
        self.edges = cv2.Canny(self.dst, threshold1=30, threshold2=60)
        cv2.imshow('Edge Detection', self.edges)

    def corner_detect(self):
        self.out_img = self.copy[10:470, 80:560]  # 裁剪后的图像
        # 找到所有轮廓
        contours, _ = cv2.findContours(self.edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  # 这个方法没有层级 可能识别多个轮廓
        # print("轮廓数量：%d" % len(contours))
        # 定义面积范围
        min_area = 0.1 * self.out_img.shape[0] * self.out_img.shape[1]
        max_area = 0.9 * self.out_img.shape[0] * self.out_img.shape[1]

        # 过滤轮廓并找到每个轮廓的最小包围矩形
        filtered_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if min_area < area < max_area:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                filtered_contours.append(box)

        # 打印检测到的轮廓数量
        print("检测到的轮廓数量:", len(filtered_contours))

        # 绘制过滤后的轮廓并打印坐标
        for idx, contour in enumerate(filtered_contours):
            # cv2.drawContours(self.out_img, [contour], -1, (0, 255, 0), 2)
            print(f"轮廓{idx} 坐标点为：")
            print(contour)
        # 显示带有轮廓的图像
        cv2.imshow('Contours', self.out_img)

        # 调用上述函数来合并接近的轮廓
        merged_contours = self.merge_close_contours(filtered_contours, threshold=50)
        print("合并后的轮廓数量:", len(merged_contours))
        for idx, contour in enumerate(merged_contours):
            # cv2.drawContours(self.out_img, [contour], -1, (0, 255, 0), 2)
            # print(f"合并后的轮廓{idx} 坐标点为：")
            print(contour)
        # 确保只有两个轮廓
        if len(merged_contours) == 2:
            # 取两个轮廓的平均值得到中间轮廓
            weight_outer = 0.8
            weight_inner = 0.2
            # 合并前确保轮廓对齐
            merged_contours[0], merged_contours[1] = self.align_contours(merged_contours[0], merged_contours[1])
            middle_contour = weight_outer * merged_contours[0] + weight_inner * merged_contours[1]
            # 排序角点顺时针
            middle_contour_sorted = self.sort_clockwise(middle_contour)

            # 保存到self.list
            self.points_list = middle_contour_sorted.tolist()
            # 将结果转换为整数坐标
            middle_contour = middle_contour.astype(np.int0)
            # middle_contour = (merged_contours[0] + merged_contours[1]) / 2

            # 打印中间轮廓的坐标
            print("框上的点坐标为：")
            # self.points_list = self.normalized_xy(self.points_list)  # 归一化一下坐标
            print(self.points_list)

            # 绘制中间轮廓
            cv2.drawContours(self.out_img, [middle_contour], -1, (0, 255, 0), 2)
            # 绘制中间轮廓的四个角点上的小圆圈
            for point in middle_contour:
                cv2.circle(self.out_img, tuple(point), 5, (0, 0, 255), -1)
            cv2.imshow('Contours', self.out_img)
        else:
            print("请确保合并后有两个轮廓！")

        # cv2.imshow('Contours', self.out_img)

    def merge_close_contours(self, contours, threshold=10):
        """
        合并接近的轮廓
        :param contours: 输入的轮廓
        :param threshold: 合并的阈值
        :return:
        """
        merged_contours = []
        merged_flags = [False] * len(contours)

        for i in range(len(contours)):
            if merged_flags[i]:
                continue

            # 计算轮廓i与其他轮廓之间的距离
            distances = [np.sum(np.abs(contours[i] - contours[j])) for j in range(len(contours))]

            # 查找与轮廓i距离小于阈值的所有轮廓
            close_contours = [contours[j] for j in range(len(contours)) if distances[j] < threshold and not merged_flags[j]]

            # 将这些轮廓合并为一个
            if close_contours:
                merged_contour = np.mean(close_contours, axis=0).astype(np.int0)
                merged_contours.append(merged_contour)
                for j in range(len(contours)):
                    if distances[j] < threshold:
                        merged_flags[j] = True

        return merged_contours

    def align_contours(self, contour1, contour2):
        # 寻找每个轮廓的左下角
        ref_index1 = np.argmin(-contour1[:, 1] + contour1[:, 0])
        ref_index2 = np.argmin(-contour2[:, 1] + contour2[:, 0])

        # 重新排序轮廓，使左下角的点在首位
        aligned_contour1 = np.roll(contour1, shift=-ref_index1, axis=0)
        aligned_contour2 = np.roll(contour2, shift=-ref_index2, axis=0)

        return aligned_contour1, aligned_contour2

    def sort_clockwise(self, points):
        """
        顺时针排序
        :return:
        """
        # 计算几何中心
        center = np.mean(points, axis=0)

        # 使用arctan2计算每个角点相对于中心的角度
        angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])

        # 对角点进行排序
        sorted_points = points[np.argsort(angles)[::-1]]  # 顺时针排序

        return sorted_points

    def normalized_xy(self, point_input):
        """

        :return:
        """
        # 保存归一化后的坐标
        normalized_points = []

        # 保存归一化后的坐标
        for point in self.points_list:
            """
            总共的坐标长宽都是380乘以380像素点，从左上角为起点搬运到了190，190为原点
            原点按照正常的坐欧拉坐标系建系，由于是50cm对应的380像素点（归一化就是除以380乘以50）
            得到cm的单位坐标
            """
            # 保留小数点后两位，即精度到0.01cm 0.1mm
            x = round((point[0] - 190) / 380 * 50, 2)
            y = round((-point[1] + 190) / 380 * 50, 2)
            normalized_points.append([x, y])

        # 打印归一化后的坐标
        # print("归一化后的坐标为：")
        # print(normalized_points)

        return normalized_points

class Serial:
    """
    这里是串口的类
    """
    def __init__(self, port="/dev/ttyAMA0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(self.port, self.baudrate)
        self.data = []
        self.data = [0] * 32  # 初始化32个数据位为0

    def send_frame(self):
        """
        发送帧头为FF，帧尾为FE，中间数据位为32位的数据
        :return:
        """
        frame_head = "FF"
        frame_data = "".join(f"{d:02X}" for d in self.data)  # 将数据位转换成16进制字符串格式
        frame_end = "FE"

        frame = frame_head + frame_data + frame_end
        frame_bytes = bytes.fromhex(frame)  # 将帧转换成字节格式

        self.ser.write(frame_bytes)
    def float_to_hex(self, f):
        return hex(struct.unpack('<I', struct.pack('<f', f))[0])

    def send_coordinates(self, coordinates_list):
        """
        将坐标列表转换为HEX数据包并通过串口发送
        :param coordinates_list: 浮点坐标列表
        :return: None
        """
        frame_head = "FF"
        frame_data = ""
        for point in coordinates_list:
            for coord in point:
                frame_data += self.float_to_hex(coord)[2:].zfill(8)  # 转换为HEX并去掉"0x"前缀
        frame_end = "FE"

        frame = frame_head + frame_data + frame_end
        frame_bytes = bytes.fromhex(frame)  # 将帧转换成字节格式

        self.ser.write(frame_bytes)

    def send_coordinates_as_string(self, coordinates_list):
        frame_head = "FF"
        frame_end = "FE"
        frame_data = ",".join([f"{x},{y}" for x, y in coordinates_list])

        frame = frame_head + frame_data + frame_end
        self.ser.write(frame.encode('utf-8'))  # 将字符串转换为字节串并发送




class DualServo:
    """
    这里是舵机的类,GPT写的 舵机做烂了 懒得写注释了 你就说能不能用嘛
    """
    def __init__(self, pin1=17, pin2=27):
        self.pins = [pin1, pin2]
        self.current_angles = [135, 135]

        GPIO.setmode(GPIO.BCM)
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
        self.pwms = [GPIO.PWM(pin, 50) for pin in self.pins]
        for pwm in self.pwms:
            pwm.start(0)

    def set_angle(self, servo_index, angle):
        duty = 2.5 + (angle / 270) * (12.5 - 2.5)
        self.pwms[servo_index].ChangeDutyCycle(duty)
        self.current_angles[servo_index] = angle

    def move_by_direction(self, direction_x=0, speed_x=0, direction_y=0, speed_y=0):
        target_angle_x = self.current_angles[0] + direction_x * speed_x if direction_x != 0 else None
        target_angle_y = self.current_angles[1] + direction_y * speed_y if direction_y != 0 else None
        self.move_to_angle(target_angle_x, speed_x, target_angle_y, speed_y)

    def move_to_angle(self, target_angle_x=None, speed_x=0, target_angle_y=None, speed_y=0):
        if target_angle_x is not None:
            target_angle_x = max(90, min(180, target_angle_x))  # 限制X方向角度在90到180度之间

        if target_angle_y is not None:
            target_angle_y = max(100, min(180, target_angle_y))  # 限制Y方向角度在100到180度之间

        for i, (target_angle, speed, current_angle) in enumerate([(target_angle_x, speed_x, self.current_angles[0]),
                                                                  (target_angle_y, speed_y, self.current_angles[1])]):
            if target_angle is not None:
                angle_diff = target_angle - current_angle
                direction = 1 if angle_diff > 0 else -1
                steps = int(abs(angle_diff) / speed * 50) if speed != 0 else 0

                for step in range(steps):
                    step_size = direction * speed / 50.0
                    self.current_angles[i] += step_size
                    self.set_angle(i, self.current_angles[i])
                    time.sleep(0.02)  # 50Hz更新频率

                self.set_angle(i, target_angle)

    def freeze(self):
        for pwm in self.pwms:
            pwm.ChangeDutyCycle(0)

    def stop(self):
        for pwm in self.pwms:
            pwm.stop()
        GPIO.cleanup()

class IncrementalPID:
    def __init__(self, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y):
        # X方向的PID参数
        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x
        self.prev_error_x = 0
        self.delta_error_x = 0
        self.sum_error_x = 0
        self.target_x = 0
        self.pid_output_x = 0

        # Y方向的PID参数
        self.kp_y = kp_y
        self.ki_y = ki_y
        self.kd_y = kd_y
        self.prev_error_y = 0
        self.delta_error_y = 0
        self.sum_error_y = 0
        self.target_y = 0
        self.pid_output_y = 0

        self.reached = 0
        self.reached_flag = 0
        self.threshold = 8

    def calculate(self, current_x, current_y,flag_set=5):
        # 计算X方向的误差
        error_x = self.target_x - current_x
        delta_error_x = error_x - self.prev_error_x
        self.pid_output_x = self.kp_x * error_x + self.ki_x * self.sum_error_x + self.kd_x * delta_error_x
        self.prev_error_x = error_x
        self.sum_error_x += error_x

        if self.sum_error_x > 5:
            self.sum_error_x = 5
        elif self.sum_error_x < -5:
            self.sum_error_x = -5

        # 计算Y方向的误差
        error_y = self.target_y - current_y
        delta_error_y = error_y - self.prev_error_y
        self.pid_output_y = self.kp_y * error_y + self.ki_y * self.sum_error_y + self.kd_y * delta_error_y
        self.prev_error_y = error_y
        self.sum_error_y += error_y
        if self.sum_error_y > 5:
            self.sum_error_y = 5
        elif self.sum_error_y < -5:
            self.sum_error_y = -5

        error_threshold = 14  # 你可以根据你的需求调整这个值
        error_x = abs(self.target_x - current_x)
        error_y = abs(self.target_y - current_y)
        if error_x < error_threshold and error_y < error_threshold:
            self.reached_flag += 1
            if self.reached_flag > flag_set:  # 连续多次稳定才停下
                print("reached")
                self.reached = 1
                # 清空所有的值
                self.reset()
        else:  # 没有达到
            self.reached_flag = 0  # 清空稳态位置

    def calculate_pid_increment(self, current_x, current_y, flag_set=5,not_break=1):
        # 计算X方向的误差
        # 计算Y方向的误差
        set_th = 1
        error_y = self.target_y - current_y
        error_x = self.target_x - current_x
        # if abs(error_x) < set_th:
        #     error_x = 0
        # if abs(error_y) < set_th:
        #     error_y = 0
        # if (-5 < error_x < 5) and (-5 < error_y < 5):
        #     # pass
        #     self.ki_x = 0.001
        #     self.ki_y = 0.001
        # else:
        #     self.ki_x = 0
        #     self.ki_y = 0
        delta_error_x = self.kp_x * (error_x - self.prev_error_x) + self.ki_x * error_x + self.kd_x * (
                    error_x - 2 * self.prev_error_x + self.delta_error_x)
        self.delta_error_x = self.prev_error_x
        self.prev_error_x = error_x
        self.pid_output_x += delta_error_x
        delta_error_y = self.kp_y * (error_y - self.prev_error_y) + self.ki_y * error_y + self.kd_y * (
                    error_y - 2 * self.prev_error_y + self.delta_error_y)
        self.delta_error_y = self.prev_error_y
        self.prev_error_y = error_y
        self.pid_output_y += delta_error_y

        if not_break:
            error_threshold = self.threshold # 你可以根据你的需求调整这个值
            error_x = abs(self.target_x - current_x)
            error_y = abs(self.target_y - current_y)
            if error_x < error_threshold and error_y < error_threshold:
                self.reached_flag += 1
                if self.reached_flag >= flag_set:  # 连续多次稳定才停下
                    print("reached")
                    self.reached = 1
                    # 清空所有的值
                    self.reset()
            else:  # 没有达到
                self.reached_flag = 0  # 清空稳态位置

    def reset(self):
        """
        清空所有的增量PID控制
        :return:
        """
        self.prev_error_x = 0
        self.delta_error_x = 0
        self.sum_error_x = 0
        self.prev_error_y = 0
        self.delta_error_y = 0
        self.sum_error_y = 0
        self.pid_output_x = 0
        self.pid_output_y = 0
        self.reached_flag = 0


class ServoSTM32:
    def __init__(self):
        """
        init the car
        """
        self.car_com1 = serial.Serial("/dev/ttyAMA1", 115200)  # init the car com
        self.data = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE]  # init the data

    def control_servo(self, servo_1_speed, servo_2_speed):
        """
        control the servo
        :param servo_1_speed: servo 1 speed
        :param servo_2_speed: servo 2 speed
        :return: None
        """
        # print(1)
        if servo_1_speed <= 0:
            self.data[1] = 0x01
            speed = abs(servo_1_speed)
            self.data[2] = speed >> 8
            self.data[3] = speed & 0xFF
        else:
            self.data[1] = 0x00
            speed = servo_1_speed
            self.data[2] = speed >> 8
            self.data[3] = speed & 0xFF
        if servo_2_speed <= 0:
            self.data[4] = 0x01
            speed = abs(servo_2_speed)
            self.data[5] = speed >> 8
            self.data[6] = speed & 0xFF
        else:
            self.data[4] = 0x00
            speed = servo_2_speed
            self.data[5] = speed >> 8
            self.data[6] = speed & 0xFF
        self.car_com1.write(self.data)

    def control_stop(self):
        self.data[1] = 0x00  # 或其他有效的停止标志
        self.data[2] = 0x00
        self.data[3] = 0x00
        self.data[4] = 0x00
        self.data[5] = 0x00
        self.data[6] = 0x00
        self.car_com1.write(self.data)

    def set_angle(self, angle_x, angle_y):
        self.data[1] = 0xFF
        self.data[2] = angle_x
        self.data[3] = angle_y
        self.car_com1.write(self.data)
# 示例用法
if __name__ == "__main__":
    servos = DualServo()
    servos.set_angle(0, 105)
    servos.set_angle(1, 135)
    # servos.move_by_direction(direction_x=1, speed_x=40, direction_y=-1, speed_y=40)
    time.sleep(2)
    servos.freeze()  # 停下舵机
    print("test done")

