from os import popen
from time import time
import math
import numpy as np
from .fine_tune import FineTune
import cv2


class Lidar:
    def __init__(self, img, model_path="../model/ultra_simple --channel --serial /dev/ttyAMA0 115200"):
        self.result = popen(model_path)
        self.fineTuner = FineTune(img)
        for i in range(6):
            self.result.buffer.readline().decode("utf8")

    def get_each(self):
        while True:
            start = time()
            line = self.result.buffer.readline().decode("utf8")
            if time() - start > 0.1:
                break
        return line

    def get_data(self):
        return eval(self.get_each())

    def calculate_target_coordinates(self, angle, distance):
        angle_radians = math.radians(angle)
        target_x = distance * math.cos(angle_radians)
        target_y = distance * math.sin(angle_radians)
        return target_x, target_y

    def get_true_scan(self):
        true_scan = np.ones((400, 400), dtype=np.uint8) * 255
        distance_dict = self.get_data()
        for key, value in distance_dict.items():
            x, y = self.calculate_target_coordinates(key, value)
            true_scan[200 - int(x / 10), 200 + int(y / 10)] = 0
        return true_scan

    def get_angle_dis(self, now_xy, angle_range=180):
        true_scan = self.get_true_scan()
        return self.fineTuner.get_angle_dis(now_xy, true_scan, angle_range=angle_range)


if __name__ == '__main__':
    # 这是21*21迷宫。如果地图不会变化的话他就一定是存储在img。如果地图变化那么在识别地图架构的时候也会识别出来这个图
    img = cv2.imread("/home/pi/Desktop/main_program/img/small_labyrinth.png", 0)
    # 小车当前位置
    now_xy = (3, 1)
    # 初始化激光雷达对象。这个初始化需要输入21*21图像以及C++打包的可执行文件所在位置（使用默认即可）
    my_lidar = Lidar(img, model_path="./ultra_simple --channel --serial /dev/ttyAMA0 115200")
    # 当你识别完宝藏之后调用这个方法即可得到小车需要旋转的角度(正数表示逆时针)以及旋转后需要平移的距离
    angle, (min_x, min_y), _ = my_lidar.get_angle_dis(now_xy)
    print(angle)





    cv2.imshow("img", fine_tune_image)
    cv2.waitKey(0)
