import sys
sys.path.insert(0, "/home/pi/.local/lib/python3.7/site-packages")
import os
os.chdir("/home/pi/Desktop/main_2023")
import raspberry_king
from util.get_map import show_lcd
import cv2
import numpy as np


# 摄像头反复调用
exit_loop = False
while True:
    cam = cv2.VideoCapture(0)  # -1就是使用默认摄像头 防止报错
    cam.set(cv2.CAP_PROP_EXPOSURE, -4)  # 设置曝光值
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


def sort_corners(corners):
    # 计算几何中心
    center = np.mean(corners, axis=0)

    # 使用arctan2计算每个角点相对于中心的角度
    angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])

    # 对角点进行排序
    sorted_corners = corners[np.argsort(angles)[::-1]]  # 顺时针排序

    return sorted_corners

def average_corners(corner_list):
    # 将每个矩形的对应角点相加
    sum_corners = sum(corner_list)

    # 除以矩形的数量以获得每个角点的平均值
    avg_corners = sum_corners / len(corner_list)

    return avg_corners

# cv2.waitKey(0)
# cv2.destroyAllWindows()

if __name__ == '__main__':
    key = raspberry_king.KeyInput()
    cap = raspberry_king.Video(camera=cam)
    serial_0 = raspberry_king.Serial()
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
        serial_0.send_coordinates(cap.points_list)
        # serial_0.send_coordinates_as_string(cap.points_list)


    print("test done")
