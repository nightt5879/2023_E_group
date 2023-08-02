import sys
sys.path.insert(0, "/home/pi/.local/lib/python3.7/site-packages")
import os
os.chdir("/home/pi/Desktop/main_program")
import raspberry_king
from util.get_map import show_lcd
import cv2
import numpy as np


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

import cv2
import numpy as np


def find_corners(image):
    # 轮廓检测
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 定义面积范围
    min_area = 0.3 * 380 * 380
    max_area = 0.8 * 380 * 380

    # 用于显示边缘检测结果的图像
    contour_image = np.zeros_like(image)

    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            # 绘制轮廓，用于可视化
            cv2.drawContours(contour_image, [contour], -1, (255, 0, 0), 2)

            # 多边形逼近，降低精度（例如，使用0.1而不是0.05）
            epsilon = 0.5 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # 如果逼近的多边形有4个顶点，则可能是矩形
            if len(approx) == 4:
                return approx.reshape(-1, 2), contour_image

    return None, contour_image


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
    while True:
        rectangles_corners = []  # 重置矩形角点列表
        corner_list = []
        cap.read_frame()
        cap.draw_circle()
        cap.hsv_frame()
        cap.show_frame()
        cap.show_frame(window_name="init", img_show=cap.copy)
        cap.show_frame(window_name="hsv", img_show=cap.dst)
        show_lcd(cap.copy)
        cap.edge_frame()
        out_img = cap.copy[50:430, 130:510]  # 裁剪后的图像
        # 找到所有轮廓
        contours, _ = cv2.findContours(cap.edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        print("轮廓数量：%d" % len(contours))
        # 定义面积范围
        min_area = 0.2 * out_img.shape[0] * out_img.shape[1]
        max_area = 0.9 * out_img.shape[0] * out_img.shape[1]

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
            cv2.drawContours(out_img, [contour], -1, (0, 255, 0), 2)
            print(f"轮廓{idx} 坐标点为：")
            print(contour)

        # 显示带有轮廓的图像
        cv2.imshow('Contours', out_img)
    print("test done")
