import copy
import cv2
import numpy as np
from .map import Treasure
from util import LCD_2inch4
from PIL import Image, ImageFilter
import RPi.GPIO as GPIO
import time

disp = LCD_2inch4.LCD_2inch4()
disp.Init()
disp.clear()


def perspective_trans(corner: list, map_img):
    """
    透视变换
    :param corner:是四个点的坐标[(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
    :param map_img:原始图像
    :return: bool, output_img
    """
    w, h, _ = map_img.shape
    input_pts = np.float32(corner)
    output_pts = np.float32([[0, 0], [0, h], [w, 0], [w, h]])
    M = cv2.getPerspectiveTransform(input_pts, output_pts)
    output_img = cv2.warpPerspective(map_img, M, (w, h))
    output_img = cv2.resize(output_img, (800, 800))
    return True, output_img


def order_points(pts):
    sort_x = pts[np.argsort(pts[:, 0]), :]
    Left = sort_x[:2, :]
    Right = sort_x[2:, :]
    # Left sort
    Left = Left[np.argsort(Left[:, 1])[::-1], :]
    # Right sort
    Right = Right[np.argsort(Right[:, 1]), :]
    return np.concatenate((Left, Right), axis=0)


def gama_transfer(map_img, power):
    if len(map_img.shape) == 3:
        map_img = cv2.cvtColor(map_img, cv2.COLOR_BGR2RGB)
    map_img = 255 * np.power(map_img / 255, power)
    map_img = np.around(map_img)
    map_img[map_img > 255] = 255
    out_img = map_img.astype(np.uint8)
    return out_img


def re_sorted_points(point_list):
    if len(point_list) == 4:
        ordered_points = list(order_points(np.array(point_list)))
        ordered_points[0], ordered_points[1] = ordered_points[1], ordered_points[0]
        for idx, point_array in enumerate(ordered_points):
            ordered_points[idx] = tuple(point_array)
        return True, ordered_points
    else:
        return False, None


def find_scaling_points(map_img):
    img = copy.deepcopy(map_img)
    sum = 0
    squares = []
    points = []
    w, h = img.shape[:-1]
    # 这张图像的总大小
    img_area = w * h
    bin = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (9, 9), 3), 10, 30, apertureSize=3)
    contours, _hierarchy = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for idx, cnt in enumerate(contours):
        cnt_len = cv2.arcLength(cnt, True)
        area = cv2.contourArea(cnt)
        cnt = cv2.approxPolyDP(cnt, 0.1 * cnt_len, True)
        if len(cnt) == 4 and cv2.isContourConvex(cnt) and area > 0.001 * img_area:
            cnt = cnt.reshape(-1, 2)
            squares.append(cnt)
            sum += 1
            M = cv2.moments(contours[idx])
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            points.append((cx, cy))
            if len(points) == 4:
                flag_resorted, points = re_sorted_points(points)
                if flag_resorted:
                    return True, points
    return False, points


def crop_map_img(map_img):
    map = copy.deepcopy(map_img)
    gray = cv2.cvtColor(map, cv2.COLOR_BGR2GRAY)
    threshold = cv2.threshold(gray, 0, 225, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    edged = cv2.Canny(threshold, 30, 200)
    contours, _hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts = []
    # get the max contour area
    for idx, cnt in enumerate(contours):
        cnts.append(cv2.contourArea(contours[idx]))
    max_cnt_id = cnts.index(max(cnts))
    # print("max_cnt_are:", cnts[max_cnt_id])
    if cnts[max_cnt_id] > 10000:
        # fit the max contour into rectangle
        rect = cv2.minAreaRect(contours[max_cnt_id])
        rect = list(rect)
        # get the absolute value of the angle
        if abs(rect[2]) < 5:
            rect[1] = (rect[1][0], rect[1][1] - (rect[1][1] * 0.15))
        else:
            rect[1] = (rect[1][0] - (rect[1][0] * 0.15), rect[1][1])
        # write the rectangle into the image
        # np.intp make the float point to int point
        box = np.intp(
            cv2.boxPoints(rect)
        )  # box [[x1,y1](left_bottom),[x2,y2](left_top),[x3,y3](right_top),[x4,y4](right_bottom)]
        box = order_points(box)
        # output_img = cv2.drawContours(map, [box], 0, (0, 255, 0), 3)
        list_box = box.tolist()
        # print("box:", list_box)
        # cut the image by the rectangle box points
        output_img = map[list_box[1][1]:list_box[3][1], list_box[1][0]:list_box[3][0]]
        return True, output_img
    else:
        return False, map


def find_squares(map_img):
    img = map_img
    sum = 0
    squares = []
    block_list = []
    w, h = img.shape[:-1]
    # 这张图像的总大小
    img_area = w * h
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 3)
    bin = cv2.Canny(blur, 10, 30, apertureSize=3)
    contours, _hierarchy = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print("轮廓数量：%d" % len(contours))

    # 轮廓遍历
    # for cnt in contours:
    for idx, cnt in enumerate(contours):
        cnt_len = cv2.arcLength(cnt, True)  # 计算轮廓周长
        area = cv2.contourArea(cnt)
        cnt = cv2.approxPolyDP(cnt, 0.1 * cnt_len, True)  # 多边形逼近
        # 条件判断逼近边的数量是否为4，轮廓面积是否大于1000，检测轮廓是否为凸的
        if len(cnt) == 4 and cv2.isContourConvex(cnt) and area > 0.001 * img_area:
            cnt = cnt.reshape(-1, 2)
            squares.append(cnt)
            sum += 1
            M = cv2.moments(contours[idx])
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 2, (0, 255, 0), 2)
    output_img = cv2.drawContours(img, squares, -1, (0, 255, 0), 2)
    print("我找到了方块数量：", sum, squares)
    return bin, blur, output_img


def show_lcd(frame):
    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    image = image.resize((320, 240), Image.ANTIALIAS)
    image = image.filter(ImageFilter.SHARPEN)
    image = image.rotate(180)
    disp.ShowImage(image)


def button_input():
    BUTTON_PIN = 18  # 按钮连接的GPIO口
    # 选择BCM模式
    GPIO.setmode(GPIO.BCM)
    # 设置GPIO口为输入
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    press_flag = 0
    press_time = 0
    while True:
        # 如果按键被按下
        # if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        #     print('Button is not pressed')
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.1)  # 按键消除抖动
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                press_flag = 1
                # print('Button is pressed')
                press_time += 1
        if GPIO.input(BUTTON_PIN) == GPIO.HIGH and press_flag == 1:
            time.sleep(0.1)
            if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
                print("short_press")
                button = "short_press"
                break
        if press_time > 10:
            print("long_press")
            button = "long_press"
            break
        # print(press_time)
        # 暂停一段时间
        time.sleep(0.1)
    # GPIO.cleanup()  # 不能清楚因为本来有使用GPIO
    return button


def get_loc(cam):
    """
    根据摄像头捕获的图像进行四个角点识别、透视变换、宝藏识别。如果识别成功，图像就会冻结，
    如果发现识别出来的宝藏不对，就点一下按钮，就会继续捕获图像。
    如果发现识别出来的宝藏正确，那就长按按钮，即可结束这个函数并且得到所有宝藏的位置。
    :return:
    """
    T = Treasure(1)

    # cap = cv2.VideoCapture("./videos/output.avi")
    # cap = cv2.VideoCapture(-1) # -1就是使用默认摄像头 防止报错
    while True:
        success, img = cam.read()
        # print(img.shape)  # (480, 640, 3)
        if success is False:
            continue
        show_lcd(img)
        flag, points = find_scaling_points(img)
        if flag:
            flag2, new_img = perspective_trans(points, img)
            flag3, new_img2 = crop_map_img(new_img)
            if flag3:
                # cv2.imshow("crop", new_img2)
                new_img3, loc = T.FindTreasure(new_img2, output_mode=1)
                # print(len(loc))
                if len(loc) == 8:
                    show_lcd(new_img3)
                    # cv2.imshow("treasure", new_img3)
                    # key = input("输入y表示接受这个识别结果")
                    # key = button_input()
                    # if key == "short_press":
                    #     # 长按就表示这个图像是可以用的，那就使用这次识别出来的坐标
                    #     # print(loc)
                    #     img_success = cv2.imread("/home/pi/Desktop/guangshe2023/main_program/util/success.jpg")
                    #     if img_success is None:
                    #         print("未成功加载图片")
                    #     show_lcd(img_success)
                    #     return loc
                    #
                    # elif key == "long_press":
                    #     # 如果只是单击以下按键，那就跳过这张图片继续进行识别
                    #     continue
                    # show_lcd(cv2.imread("/home/pi/Desktop/guangshe2023/main_program/util/success.jpg"))
                    return loc, new_img3
        cv2.waitKey(25)  # 按数字0就是前进1帧


if __name__ == '__main__':
    cap = cv2.VideoCapture("output.avi")
    while True:
        success, img = cap.read()
        cv2.imshow("origin", img)
        # full_map, blur, outputt = find_squares(img)
        flag, points = find_scaling_points(img)
        if flag:
            # cv2.imshow("edges", full_map)
            # cv2.imshow("blur", blur)
            # cv2.imshow("draw", outputt)
            flag2, new_img = perspective_trans(points, img)
            flag3, new_img2 = crop_map_img(new_img)
            new_img2 = \
                cv2.threshold(cv2.cvtColor(new_img2, cv2.COLOR_BGR2GRAY), 0, 225, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[
                    1]
            if flag3:
                cv2.imshow("origin", img)
                cv2.imshow("catch", new_img)
                cv2.imshow("contours", new_img2)
                cv2.waitKey(0)  # 按数字0就是前进1帧
