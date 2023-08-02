import cv2
import numpy as np
from json import dumps


class MapArchRecognizer:
    def __init__(self, img):
        # 识别出来的地图

        # 先resize到500 * 500
        img = cv2.resize(img, (500, 500))

        # 先转成灰度图
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 转成二值化图像
        ret, self.binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))  # 可根据需要调整结构元素的大小
        self.binary = cv2.morphologyEx(self.binary, cv2.MORPH_OPEN, kernel)

    def getByX(self, binary):
        """
        从x方向逐行扫描

        :return:
        """
        # 定义一个列表来存储墙壁位置
        black = []
        zero_array = np.zeros((21, 21), dtype=np.uint8)
        for y in range(0, 10):
            black.append([])
            for x in range(500):
                if binary[25 + y * 50, x] == 0:
                    for uni in range(1, 10):
                        if 8 > abs(uni * 50 - x):
                            black[y].append(uni)
                            break

            black[y] = sorted(list(set(black[y])))
        migon = []
        for index, row_list in enumerate(black):
            left = 0
            block = 0
            migon.append([])
            for block in row_list:
                for num in range(2 * (block - left) - 1):
                    migon[index].append(255)
                migon[index].append(0)
                left = block
            for num in range(2 * (10 - block) - 1):
                migon[index].append(255)

        for index, pix in enumerate(migon):
            for color_index, color in enumerate(pix):
                zero_array[2 * index + 1, color_index + 1] = color
        return zero_array

    def getByY(self, binary):
        """
        从y方向逐列扫描
        :return:
        """
        # 定义一个列表来存储墙壁位置
        black = []
        zero_array = np.zeros((21, 21), dtype=np.uint8)
        for x in range(0, 10):
            black.append([])
            for y in range(500):
                if binary[y, 25 + x * 50] == 0:
                    for uni in range(1, 10):
                        if 10 > abs(uni * 50 - y):
                            black[x].append(uni)
                            break
            black[x] = sorted(list(set(black[x])))

        migon = []
        for index, row_list in enumerate(black):
            left = 0
            block = 0
            migon.append([])
            for block in row_list:
                for num in range(2 * (block - left) - 1):
                    migon[index].append(255)
                migon[index].append(0)
                left = block
            for num in range(2 * (10 - block) - 1):
                migon[index].append(255)

        for index, pix in enumerate(migon):
            for color_index, color in enumerate(pix):
                zero_array[color_index + 1, 2 * index + 1] = color
        return zero_array

    def analysis_map(self):
        zero_array_x = self.getByX(self.binary)
        zero_array_y = self.getByY(self.binary)
        new_zero = np.ones_like(zero_array_x) * 255
        for x in range(21):
            for y in range(21):
                if zero_array_x[x, y] == 0 and zero_array_y[x, y] == 0:
                    new_zero[x, y] = 0
        return new_zero


if __name__ == '__main__':
    # 输入图片。不管这个地图有没有宝藏都行
    img = cv2.imread(f"test.png")
    # 实例化一个地图架构解析对象
    mapAnalysiser = MapArchRecognizer(img)
    # 转换地图得到21 * 21的矩阵
    map_array = mapAnalysiser.analysis_map()
