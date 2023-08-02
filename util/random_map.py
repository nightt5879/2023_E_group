import cv2
from random import *
import numpy as np


class Treasure():
    """ 宝藏函数
    注意这里描述坐标和CV2一样，左上角为原点 往右和往下分别是x和y的正方向
    Attributes:
        self.origin_map: 原始输入的地图
        self.x: 方块区域x坐标
        self.y: 方块区域y坐标
    """

    def __init__(self):
        """ 随机宝藏初始化
        :return: None
        """
        # 将地图化为10*10个方块 分为了100个区域用正交坐标唯一表示 注意到地图是中心对称分布，先生成一半再生成另一半
        self.origin_map = None
        points = [(2, 9), (3, 3), (3, 6), (3, 10), (4, 1), (4, 5), (5, 9)]
        shuffle(points)
        choose_points = points[:4]

        self.x = [point[0] for point in choose_points]
        self.y = [point[1] for point in choose_points]

        for i in range(4):
            symmetry_x = 11 - self.x[i]
            symmetry_y = 11 - self.y[i]
            self.x.append(symmetry_x)
            self.y.append(symmetry_y)
        self.all_points = list(zip(self.x, self.y))

    def DrawTreasure(self, img_address):
        """ 画宝藏位置

        Args:
            :param img_address:输入图像的地址
        :return:
            draw_map: 画好宝藏的地图
        """
        self.origin_map = cv2.imread(img_address)  # 读取图像
        draw_map = cv2.imread(img_address)  # 保存一份原图
        size = self.origin_map.shape  # 读取图像的尺寸
        length = size[1]  # 图像长度
        height = size[0]  # 图像高度
        # xy轴拆分成为10份的单位长度
        x_stepping = int(length / 10)
        y_stepping = int(height / 10)
        # 定位的点是小方块的右下角 需要修正到小方块的中心
        x_correction = int(length / 20)
        y_correction = int(height / 20)
        # 椭圆xy方向的轴长 0.4 这个系数是看图像看的大概 椭圆xy轴长/单个小方格的xy = 0.4
        ellipse_x = int(0.18 * x_stepping)
        ellipse_y = int(0.18 * y_stepping)

        for i in range(8):
            # 画8个宝藏的位置 ellipse里面参数 第一个括号两个坐标 第二个括号xy方向轴长 0 0 360 角度相关不管 （0，0，0）颜色 -1 填充
            cv2.ellipse(draw_map, (self.x[i] * x_stepping - x_correction, self.y[i] * y_stepping - y_correction),
                        # 坐标步长*第几个 - 修正坐标
                        (ellipse_x, ellipse_y), 0, 0, 360, (0, 0, 0), -1)
        return draw_map, self.all_points

    def FindTreasure(self, img):
        """ 找宝藏

        Args:
            img: 输入图像

        Returns:
            img: 画好的图像
            location: 宝藏所在位置的xy坐标
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)  # contours为轮廓集，可以计算轮廓的长度、面积等
        location = []  # 保存坐标的数组
        for cnt in contours:
            if len(cnt) > 5:  # 点的数量够多，滤噪点
                S1 = cv2.contourArea(cnt)  # 封闭曲面的面积
                ell = cv2.fitEllipse(cnt)
                """
                返回值：ellipse = [ (x, y) , (a, b), angle ]
                （x, y）代表椭圆中心点的位置
                （a, b）代表长短轴长度，应注意a、b为长短轴的直径，而非半径
                angle 代表了中心旋转的角度
                """
                contour = cnt.reshape(-1, 2)
                # 找出轮廓坐标中x的最大值
                max_x = np.max(contour[:, 0])
                # 找出轮廓坐标中x的最小值
                min_x = np.min(contour[:, 0])
                # 找出轮廓坐标中y的最大值
                max_y = np.max(contour[:, 1])
                # 找出轮廓坐标中y的最大值
                min_y = np.min(contour[:, 1])

                # 计算轮廓最大xy边长 （即外接矩形长宽）
                contour_x = max_x - min_x
                contour_y = max_y - min_y
                # S2 =math.pi*ell[1][0]*ell[1][1]
                size = thresh.shape  # 读取图像的尺寸
                length = size[1]  # 图像长度
                height = size[0]  # 图像高度
                block_x = length / 10
                block_y = height / 10
                block_s = (length * height) / 100  # 每一个小方块的面积
                # S=π(圆周率)×a×b  ab的比上小方块边长约为0.39，即“半径”为0.18 比值为 (π * 0.18 * 0.18 )/ 1 算出为0.102 取0.09（放宽下限）
                min = 0.09 * block_s
                max = 0.2 * block_s  # 随便给的
                if (min < S1 < max) and \
                        (0.15 * block_x < contour_x < 0.5 * block_x) and \
                        (0.15 * block_y < contour_y < 0.5 * block_y):  # 判断是否满足条件 分别是面积 xy轴长 后面两个直接写里面了
                    cv2.ellipse(img, ell, (0, 255, 0), 2)
                    location.append(ell[0])
                # print(str(S1) + "    " + str(S2)+"   "+str(ell[0][0])+"   "+str(ell[0][1]))
        return img, location

if __name__ == '__main__':

    for i in range(10):
        a = Treasure()
        img, all_points = a.DrawTreasure("origin.jpg")
        cv2.imshow("img", cv2.resize(img, (500, 500)))
        cv2.waitKey(100)
