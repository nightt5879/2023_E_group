import cv2
import numpy as np
from scipy.spatial.distance import cdist


class FineTune:
    def __init__(self, img21):
        self.ori_img = self.get_ori_img(img21)

    def simulate_lidar_scan(self, maze, start_pos, scan_angle=360, scan_resolution=1):
        """
        模拟激光雷达的目标图像
        :param maze:
        :param start_pos: 小车当前的位置坐标
        :param scan_angle: 激光雷达扫描范围。默认360度
        :param scan_resolution:
        :return:
        """
        # 计算结果图中黑色像素需要偏移的位置
        shift_x = 200 - start_pos[0]
        shift_y = 200 - start_pos[1]
        # 获取地图尺寸
        height, width = maze.shape

        # 初始化扫描结果数组
        scan_result = np.full((height, width), 255, dtype=np.uint8)

        # 起始位置
        x, y = start_pos

        # 角度范围和步长
        angles = np.arange(0, scan_angle, scan_resolution)

        for angle in angles:
            # 计算当前角度下的扫描线的坐标
            theta = np.deg2rad(angle)
            x_coords = x + np.cos(theta) * np.arange(0, width)
            y_coords = y + np.sin(theta) * np.arange(0, height)

            for i in range(len(x_coords)):
                # 检查坐标点是否在地图范围内
                if x_coords[i] >= 0 and x_coords[i] < width and y_coords[i] >= 0 and y_coords[i] < height:
                    row = int(y_coords[i])
                    col = int(x_coords[i])

                    # 如果遇到障碍物，记录距离，否则设置为最大值
                    if maze[row, col] == 0:
                        scan_result[row + shift_y, col + shift_x] = 0
                        break

        return scan_result

    def get_scan(self, now_xy):
        """
        转换now_xy成实际坐标。输入到模拟函数。最终得到模拟的激光雷达扫描图
        :param now_xy:
        :return:
        """
        x_t = int(40 * ((now_xy[0] - 1) / 2) + 20)
        y_t = int(400 - (40 * ((now_xy[1] - 1) / 2) + 20))
        scan = self.simulate_lidar_scan(self.ori_img, (x_t, y_t))
        return scan

    def get_ori_img(self, img21):
        """
        根据21*21图像得到真实大小的迷宫图
        :param img21:
        :return:
        """
        ori_img = np.ones((401, 401), dtype=np.uint8) * 255
        for i in range(1, 20, 2):
            for j in range(1, 20, 2):
                x_top = ((i - 1) // 2) * 40
                x_bottom = x_top + 40
                y_left = ((j - 1) // 2) * 40
                y_right = y_left + 40
                if img21[i - 1, j] == 0:
                    ori_img[x_top, y_left:y_right] = 0
                if img21[i + 1, j] == 0:
                    ori_img[x_bottom, y_left:y_right] = 0
                if img21[i, j - 1] == 0:
                    ori_img[x_top:x_bottom, y_left] = 0
                if img21[i, j + 1] == 0:
                    ori_img[x_top:x_bottom, y_right] = 0
        return ori_img

    def calculate_similarity(self, image1, image2):
        # 图像1的黑色像素的坐标点
        black1 = np.transpose(np.where(image1 == 0))
        # 图像2的黑色像素坐标点
        black2 = np.transpose(np.where(image2 == 0))

        # 计算距离矩阵
        distance_matrix = cdist(black1, black2)

        # 计算总距离
        total_distance = np.mean(np.min(distance_matrix, axis=1))

        return total_distance

    def rotate_image(self, image, angle, center):
        """
        旋转图像
        :param image:
        :param angle:
        :param center:
        :return:
        """
        # 获取图像尺寸
        height, width = image.shape[:2]

        # 计算旋转矩阵
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)

        # 执行图像旋转
        rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height), borderMode=cv2.BORDER_CONSTANT,
                                       borderValue=(255, 255, 255))
        _, rotated_image = cv2.threshold(rotated_image, 127, 255, cv2.THRESH_BINARY)
        return rotated_image

    def translate_image(self, image, shift_x, shift_y):
        # 定义平移矩阵
        M = np.float32([[1, 0, shift_x],
                        [0, 1, shift_y]])

        # 应用仿射变换
        translated_image = cv2.warpAffine(image, M, (image.shape[1], image.shape[0]), borderMode=cv2.BORDER_CONSTANT,
                                          borderValue=(255, 255, 255))

        return translated_image


    def delete_white(self, img, scan_img):
        """
        删除白色的周围
        :param img:
        :param scan_img:
        :return:
        """
        index = None
        for i in range(200):
            if (img[i, :] == 255).all() and (img[:, i] == 255).all() and (img[399 - i, :] == 255).all() and (
                    img[:, 399 - i] == 255).all():
                continue
            index = i
            break
        index2 = None
        for i in range(200):
            if (scan_img[i, :] == 255).all() and (scan_img[:, i] == 255).all() and (scan_img[399 - i, :] == 255).all() and (
                    scan_img[:, 399 - i] == 255).all():
                continue
            index2 = i
            break
        index = min([index,index2])
        return img[index:400-index,index:400-index],  scan_img[index:400-index,index:400-index]

    def get_angle_dis(self, now_xy, img, angle_range=180, cal_dis=False,
                      dis_range=(-10, 10)):
        """
        获得小车需要旋转的角度。逆时针为正
        :param img: 输入由激光雷达绘制出来的图
        :param scan_img: 输入模拟激光雷达的理想图
        :param angle_range: 旋转角度范围，如果觉得小车旋转误差不大的话可以调小这个值
        :param cal_dis: 是否计算平移距离，默认不计算，只微调角度
        :param dis_range: 平移距离的范围
        :return:
        """
        sim_scan_img = self.get_scan(now_xy)
        sim_scan_img, img = self.delete_white(sim_scan_img, img)
        center = (img.shape[0]//2, img.shape[0]//2)
        # center = (200, 200)
        # 存储
        similarity_list = []
        for angle in range(-angle_range, angle_range):
            # 逆时针旋转
            result = self.rotate_image(img, angle, center)
            similarity_list.append(self.calculate_similarity(result, sim_scan_img))

        min_angle = np.argmin(similarity_list) - angle_range
        rotate_img = self.rotate_image(img, min_angle, center)

        if cal_dis:
            dis_list_x = []
            for shift_x in range(dis_range[0], dis_range[1]):
                translated_image = self.translate_image(rotate_img, shift_x, 0)
                dis_list_x.append(self.calculate_similarity(translated_image, sim_scan_img))

            min_x = np.argmin(dis_list_x) - 10
            dis_list_y = []
            for shift_x in range(dis_range[0], dis_range[1]):
                translated_image = self.translate_image(rotate_img, min_x, 0)
                dis_list_y.append(self.calculate_similarity(translated_image, sim_scan_img))
            min_y = np.argmin(dis_list_y) - 10

            translated_image = self.translate_image(rotate_img, min_x, min_y)
            return min_angle, (min_x, min_y), translated_image
        return min_angle, (0, 0), rotate_img


if __name__ == '__main__':
    # 21 * 21的迷宫图,在一开始识别地图的时候会得到
    img = cv2.imread("../img/small_labyrinth.png", 0)
    # 小车当前位置
    now_xy = (3, 1)
    # 定义一个角度距离微调对象
    fineTuner = FineTune(img)
    # 用于测试的图片
    test_img = cv2.imread("../../img/test2.jpg", 0)

    # 对测试图片进行二值化
    _, test_img = cv2.threshold(test_img, 127, 255, cv2.THRESH_BINARY)
    # 计算需要微调的角度和距离（默认不计算距离）
    angle, (min_x, min_y), translated_image = fineTuner.get_angle_dis(now_xy, test_img)
    # 打印出小车需要微调的距离,角度值是正的,代表逆时针旋转的度数
    print(angle)

