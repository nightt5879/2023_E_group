from os import popen
from time import time

result = None


class Lidar:
    def __init__(self, model_path="../model/ultra_simple --channel --serial /dev/ttyAMA0 115200"):
        self.result = popen(model_path)
        for i in range(6):
            self.result.buffer.readline().decode("utf8")


    def get_each(self):
        # 获取theta字符串所在位置的索引
        index_theta = 10
        # 读取一行数据，数据类似："   theta: 6.80 Dist: 00133.00 Q: 47"
        line = self.result.buffer.readline().decode("utf8")
        # 获取Dist字符串所在位置索引
        index_Dist = line.find("Dist")
        # 获取Q字符串所在位置索引
        index_Q = line.rfind("Q")
        # 使用刚刚得到的索引来获取角度并转成int类型，而且角度在0到359范围内
        
        angle = min([359,int(float(line[index_theta:index_Dist-1]))])
        # 根据刚刚得到的索引来获取u建立并转成int类型
        distance = int(line[index_Dist+6:index_Q-4])
        return angle,distance

    def get_data(self):
        scan_data = [0] * 360
        # 获取上一次循环得到的角度
        last_angle = 0
        # 获取上一次循环得到的距离
        last_distance = 1
        while True:
            # 获取单条数据
            angle, distance = self.get_each()
            if angle == 0:
                # 把刚刚获取的距离根据角度放入列表的对应位置
                scan_data[angle] = distance if distance != 0 else last_distance
                while True:
                    # 获取单条数据
                    angle, distance = self.get_each()
                    # 把刚刚获取的距离根据角度放入列表的对应位置
                    scan_data[angle] = distance if distance != 0 else last_distance
                    if angle == 359 and last_angle != 359:
                        return scan_data
                    last_angle = angle
                    last_distance = distance
                
if __name__ == '__main__':
    my_lidar = Lidar(model_path="./model/ultra_simple --channel --serial /dev/ttyAMA0 115200")
    while True:
        print(my_lidar.get_data())
