import cv2
import numpy as np
import math
import itertools
from copy import deepcopy
import random


def get_obs(small_labyrinth_file="./img/small_labyrinth.png"):
    """
    读取读取压缩后的迷宫图片,获取迷宫墙壁。
    用于后续路径规划
    :param small_labyrinth_file: 迷宫图片文件所在位置
    :return:
    """
    # 读取图片
    img = cv2.imread(small_labyrinth_file, 0)
    # 获取黑色像素即墙壁的索引位置
    zero = np.where(img == 0)
    # 生成墙壁坐标点, 由于numpy矩阵起点是左上角, matplotlib画图的起点是左下角, 所以y方向上要转换坐标
    obs = set((ys, 20 - xs) for xs, ys in zip(zero[0], zero[1]))
    return obs


if __name__ == '__main__':
    obs = get_obs(small_labyrinth_file="../img/small_labyrinth.png")
else:
    obs = get_obs()


class Env:
    def __init__(self):
        self.x_range = 21
        self.y_range = 21
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = obs

    def update_obs(self, obs):
        self.obs = obs


class ADStar:
    def __init__(self, s_start, s_goal, car_direct, just_get_distance=False, eps=2.5, heuristic_type="euclidean"):
        """
        Anytime D*路径规划算法
        :param s_start: 起点坐标, 一般情况类似(1, 3)但是有时候会出现类似[(1, 3), "上"], 最后一个是用于让小车到达宝藏位置后朝向宝藏, 对于起点坐标来说没用
        :param s_goal:  终点坐标, 同上, 但是朝向是有用的, 会存储在self.target_direct中
        :param car_direct: 小车初始朝向, 上、下、左、右
        :param plot: 是否绘图, 在调试的时候推荐绘制, 可以可视化路径规划, 实际的时候为了提高运行速度, 不绘制
        :param just_get_distance: 是否是仅获取两个坐标点之间的距离而不获取具体运动指令, 一般在旅行商问题暴力枚举算法计算宝藏之间距离的时候设置为True
        :param eps: 路径规划的误差阈值, 当达到这个误差范围内的时候视为达到目标, 即可完成路径规划, 一般设置为2.5就可以了不需要变动
        :param heuristic_type: 同上, 不需要变动
        """
        # 下面有些变量在上面注释的时候已经解释了。解释过的就不会再进行注释
        self.s_start = s_start[0] if isinstance(s_start[-1], str) else s_start
        self.s_goal = s_goal[0] if isinstance(s_goal[-1], str) else s_goal
        self.car_direct = car_direct
        self.target_direct = s_goal[1]
        self.heuristic_type = heuristic_type
        self.just_get_distance = just_get_distance

        # 下面的内容就是初始化迷宫地图以及路径规划参数和变量了, 都不需要变动
        self.Env = Env()
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.g, self.rhs, self.OPEN = {}, {}, {}

        for i in range(1, self.Env.x_range - 1):
            for j in range(1, self.Env.y_range - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.s_goal] = 0.0
        self.eps = eps
        self.OPEN[self.s_goal] = self.Key(self.s_goal)
        self.CLOSED, self.INCONS = set(), dict()

    def run(self):

        self.ComputeOrImprovePath()
        path, total_distance, car_direct = self.get_path(
            [(x, y) for (x, y) in self.extract_path() if x % 2 == 1 and y % 2 == 1])

        return path, total_distance, car_direct

    def ComputeOrImprovePath(self):
        while True:
            s, v = self.TopKey()
            if v >= self.Key(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:
                break
            self.OPEN.pop(s)
            if self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                self.CLOSED.add(s)
                for sn in self.get_neighbor(s):
                    self.UpdateState(sn)
            else:
                self.g[s] = float("inf")
                for sn in self.get_neighbor(s):
                    self.UpdateState(sn)
                self.UpdateState(s)

    def UpdateState(self, s):
        if s != self.s_goal:
            self.rhs[s] = float("inf")
            for x in self.get_neighbor(s):
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
        if s in self.OPEN:
            self.OPEN.pop(s)

        if self.g[s] != self.rhs[s]:
            if s not in self.CLOSED:
                self.OPEN[s] = self.Key(s)
            else:
                self.INCONS[s] = 0

    def Key(self, s):
        if self.g[s] > self.rhs[s]:
            return [self.rhs[s] + self.eps * self.h(self.s_start, s), self.rhs[s]]
        else:
            return [self.g[s] + self.h(self.s_start, s), self.g[s]]

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.OPEN, key=self.OPEN.get)
        return s, self.OPEN[s]

    def h(self, s_start, s_goal):
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_start]
        s = self.s_start

        for k in range(200):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_goal:
                break

        return list(path)

    def get_path(self, path):
        """
        根据计算出来的路径, 生成运动指令, 路径长度, 小车最终朝向, 如果只需要距离的话, 运动指令和最终朝向都是None
        :param path: 使用路径规划算法计算出来的路径小车的详细运动指令、路径长度、小车最终朝向
        :return: 返回运动指令、路径长度、小车最终朝向
        """
        # 定义两个列表用于存储路径的所有x值和y值
        px = []
        py = []
        for x, y in path:
            px.append(x)
            py.append(y)

        # 如果只是计算距离, 不需要小车详细运动细节以及最终小车朝向, 那就仅仅返回路径长度
        if self.just_get_distance:
            return None, len(px) - 1, None

        # 获取小车详细运动细节以及小车最终的朝向, 这个朝向不一定是最终可用的朝向, 后续会进行修正
        directs, car_direct = self.get_directs(px, py)

        # 如果self.target_direct是字符串的话, 就是存储小车最终朝向, 如果路径规划后小车朝向与其不一致, 就需要添加运动指令（左转或者右转）, 修正小车最终朝向
        if car_direct != self.target_direct and isinstance(self.target_direct, str):
            last_distance = get_distance(directs[-1]['target_xy'][0], directs[-1]['target_xy'][1], self.target_direct)
            directs.append({
                'now_distance': directs[-1]['target_distance'],
                'target_distance': last_distance,
                'now_xy': directs[-1]['target_xy'],
                'target_xy': directs[-1]['target_xy']
            })
            List = ['上', '右', '下', '左']
            ori_index = List.index(car_direct)
            tar_index = List.index(self.target_direct)
            car_direct = self.target_direct
            if tar_index - ori_index in [-1, 3]:
                directs[-1]['move_mode'] = "左转"

            elif tar_index - ori_index in [1, -3]:
                directs[-1]['move_mode'] = "右转"
            else:
                print("这边出错了")
                exit()

        # 存储小车所有走过的坐标点, 用于可视化小车走完全程

        return directs, len(px) - 1, car_direct

    def get_directs(self, px, py):
        """

        方向：0是上。1是右。2是下。3是左
        :param px: 存储路径规划的路线中的所有x值
        :param py: 存储路径规划的路线中所有y值
        :return: 返回字典类型的小车详细运动指令以及字符串类型的小车最终朝向
        """
        directs = []
        car_direct = self.car_direct
        for i in range(len(px) - 1):

            if px[i] > px[i + 1]:
                # 向左
                if car_direct == "下":
                    directs.append({
                        "move_mode": "右转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "左"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '上':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "左"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '右':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "上"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], "上"),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "左"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                car_direct = "左"
            elif px[i + 1] > px[i]:
                # 向右
                if car_direct == "下":
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "右"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '上':
                    directs.append({
                        "move_mode": "右转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "右"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '左':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "下"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], "下"),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "右"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                car_direct = "右"
            elif py[i] > py[i + 1]:
                # 向下
                if car_direct == "左":
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "下"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })

                elif car_direct == '右':
                    directs.append({
                        "move_mode": "右转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "下"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '上':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "左"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], "左"),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "下"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                car_direct = "下"
            elif py[i + 1] > py[i]:
                # 向上
                if car_direct == "左":
                    directs.append({
                        "move_mode": "右转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "上"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '右':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "上"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                elif car_direct == '下':
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], car_direct),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "右"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])
                    })
                    directs.append({
                        "move_mode": "左转",
                        # 获取当前位置小车前后左右的距离
                        "now_distance": get_distance(px[i], py[i], "右"),
                        # 获取完成move_mode操作后小车前后左右的距离
                        "target_distance": get_distance(px[i], py[i], "上"),
                        "now_xy": (px[i], py[i]),
                        "target_xy": (px[i], py[i])

                    })
                car_direct = "上"
            directs.append({
                "move_mode": "前进",
                # 获取当前位置小车前后左右的距离
                "now_distance": get_distance(px[i], py[i], car_direct),
                # 获取完成move_mode操作后小车前后左右的距离
                "target_distance": get_distance(px[i + 1], py[i + 1], car_direct),
                "now_xy": (px[i], py[i]),
                "target_xy": (px[i + 1], py[i + 1])
            })

        return directs, car_direct


def get_distance(now_x, now_y, car_direct):
    """
    获取小车当前坐标上右下左的距离。其中上是指小车的朝向位置
    返回的距离单位是白色块。0表示紧挨着墙壁。None表示那个方向没有任何障碍物
    :param now_x: x坐标
    :param now_y: y坐标
    :param car_direct: 小车朝向
    :return: 返回一个四元组。分别是上右下左的距离。
    """
    # 上右下左
    abs_distance = [None, None, None, None]
    for x, y in obs:
        if x == now_x:
            distance = (y - now_y)
            if distance > 0:
                distance = (distance - 1) // 2
                if abs_distance[0] is None or abs_distance[0] > distance:
                    abs_distance[0] = distance
            else:
                distance = ((-distance) - 1) // 2
                if abs_distance[2] is None or abs_distance[2] > (distance):
                    abs_distance[2] = distance
        elif y == now_y:
            distance = (x - now_x)
            if distance > 0:
                distance = (distance - 1) // 2
                if abs_distance[1] is None or abs_distance[1] > distance:
                    abs_distance[1] = distance
            else:
                distance = ((-distance) - 1) // 2
                if abs_distance[3] is None or abs_distance[3] > distance:
                    abs_distance[3] = distance

    if car_direct == '右':
        relate_distance = abs_distance[1:] + [abs_distance[0]]
    elif car_direct == '上':
        relate_distance = abs_distance

    elif car_direct == '下':
        relate_distance = abs_distance[2:] + abs_distance[:2]
    else:
        relate_distance = [abs_distance[-1]] + abs_distance[:-1]
    return tuple(relate_distance)


def get_path(s_start, s_goal, car_direct, just_get_distance=False):
    """
    获取路径、路径长度、小车最终朝向
    :param s_start: 起点坐标
    :param s_goal: 终点坐标
    :param car_direct: 小车当前朝向
    :param plot: 是否画图
    :param just_get_distance: 是否是仅需要路径长度
    :return:
    """
    path, total_distance, car_direct = ADStar(s_start, s_goal, car_direct,
                                              just_get_distance=just_get_distance).run()
    return path, total_distance, car_direct


def get_distances(begin_point: tuple, target_point: tuple, mine_points: list) -> dict:
    """
    获取8个宝藏＋起点终点共10个点的两两之间的距离
    这是一个旅行商问题, 用于寻找全局最短路径, 由于宝藏数量较少, 所以解决这个旅行商问题使用的是暴力枚举法, 直接遍历所有可能的路径, 当然起点和终点是固定的
    :param begin_point: 起点坐标
    :param target_point:终点坐标
    :param mine_points:所有宝藏的坐标
    :return: 返回一个字典, 存放着所有边的距离
    """
    # 定义一个字典用于存储所有距离
    distance_dict = {}
    num_mines = len(mine_points)
    # 先计算起点和8个宝藏的距离, 计算终点和8个宝藏的距离
    for i in range(num_mines):
        # 起点到每个点之间的距离
        distance_dict[(i, -1)] = \
            get_path(mine_points[i], begin_point, car_direct='上', just_get_distance=True)[1]

        # 每个点到终点的距离
        distance_dict[(i, -2)] = \
            get_path(mine_points[i], target_point, car_direct='上', just_get_distance=True)[1]

    # 计算8个宝藏两两之间的距离
    for i in range(num_mines - 1):
        for j in range(i + 1, num_mines):
            distance_dict[tuple(sorted([i, j]))] = \
                get_path(mine_points[i], mine_points[j], car_direct='上', just_get_distance=True)[1]

    return distance_dict


def brute_force_tsp(begin_point: tuple, target_point: tuple, mine_points: list) -> tuple:
    """
    暴力枚举解决旅行商问题
    :param begin_point: 起点坐标
    :param target_point: 终点坐标
    :param mine_points: 所有宝藏的坐标
    :return: 返回一个元组类似(6, 5, 7, 4, 1, 3, 0, 2)
    """
    # 先获取所有距离
    distance_dict = get_distances(begin_point, target_point, mine_points)

    # 初始化最小距离为无穷大
    min_dist = float('inf')
    # 初始化最短路径为None
    shortest_path = None
    # 暴力枚举遍历所有可能的路径, 计算路径总长度,最终得到最短的路径
    # print(mine_points)
    for path in itertools.permutations(range(len(mine_points))):  # 遍历所有可能的排列，即所有可能的路径
        cur_dist = sum(distance_dict[tuple(sorted([path[i], path[i + 1]]))] for i in range(len(path) - 1))  # 计算该路径的总距离
        cur_dist += distance_dict[(path[0], -1)] + distance_dict[(path[-1], -2)]
        if cur_dist < min_dist:
            min_dist = cur_dist
            shortest_path = path  # 如果该路径小于min_dist，更新min_dist和shortest_path
    return shortest_path, min_dist


def get_total_path(begin_point: tuple, target_point: tuple, mine_points: list, shortest_path, car_direct="上") -> list:
    """

    :param begin_point: 起点坐标
    :param target_point: 终点坐标
    :param mine_points: 所有宝藏位置
    :param car_direct: 小车初始朝向
    :return:
    """

    # 定义一个列表存储所有路径。最终里面会有9个列表代表9条路线。每个列表里面有很多个字典存储小车详细运动指令
    total_path = []

    # 先获取起点和第一个宝藏之间的路径、路径长度、小车最终朝向
    path, total_distance, car_direct = get_path(begin_point, mine_points[shortest_path[0]], car_direct)
    # 把得到的路径添加到列表中
    total_path.append(path)
    # 遍历所有宝藏得到每个宝藏到下一个宝藏之间的路径、路径长度和小车最终朝向
    for index in range(len(shortest_path) - 1):
        path, total_distance, car_direct = get_path(mine_points[shortest_path[index]],
                                                    mine_points[shortest_path[index + 1]], car_direct, )
        total_path.append(path)
    # 获取终点和第一个宝藏之间的路径、路径长度、小车最终朝向
    path, total_distance, car_direct = get_path(mine_points[shortest_path[-1]], target_point, car_direct, )
    total_path.append(path)

    return total_path


def fine_tune_mine(begin_point, target_point, mine_points: list, optimize, ori_mine_points):
    """
    计算每个宝藏距离周围墙壁的距离, 偏移宝藏的坐标点以防止小车直接与宝藏相撞, 并且记录小车最终正确朝向
    :param begin_point:
    :param target_point:
    :param mine_points: 所有宝藏位置
    :param optimize:
    :param ori_mine_points: 后面需要修改宝藏位置, 所以先把原有宝藏位置存储下来
    :return:
    """

    if optimize is False:

        for i in range(len(mine_points)):
            dis = get_distance(mine_points[i][0], mine_points[i][1], "上")

            if dis[0] != 0:
                # 向上。y+1
                mine_points[i] = [(mine_points[i][0], mine_points[i][1] + 2), "下"]
            elif dis[2] != 0:
                # 向右 。x+1
                mine_points[i] = [(mine_points[i][0], mine_points[i][1] - 2), "上"]
            elif dis[1] != 0:
                # 向下 y-1
                mine_points[i] = [(mine_points[i][0] + 2, mine_points[i][1]), "左"]
            elif dis[3] != 0:
                # 向左 x-1
                mine_points[i] = [(mine_points[i][0] - 2, mine_points[i][1]), "右"]
            else:
                print("出错了")
                exit()
        # 把原有宝藏的位置也设置成墙壁, 防止小车穿越宝藏点

        obs.update(set(ori_mine_points))
        # 使用暴力枚举法解决旅行商问题求出小车前往的宝藏顺序
        shortest_path, min_dist = brute_force_tsp(begin_point, target_point, mine_points)
        return shortest_path, mine_points
    else:
        # 记录那些周围白色块不止一个的宝藏的索引以及它周围白色块的个数
        index_list = []
        # 记录一共有多少种宝藏摆放位置
        path_num = 1
        # 先修改那些周围只有一个白色块的宝藏的坐标
        for i in range(len(mine_points)):

            dis = get_distance(mine_points[i][0], mine_points[i][1], "上")

            white_block_num = 4 - dis.count(0)
            path_num *= white_block_num

            # print(mine_points)

            if white_block_num == 1:
                # 如果只有一个方向有白色块。那这个宝藏的偏移就固定了
                if dis[0] != 0:
                    # 向上。y+1
                    mine_points[i] = [(mine_points[i][0], mine_points[i][1] + 2), "下"]
                elif dis[1] != 0:
                    # 向右 。x+1
                    mine_points[i] = [(mine_points[i][0] + 2, mine_points[i][1]), "左"]
                elif dis[2] != 0:
                    # 向下 y-1
                    mine_points[i] = [(mine_points[i][0], mine_points[i][1] - 2), "上"]
                elif dis[3] != 0:
                    # 向左 x-1
                    mine_points[i] = [(mine_points[i][0] - 2, mine_points[i][1]), "右"]
            elif white_block_num == 0:
                print("出错了")
                exit()
            else:
                index_list.append((i, dis))
        all_mine_points = []
        for num in range(path_num):
            all_mine_points.append(mine_points.copy())

        """
        一共有两个宝藏是周围有超过一个白色块的。all_mine_points列表里面的每一个列表都需要被修改两次
        """
        Dict = {}
        for i, dis in index_list:
            if dis[0] != 0:
                # 向上。y+1
                if Dict.get(i, False) is False:
                    Dict[i] = [[(mine_points[i][0], mine_points[i][1] + 2), "下", mine_points[i]]]
                else:
                    Dict[i].append([(mine_points[i][0], mine_points[i][1] + 2), "下", mine_points[i]])

            if dis[1] != 0:
                # 向右 。x+1
                if Dict.get(i, False) is False:
                    Dict[i] = [[(mine_points[i][0] + 2, mine_points[i][1]), "左", mine_points[i]]]
                else:
                    Dict[i].append([(mine_points[i][0] + 2, mine_points[i][1]), "左", mine_points[i]])

            if dis[2] != 0:
                # 向下 y-1
                if Dict.get(i, False) is False:
                    Dict[i] = [[(mine_points[i][0], mine_points[i][1] - 2), "上", mine_points[i]]]
                else:
                    Dict[i].append([(mine_points[i][0], mine_points[i][1] - 2), "上", mine_points[i]])

            if dis[3] != 0:
                # 向左 x-1
                if Dict.get(i, False) is False:
                    Dict[i] = [[(mine_points[i][0] - 2, mine_points[i][1]), "右", mine_points[i]]]
                else:
                    Dict[i].append([(mine_points[i][0] - 2, mine_points[i][1]), "右", mine_points[i]])

        # 首先将字典的值转换为一个列表
        values_list = list(Dict.values())
        index = 0
        # 求所有可能的配对
        pair_list = list(itertools.product(*values_list))

        for pair in pair_list:
            for ls in pair:
                all_mine_points[index][all_mine_points[index].index(ls[2])] = ls[:2]
            index += 1
        obs.update(set(ori_mine_points))
        cur_dist = float("inf")
        choose_path = None
        choose_point = None
        for points in all_mine_points:
            shortest_path, min_dist = brute_force_tsp(begin_point, target_point, points)

            if cur_dist > min_dist:
                cur_dist = min_dist
                choose_path = shortest_path
                choose_point = points
        return choose_path, choose_point


def get_paths(mine_points, eight_mines, begin_point=(19, 1), target_point=(1, 19), optimize=False, car_direct='上'):
    """

    :param mine_points: 所有宝藏的位置, 这里起点是左上角并且x和y单位是白色块
    :param begin_point: 起点坐标, 右下角
    :param eight_mines: 八个宝藏。用于更新obs
    :param target_point: 终点坐标, 左上角
    :param optimize: 是否进一步优化使得路径真正最短,这会导致运行速度变慢
    :param plot: 是否画图
    :return:
    """
    ori_mine_points = mine_points.copy()

    # 计算每个宝藏距离周围墙壁的距离, 偏移宝藏的坐标点以防止小车直接与宝藏相撞, 并且记录小车最终正确朝向
    shortest_path, mine_points = fine_tune_mine(begin_point, target_point, mine_points, optimize, eight_mines)

    total_path = get_total_path(begin_point, target_point, mine_points, shortest_path, car_direct=car_direct)
    # for path in total_path:
    #     print(path[-1])
    # print("---")
    # 给原来的宝藏列表增加一个象限信息
    for i in shortest_path:
        x, y = ori_mine_points[i]

        if 10 > x:
            if 10 > y:
                mine_points[i].append(3)
            else:
                mine_points[i].append(2)
        else:
            if 10 > y:
                mine_points[i].append(4)
            else:
                mine_points[i].append(1)
    quadrant_mines = []
    ori_mines = []
    for index in shortest_path:
        quadrant_mines.append(mine_points[index])
        ori_mines.append(ori_mine_points[index])
    return total_path, quadrant_mines, ori_mines


class pathPlaner:
    def __init__(self, mine_points):
        # 把宝藏位置转换成我们所需的正确的宝藏格式
        self.eight_mines = [(x * 2 - 1, (11 - y) * 2 - 1) for (x, y) in mine_points]
        self.paths, self.quadrant_mines, self.ori_mines = get_paths(self.eight_mines.copy(), self.eight_mines.copy(),
                                                                    optimize=False)

        self.ori_paths = deepcopy(self.paths)

    def update_paths(self):
        """
        当你找到一个宝藏后,如果发现这个宝藏是我们的宝藏,那你就把paths列表作为输入参数放到这个函数里面
        这个函数就会自动为你更新你的路径,除去不需要搜索的宝藏
        :param index_mine: 当前到达第几个宝藏
        :return:
        """
        index = len(self.quadrant_mines) - len(self.paths)
        now_xy, car_direct, quadrant = self.quadrant_mines[index]
        # 删除已经遇到过的宝藏
        for i in range(index + 1):
            self.quadrant_mines.pop(0)
            self.ori_mines.pop(0)
        # 删除一些为探索过但是确定是对方宝藏的宝藏
        for i in range(len(self.ori_mines)):
            if self.quadrant_mines[i][2] == quadrant:
                self.quadrant_mines.pop(i)
                self.ori_mines.pop(i)
                break

        if len(self.quadrant_mines) == 0:

            path, _, _ = get_path(now_xy, (1, 19), car_direct)
            self.paths = [path]

        else:
            global obs
            obs = get_obs()
            self.paths, self.quadrant_mines, self.ori_mines = get_paths(self.ori_mines, self.eight_mines, optimize=False,
                                                                        begin_point=now_xy,
                                                                        car_direct=car_direct)

    def update_paths_add(self):
        pass



def car_move(move_mode):
    pass


def mine_classify():
    if random.random() > 0.5:
        return 'red', True
    return 'blue', False


def hit_mine():
    pass


if __name__ == '__main__':
    # 一开始五分钟内得到的我们的队伍颜色,蓝色或者红色
    our_color = 'red'
    # 一开始五分钟内得到的所有宝藏的位置
    mine_points = [(3, 3), (4, 5), (3, 6), (2, 9), (7, 6), (8, 5), (8, 8), (9, 2)]
    # 初始化路径规划类,其中planner.paths就是规划的路径
    planer = pathPlaner(mine_points)
    print(planer.paths)
    exit()
    while True:
        # 取总路径中第一个路径为当前要走的路径
        now_path = planer.paths.pop(0)
        for move_obj in now_path:
            # 遍历路径中的第一个指令
            move_mode = move_obj['move_mode']
            # 根据move_mode。小车做出指定动作：前进、左转、右转
            car_move(move_mode)
        # 如果路径列表为空,表示已经找完了所有宝藏了,那么就要离开迷宫了
        if len(planer.paths) == 0:
            # 让小车继续直走一段距离然后就走出迷宫了
            car_move("前进")
            break
        else:
            # 宝藏识别.返回识别到的颜色以及是否是真宝藏
            color, truth = mine_classify()
            # 如果识别出来的颜色和你的队伍颜色相同
            if color == our_color:
                # 更新以下总路径。看看同象限内是否还有其他宝藏没有搜索。如果有的话就删除那个宝藏。因为那个宝藏必定是对方的
                planer.update_paths()
                # 如果这个是真宝藏。那就前进一格撞它然后在退后一格
                if truth:
                    hit_mine()


