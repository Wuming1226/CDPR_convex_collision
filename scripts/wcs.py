import copy
import numpy as np
import time
from calculate_separation_v2 import (calculate_separation_1, calculate_separation_2, calculate_separation_3,
                                     check_new_collision, check_collision_infinite, get_distance_between_lines,
                                     calculate_cable_length)
# import calculate_separation_v1 as v1
import matplotlib.pyplot as plt


class Edge:
    def __init__(self, end1, end2):
        self.end1 = end1
        self.end2 = end2

    def __eq__(self, other):
        if np.array_equal(self.end1, other.end1) and np.array_equal(self.end2, other.end2):
            return True
        else:
            return False


class WCS:
    def __init__(self, root, all_nodes, init_tree=None):
        self.remained_nodes = all_nodes[:]  # 记录剩余节点 （切片复制新建地址）
        if init_tree is None:
            init_tree = []
        self.tree = [root]
        self.tree.extend(init_tree)
        for node in init_tree:
            self.remained_nodes.remove(node)

    def _add_node(self, added_node: Edge, parent):
        parent_index = self.tree.index(parent)
        self.tree.insert(parent_index + 1, added_node)
        self.remained_nodes.remove(added_node)

    def _remove_node(self, removed_node: Edge):
        self.tree.remove(removed_node)
        self.remained_nodes.append(removed_node)

    def update(self, new_leaf):
        # 尝试更新
        tree_depth = len(self.tree) - 1
        # 计算碰撞点和对应的比例
        if tree_depth == 0:
            separations, ratios = [], []
        elif tree_depth == 1:
            separations, ratios = calculate_separation_1(self.tree[0].end1, new_leaf, self.tree[1])
        elif tree_depth == 2:
            separations, ratios = calculate_separation_2(self.tree[0].end1, new_leaf, self.tree[1], self.tree[2],
                                                         np.array([0, 0, 0]))
            # print(separations, ratios)
        else:
            separations, ratios = calculate_separation_3(self.tree[0].end1, new_leaf, self.tree[1], self.tree[2],
                                                         self.tree[3], np.array([0, 0, 0]))
            # print(separations, ratios)

        is_correct = False
        while not is_correct:  # 消除奇异点（主要为因步长离散而连续过多个顶点的情况）

            # 生成全体路径点序列
            whole_waypoint = separations[:]
            whole_waypoint.insert(0, self.tree[0].end1)
            whole_waypoint.append(new_leaf)

            is_diff = False  # 序列结构改变标识

            # 检测序列结构是否变化（3种情况默认不会同时发生）
            # 1.检测绳索是否从顶点滑出
            for index, ratio in enumerate(ratios):
                if ratio <= 0 or ratio >= 1:  # 比例超出范围则从顶点滑出
                    vertex = self.tree[index + 1].end1 if ratio <= 0 else self.tree[index + 1].end2  # 确定滑出的顶点
                    # 寻找滑出后可能碰撞的棱，即从该顶点出发的其他棱
                    possible_nodes = []
                    for edge in self.remained_nodes:
                        if np.array_equal(edge.end1, vertex) or np.array_equal(edge.end2, vertex):
                            possible_nodes.append(edge)
                    self._remove_node(self.tree[index + 1])
                    print("leave edge({}) from end".format(index))
                    # 判断是否两段同时从同一顶点滑出
                    double_leave = False
                    if index + 1 < len(ratios):
                        if ratios[index + 1] <= 0 or ratios[index + 1] >= 1:
                            double_leave = True
                            self._remove_node(self.tree[index + 1])
                            print("leave edge({}) from end".format(index + 1))
                    # 确定新碰撞的棱
                    new_nodes = check_new_collision(whole_waypoint[index], whole_waypoint[index + 2 + double_leave],
                                                    vertex, possible_nodes, np.array([0, 0, -10000]))
                    # 将新节点接入序列中
                    if new_nodes:
                        for i, nodes in enumerate(new_nodes):
                            if i == 0:
                                self._add_node(nodes, self.tree[index])
                            else:
                                self._add_node(nodes, new_nodes[i - 1])
                        print("add new nodes")

                    is_diff = True
                    break

            # 2.检测自由段是否碰撞新的棱
            if not is_diff:
                ori_len = len(self.tree)
                for edge in self.remained_nodes:
                    dist, t1, t2 = get_distance_between_lines(whole_waypoint[-2], whole_waypoint[-1], edge.end1,
                                                              edge.end2)
                    if 0 < t1 < 1 and 0 < t2 < 1:  # 两线段投影是否相交
                        if dist <= 0.005:  # 距离阈值，距离小于该阈值认为可能将发生碰撞
                            # 检测碰撞
                            if check_collision_infinite(whole_waypoint[-2], whole_waypoint[-1], edge,
                                                        np.array([0, 0, 0])):
                                _, ratio = calculate_separation_1(whole_waypoint[-2], whole_waypoint[-1], edge)
                                if 0 < ratio[0] < 1:
                                    self._add_node(edge, self.tree[-1])
                                    # 排除奇异情况
                                    if len(self.tree) == 3:
                                        _, ratios = calculate_separation_2(whole_waypoint[0], whole_waypoint[-1],
                                                                           self.tree[-2], self.tree[-1],
                                                                           np.array([0, 0, 0]))
                                        if ratios[0] < 0 or ratios[0] > 1 or ratios[1] < 0 or ratios[1] > 1:
                                            self._remove_node(edge)
                                        else:
                                            print("collide on a new edge")
                                    elif len(self.tree) == 4:
                                        _, ratios = calculate_separation_3(whole_waypoint[0], whole_waypoint[-1],
                                                                           self.tree[-3], self.tree[-2], self.tree[-1],
                                                                           np.array([0, 0, 0]))
                                        if (ratios[0] < 0 or ratios[0] > 1 or ratios[1] < 0 or ratios[1] > 1 or
                                                ratios[2] < 0 or ratios[2] > 1):
                                            self._remove_node(edge)
                                        else:
                                            print("collide on a new edge")
                                    else:
                                        print("collide on a new edge")
                if len(self.tree) == ori_len:
                    is_diff = False
                else:
                    is_diff = True

            # 3.检测最末两段是否离开最后一个棱
            if not is_diff:
                if separations:
                    if not check_collision_infinite(whole_waypoint[-3], whole_waypoint[-1], self.tree[-1],
                                                    np.array([0, 0, 0])):
                        self._remove_node(self.tree[-1])
                        is_diff = True
                        print("leave an edge")

            is_correct = True
            if is_diff:
                # 重新更新
                tree_depth = len(self.tree) - 1
                if tree_depth == 0:
                    separations, ratios = [], []
                elif tree_depth == 1:
                    separations, ratios = calculate_separation_1(self.tree[0].end1, new_leaf, self.tree[1])
                elif tree_depth == 2:
                    separations, ratios = calculate_separation_2(self.tree[0].end1, new_leaf, self.tree[1],
                                                                 self.tree[2], np.array([0, 0, 0]))
                    # print(separations, ratios)
                else:
                    separations, ratios = calculate_separation_3(self.tree[0].end1, new_leaf, self.tree[1],
                                                                 self.tree[2], self.tree[3], np.array([0, 0, 0]))
                    # print(separations, ratios)

                for ratio in ratios:  # 检测解的合理性
                    if ratio <= 0 or ratio >= 1:
                        is_correct = False

        return separations


if __name__ == "__main__":

    obs_side = 22.5
    frame_side = 72
    height = 55
    A1 = np.array([frame_side / 2, frame_side / 2, height])
    A2 = np.array([-frame_side / 2, frame_side / 2, height])
    A3 = np.array([-frame_side / 2, -frame_side / 2, height])
    A4 = np.array([frame_side / 2, -frame_side / 2, height])
    Om1 = np.array([obs_side / 2, obs_side / 2, obs_side])
    Om2 = np.array([-obs_side / 2, obs_side / 2, obs_side])
    Om3 = np.array([-obs_side / 2, -obs_side / 2, obs_side])
    Om4 = np.array([obs_side / 2, -obs_side / 2, obs_side])
    Ob1 = np.array([obs_side / 2, obs_side / 2, 0])
    Ob2 = np.array([-obs_side / 2, obs_side / 2, 0])
    Ob3 = np.array([-obs_side / 2, -obs_side / 2, 0])
    Ob4 = np.array([obs_side / 2, -obs_side / 2, 0])
    Ot = np.array([0, 0, obs_side + obs_side / np.sqrt(2)])

    # B = np.array([obstacle_side / 2 + 10, 0, height - 10])

    anchor1 = Edge(A1, A1)
    anchor2 = Edge(A2, A2)
    anchor3 = Edge(A3, A3)
    anchor4 = Edge(A4, A4)
    edge12 = Edge(Om1, Om2)
    edge23 = Edge(Om2, Om3)
    edge34 = Edge(Om3, Om4)
    edge41 = Edge(Om4, Om1)
    edge19 = Edge(Om1, Ot)
    edge29 = Edge(Om2, Ot)
    edge39 = Edge(Om3, Ot)
    edge49 = Edge(Om4, Ot)
    edge15 = Edge(Om1, Ob1)
    edge26 = Edge(Om2, Ob2)
    edge37 = Edge(Om3, Ob3)
    edge48 = Edge(Om4, Ob4)
    all_edges = [edge12, edge23, edge34, edge41, edge15, edge26, edge37, edge48, edge19, edge29, edge39, edge49]

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    x = np.array([Om4[0], Om1[0], Ot[0], Om1[0], Ob1[0], Om1[0], Om2[0]])
    y = np.array([Om4[1], Om1[1], Ot[1], Om1[1], Ob1[1], Om1[1], Om2[1]])
    z = np.array([Om4[2], Om1[2], Ot[2], Om1[2], Ob1[2], Om1[2], Om2[2]])
    ax.plot(x, y, z, label='parametric curve', color='blue')
    x = np.array([Ot[0], Om4[0], Ob4[0], Om4[0], Om3[0], Ot[0], Om3[0], Ob3[0]])
    y = np.array([Ot[1], Om4[1], Ob4[1], Om4[1], Om3[1], Ot[1], Om3[1], Ob3[1]])
    z = np.array([Ot[2], Om4[2], Ob4[2], Om4[2], Om3[2], Ot[2], Om3[2], Ob3[2]])
    ax.plot(x, y, z, label='parametric curve', color='blue')
    # ax.legend()

    wcs11 = WCS(anchor1, all_edges, [])
    wcs2 = WCS(anchor2, all_edges, [])
    wcs3 = WCS(anchor3, all_edges, [edge49])
    wcs4 = WCS(anchor4, all_edges, [])
    wcs1 = copy.deepcopy(wcs11)

    for i in range(30):
        print('------------------------------------')
        print(i)
        y = 15 - 1 * i
        B = np.array([15, y, 18])
        start_time = time.time()
        seps1 = wcs1.update(B)
        print("----------")
        seps2 = wcs2.update(B)
        print("----------")
        seps3 = wcs3.update(B)
        print("----------")
        seps4 = wcs4.update(B)
        update_time = time.time() - start_time
        print("update time: {}".format(update_time))
        # for i in range(len(wcs.tree)):
        #     print(wcs2.tree[i].end1, wcs2.tree[i].end2)

        cable_length2 = calculate_cable_length(A2, seps2, B)
        cable_length3 = calculate_cable_length(A3, seps3, B)
        # print("cable length: {}".format(cable_length2))

        x = [A1[0], B[0]]
        y = [A1[1], B[1]]
        z = [A1[2], B[2]]
        if seps1:
            for each in seps1:
                x.insert(-1, each[0])
                y.insert(-1, each[1])
                z.insert(-1, each[2])
        ax.plot(x, y, z, label='parametric curve', color='cyan')

        x = [A2[0], B[0]]
        y = [A2[1], B[1]]
        z = [A2[2], B[2]]
        if seps2:
            for each in seps2:
                x.insert(-1, each[0])
                y.insert(-1, each[1])
                z.insert(-1, each[2])
        ax.plot(x, y, z, label='parametric curve', color='red')

        x = [A3[0], B[0]]
        y = [A3[1], B[1]]
        z = [A3[2], B[2]]
        if seps3:
            for each in seps3:
                x.insert(-1, each[0])
                y.insert(-1, each[1])
                z.insert(-1, each[2])
        ax.plot(x, y, z, label='parametric curve', color='green')

        x = [A4[0], B[0]]
        y = [A4[1], B[1]]
        z = [A4[2], B[2]]
        if seps4:
            for each in seps4:
                x.insert(-1, each[0])
                y.insert(-1, each[1])
                z.insert(-1, each[2])
        ax.plot(x, y, z, label='parametric curve', color='yellow')

    for i in range(30):
        print('------------------------------------')
        print(i + 30)
        x = 15 - 1 * i
        B = np.array([x, -15, 18])
        start_time = time.time()
        seps1 = wcs1.update(B)
        print("----------")
        seps2 = wcs2.update(B)
        print("----------")
        seps3 = wcs3.update(B)
        print("----------")
        seps4 = wcs4.update(B)
        update_time = time.time() - start_time
        print("update time: {}".format(update_time))
        # for i in range(len(wcs.tree)):
        #     print(wcs2.tree[i].end1, wcs2.tree[i].end2)
        # print("separations2: {}".format(seps2))
        cable_length2 = calculate_cable_length(A2, seps2, B)
        cable_length3 = calculate_cable_length(A3, seps3, B)
        # print("cable length: {}".format(cable_length2))

        x = [A1[0], B[0]]
        y = [A1[1], B[1]]
        z = [A1[2], B[2]]
        if seps1:
            for each in seps1:
                x.insert(-1, each[0])
                y.insert(-1, each[1])
                z.insert(-1, each[2])
        ax.plot(x, y, z, label='parametric curve', color='cyan')

        x = [A2[0], B[0]]
        y = [A2[1], B[1]]
        z = [A2[2], B[2]]
        if seps2:
            for each in seps2:
                x.insert(-1, each[0])
                y.insert(-1, each[1])
                z.insert(-1, each[2])
        ax.plot(x, y, z, label='parametric curve', color='red')

        x = [A3[0], B[0]]
        y = [A3[1], B[1]]
        z = [A3[2], B[2]]
        if seps3:
            for each in seps3:
                x.insert(-1, each[0])
                y.insert(-1, each[1])
                z.insert(-1, each[2])
        ax.plot(x, y, z, label='parametric curve', color='green')

        x = [A4[0], B[0]]
        y = [A4[1], B[1]]
        z = [A4[2], B[2]]
        if seps4:
            for each in seps4:
                x.insert(-1, each[0])
                y.insert(-1, each[1])
                z.insert(-1, each[2])
        ax.plot(x, y, z, label='parametric curve', color='yellow')

    plt.show()
