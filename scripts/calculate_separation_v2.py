import numpy as np


def rotate_point(point, axis, axis_point, theta):
    """
    :param point: point to be rotated
    :param axis: direction vector of rotation axis
    :param axis_point: a point on the rotation axis
    :param theta: rotation angle around rotation axis
    :return: rotated point
    """
    vector = point - axis_point  # vector from one end of the axis to the point
    axis = axis / np.linalg.norm(axis)  # unit orientation vector of the axis
    return axis_point + vector * np.cos(theta) + np.cross(axis, vector) * np.sin(theta) + axis * np.dot(axis,
                                                                                                        vector) * (
                1 - np.cos(theta))  # rotate the vector


def get_united_normal_vector(*points):
    """
    :param points: 3 points on the plane in counter-clockwise order
    :return: unit normal vector of the plane which points outside the surface
    """
    vector1 = points[1] - points[0]
    vector2 = points[2] - points[1]
    normal_vector = np.cross(vector1, vector2)
    return normal_vector / np.linalg.norm(normal_vector)


def get_distance_between_lines(point1_line1, point2_line1, point1_line2, point2_line2):
    # 计算直线1的方向向量
    direction1 = np.array(point2_line1) - np.array(point1_line1)
    # 计算直线2的方向向量
    direction2 = np.array(point2_line2) - np.array(point1_line2)

    # 计算两直线之间的最短距离
    a = np.dot(direction1, direction1)
    b = np.dot(direction1, direction2)
    c = np.dot(direction2, direction2)
    d = np.dot(direction1, np.array(point1_line1) - np.array(point1_line2))
    e = np.dot(direction2, np.array(point1_line1) - np.array(point1_line2))

    # 计算参数t1和t2
    t1 = (b * e - c * d) / (a * c - b ** 2)
    t2 = (a * e - b * d) / (a * c - b ** 2)

    # 计算两个最近点
    closest_point1 = np.array(point1_line1) + t1 * direction1
    closest_point2 = np.array(point1_line2) + t2 * direction2

    # 计算最短距离
    distance = np.linalg.norm(closest_point1 - closest_point2)

    return distance, t1, t2


def get_intersection(Apoint, Bpoint, Cpoint1, Cpoint2):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param Cpoint1: left end of the bar
    :param Cpoint2: right end of the bar
    :return: intersection point and ratio, else false
            (ratio < 0 or > 1 also means no intersection)
    case:
                        A
                        |
                        |
            C1  ————————|——————————  C2
                        |
                        |
                        B

    """
    vector1 = Cpoint2 - Cpoint1  # orientation vector of the line1 (bar)
    vector2 = Bpoint - Apoint  # orientation vector of the line2 (cable)
    vector3 = Apoint - Cpoint1

    vecS1 = np.cross(vector1, vector2)  # signed area
    vecS2 = np.cross(vector3, vector2)  # signed area

    # check whether two lines are coplanar
    if np.dot(vector3, vecS1) >= 0.001 or np.dot(vector3, vecS1) <= -0.001:
        return False

    # distance from one end of the segment to the intersection / length of the segment
    ratio = np.dot(vecS1, vecS2) / np.dot(vecS1, vecS1)

    # if ratio < 0 or ratio > 1:
    #     return False
    # else:
    #     return Cpoint1 + vector1 * ratio
    # check whether the intersection is on the segment
    return Cpoint1 + vector1 * ratio, ratio  # position of the intersection


def check_collision_infinite(Apoint, Bpoint, edge, inner_point):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param edge: edge
    :param inner_point: a point inside the polyhedron
    :return: whether collision occurs on the bar

    case:
                        A
                        |
                        |
            C1  ————————|——————————  C2
                        |
                        |
                        B

    """

    Cpoint1 = edge.end1
    Cpoint2 = edge.end2
    norm_vector1 = get_united_normal_vector(Apoint, Cpoint1, Cpoint2)
    norm_vector2 = get_united_normal_vector(Bpoint, Cpoint2, Cpoint1)
    axis = (Cpoint2 - Cpoint1) / np.linalg.norm(Cpoint2 - Cpoint1)
    if np.dot(Cpoint1 - inner_point, norm_vector1) < 0:     # 第一个面总是在多面体外，且要求 norm1 总指向体外
        norm_vector1 = get_united_normal_vector(Apoint, Cpoint2, Cpoint1)
        norm_vector2 = get_united_normal_vector(Bpoint, Cpoint1, Cpoint2)
        axis = (Cpoint1 - Cpoint2) / np.linalg.norm(Cpoint1 - Cpoint2)

    # collision occurs when the angle between norm vectors of two planes is bigger than 90°
    if np.dot(np.cross(norm_vector1, norm_vector2), axis) > 0:  # 可能的 warning 来自于高版本 np.cross 的 bug，不影响运行
        return True
    else:
        return False


def calculate_separation_1(Apoint, Bpoint, edge):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param edge:
    :return separation point
    :return ratio: the separation ratio from end1

    case:
                        A
                        |
                        |
            C1  ————————|——————————  C2
                        |
                        |
                        B

    """

    Cpoint1 = edge.end1
    Cpoint2 = edge.end2

    norm1 = get_united_normal_vector(Apoint, Cpoint1, Cpoint2)
    norm2 = get_united_normal_vector(Bpoint, Cpoint2, Cpoint1)  # 三点的顺序保证了两个法向量指向同一侧
    axis = np.cross(norm1, norm2)
    theta = np.arccos(np.dot(norm1, norm2))
    Apoint_r = rotate_point(Apoint, axis, Cpoint1, theta)
    separation, ratio = get_intersection(Apoint_r, Bpoint, Cpoint1, Cpoint2)

    return [separation], [ratio]


def calculate_separation_2(Apoint, Bpoint, edge1, edge2, inner_point):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param edge1: the first cylinder (必须确定一定发生了碰撞！)
    :param edge2: the second cylinder (必须确定一定发生了碰撞！)
    :return separation: the separation points
    :return ratio: the separation ratio from end1
    """

    C1point1 = edge1.end1
    C1point2 = edge1.end2
    C2point1 = edge2.end1
    C2point2 = edge2.end2

    norm1 = get_united_normal_vector(Apoint, C1point1, C1point2)

    if np.array_equal(C2point1, C1point1) or np.array_equal(C2point1, C1point2):  # 若共顶点，则用另一顶点计算
        norm2 = get_united_normal_vector(C2point2, C1point2, C1point1)
    else:
        norm2 = get_united_normal_vector(C2point1, C1point2, C1point1)
    axis1 = np.cross(norm1, norm2)

    norm3 = get_united_normal_vector(Bpoint, C2point1, C2point2)
    if np.dot(norm1, C1point1 - inner_point) * np.dot(norm3, C2point1 - inner_point) < 0:  # 通过内点保证所有法向量指向同侧
        norm3 = get_united_normal_vector(Bpoint, C2point2, C2point1)
    axis2 = np.cross(norm2, norm3)

    theta1 = np.arccos(np.dot(norm1, norm2))
    theta2 = np.arccos(np.dot(norm2, norm3))
    Apoint_r = rotate_point(Apoint, axis1, C1point1, theta1)
    Bpoint_r = rotate_point(Bpoint, axis2, C2point1, -theta2)

    intersection1, ratio1 = get_intersection(Apoint_r, Bpoint_r, C1point1, C1point2)
    intersection2, ratio2 = get_intersection(Apoint_r, Bpoint_r, C2point1, C2point2)

    separation = [intersection1, intersection2]
    ratio = [ratio1, ratio2]

    return separation, ratio


def calculate_separation_3(Apoint, Bpoint, edge1, edge2, edge3, inner_point):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param edge1: cylinder1 (必须确定一定发生了碰撞！)
    :param edge2: cylinder2 (必须确定一定发生了碰撞！)
    :param edge3: cylinder3 (必须确定一定发生了碰撞！)
    :return separation: the separation points
    :return ratio: the separation ratio from end1
    """

    C1point1 = edge1.end1
    C1point2 = edge1.end2
    C2point1 = edge2.end1
    C2point2 = edge2.end2
    C3point1 = edge3.end1
    C3point2 = edge3.end2

    norm1 = get_united_normal_vector(Apoint, C1point1, C1point2)

    if np.array_equal(C2point1, C1point1) or np.array_equal(C2point1, C1point2):
        norm2 = get_united_normal_vector(C2point2, C1point2, C1point1)
    else:
        norm2 = get_united_normal_vector(C2point1, C1point2, C1point1)
    axis1 = np.cross(norm1, norm2)

    if np.array_equal(C3point1, C2point1) or np.array_equal(C3point1, C2point2):
        norm3 = get_united_normal_vector(C3point2, C2point1, C2point2)
        if np.dot(norm1, C1point1 - inner_point) * np.dot(norm3, C2point1 - inner_point) < 0:  # 通过内点保证所有法向量指向同侧
            norm3 = get_united_normal_vector(C3point2, C2point2, C2point1)
        axis2 = np.cross(norm2, norm3)

    else:
        norm3 = get_united_normal_vector(C3point1, C2point1, C2point2)
        if np.dot(norm1, C1point1 - inner_point) * np.dot(norm3, C2point1 - inner_point) < 0:  # 通过内点保证所有法向量指向同侧
            norm3 = get_united_normal_vector(C3point1, C2point2, C2point1)
        axis2 = np.cross(norm2, norm3)

    norm4 = get_united_normal_vector(Bpoint, C3point1, C3point2)
    if np.dot(norm1, C1point1 - inner_point) * np.dot(norm4, C3point1 - inner_point) < 0:  # 通过内点保证所有法向量指向同侧
        norm4 = get_united_normal_vector(Bpoint, C3point2, C3point1)
    axis3 = np.cross(norm3, norm4)

    theta1 = np.arccos(np.dot(norm1, norm2))
    theta2 = np.arccos(np.dot(norm2, norm3))
    theta3 = np.arccos(np.dot(norm3, norm4))
    Apoint_r1 = rotate_point(Apoint, axis1, C1point1, theta1)
    Apoint_r12 = rotate_point(Apoint_r1, axis2, C2point1, theta2)
    C1point1_r2 = rotate_point(C1point1, axis2, C2point1, theta2)
    C1point2_r2 = rotate_point(C1point2, axis2, C2point1, theta2)
    Bpoint_r3 = rotate_point(Bpoint, axis3, C3point1, -theta3)
    intersection1_r2, ratio1 = get_intersection(Apoint_r12, Bpoint_r3, C1point1_r2, C1point2_r2)
    intersection2, ratio2 = get_intersection(Apoint_r12, Bpoint_r3, C2point1, C2point2)
    intersection3, ratio3 = get_intersection(Apoint_r12, Bpoint_r3, C3point1, C3point2)

    intersection1 = rotate_point(intersection1_r2, axis2, C2point1, -theta2)

    separation = [intersection1, intersection2, intersection3]
    ratio = [ratio1, ratio2, ratio3]

    return separation, ratio


def check_new_collision(Apoint, Bpoint, vertex, edges, inner_point):
    """
    :param Apoint: 固定端（滑出部分的上一路径点）
    :param Bpoint: 自由端（滑出部分的上一路径点）
    :param vertex: 公共顶点（滑出的顶点）
    :param edges: 可能碰撞的棱的数组
    :param inner_point: 凸多面体内部的点
    :return: 发生碰撞的棱的数组（无为 None）
    """
    base_norm = get_united_normal_vector(Apoint, vertex, Bpoint)  # A、B和公共顶点所在平面（基准平面）的法向量
    if np.dot(base_norm, vertex - inner_point) < 0:  # 确保法向量指向外侧
        base_norm = get_united_normal_vector(Apoint, Bpoint, vertex)
    A_angles = []  # 所有棱的非公共顶点、A和公共顶点所在的平面，与基准平面的夹角（A夹角）
    B_angles = []  # 所有棱的非公共顶点、B和公共顶点所在的平面，与基准平面的夹角（B夹角）
    ori_edges = edges[:]  # 切片复制用于遍历
    for index, edge in enumerate(ori_edges):
        private_end = edge.end2 if np.array_equal(vertex, edge.end1) else edge.end1  # 确定非公共顶点

        if np.dot(private_end - vertex, base_norm) < 0:  # 若该棱在基准平面以下，则不参与检测
            edges.remove(edge)
        else:
            # 计算夹角
            A_norm = get_united_normal_vector(Apoint, vertex, private_end)
            if np.dot(A_norm, vertex - inner_point) < 0:
                A_norm = get_united_normal_vector(Apoint, private_end, vertex)
            A_angles.append(np.arccos(np.dot(A_norm, base_norm)))

            B_norm = get_united_normal_vector(Bpoint, vertex, private_end)
            if np.dot(B_norm, vertex - inner_point) < 0:
                B_norm = get_united_normal_vector(Bpoint, private_end, vertex)
            B_angles.append(np.arccos(np.dot(B_norm, base_norm)))

    if not A_angles:
        return None
    else:
        A_highest_index = np.argmax(A_angles)  # A夹角最大的棱的序号，固定端出发的线段最优先碰撞这一条棱
        B_highest_index = np.argmax(B_angles)  # B夹角最大的棱的序号，自由端出发的线段最优先碰撞这一条棱
        if A_highest_index == B_highest_index:  # 若A、B夹角最大的是同一棱，则仅可能与该棱碰撞
            # 碰撞检测
            _, ratio = calculate_separation_1(Apoint, Bpoint, edges[A_highest_index])
            if ratio[0] <= 0 or ratio[0] >= 1:
                return None
            else:
                return [edges[A_highest_index]]
        else:  # 若夹角最大的不是同一条棱
            if len(edges) == 2:  # 若在基准面以上的棱只有2条
                # 对2条棱的碰撞检测
                _, ratios = calculate_separation_2(Apoint, Bpoint, edges[A_highest_index], edges[B_highest_index],
                                                   inner_point)
                if 0 < ratios[0] < 1 and 0 < ratios[1] < 1:
                    return [edges[A_highest_index], edges[B_highest_index]]
                else:  # 若无解，则最多可能碰撞其中一条
                    _, ratio1 = calculate_separation_1(Apoint, Bpoint, edges[A_highest_index])
                    _, ratio2 = calculate_separation_1(Apoint, Bpoint, edges[B_highest_index])
                    if 0 < ratio1[0] < 1:
                        return [edges[A_highest_index]]
                    elif 0 < ratio2[0] < 1:
                        return [edges[B_highest_index]]
                    else:
                        return None
            else:
                for i in range(3):  # 若在基准面上的棱有3条（3条为研究情况的上限）
                    if i != A_highest_index and i != B_highest_index:  # 寻找中间棱
                        middle_edge = edges[i]
                # 对3条棱的碰撞检测
                _, ratios = calculate_separation_3(Apoint, Bpoint, edges[A_highest_index], middle_edge,
                                                   edges[B_highest_index], inner_point)
                if 0 < ratios[0] < 1 and 0 < ratios[1] < 1 and 0 < ratios[2] < 1:
                    return [edges[A_highest_index], middle_edge, edges[B_highest_index]]
                else:
                    _, ratios = calculate_separation_2(Apoint, Bpoint, edges[A_highest_index], edges[B_highest_index],
                                                       inner_point)
                    if 0 < ratios[0] < 1 and 0 < ratios[1] < 1:
                        return [edges[A_highest_index], edges[B_highest_index]]
                    else:
                        _, ratio1 = calculate_separation_1(Apoint, Bpoint, edges[A_highest_index])
                        _, ratio2 = calculate_separation_1(Apoint, Bpoint, edges[B_highest_index])
                        if 0 < ratio1[0] < 1:
                            return [edges[A_highest_index]]
                        elif 0 < ratio2[0] < 1:
                            return [edges[B_highest_index]]
                        else:
                            return None
                    # 这应该还有两种情况，A_highest+中间和B_highest+中间，与第一种情况并列，但其实都没遇到，先不写了


def calculate_cable_length(Apoint, separations, Bpoint):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param separations: collision points
    :return: cable length
    """
    length = 0
    if separations:
        separations = np.array(separations).reshape(-1, 3)
        ends = np.vstack((Apoint, separations, Bpoint))
    else:
        ends = np.vstack((Apoint, Bpoint))

    # print(ends)

    for i in range(ends.shape[0] - 1):
        length += np.linalg.norm(ends[i + 1] - ends[i])

    return length


if __name__ == "__main__":
    pass
