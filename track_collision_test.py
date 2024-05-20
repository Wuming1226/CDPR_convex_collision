#! /usr/bin/env python3

from operator import contains
import time
from tokenize import endpats
import numpy as np
import math
# import rospy
import matplotlib.pyplot as plt

from cdpr_test import CDPR
# from transform import euler2quaternion
# from Jacobian import getJacobian
# from wcs import Edge, WCS

folder = 'data/17_first/'
file_order = ''
pos_ref_save_path = folder + 'pos_ref' + file_order + '.txt'
pos_save_path = folder + 'pos' + file_order + '.txt'
cable_length_ref_save_path = folder + 'cable_length_ref' + file_order + '.txt'
cable_length_save_path = folder + 'cable_length' + file_order + '.txt'
length_controller_save_path = folder + 'length_controller' + file_order + '.txt'
velocity_controller_task_save_path = folder + 'velocity_controller_task' + file_order + '.txt'
velocity_controller_joint_save_path = folder + 'velocity_controller_joint' + file_order + '.txt'
motor_velo_save_path = folder + 'motor_velo_' + file_order + '.txt'


if __name__ == "__main__":

    cdpr = CDPR()

    T = 0.1     # control period
    
    x_r_list, y_r_list, z_r_list = [], [], []
    x_list, y_list, z_list = [], [], []
    cl1_r_list, cl2_r_list, cl3_r_list, cl4_r_list = [], [], [], []
    cl1_list, cl2_list, cl3_list, cl4_list = [], [], [], []

    target1 = np.array([0.651, 0.617, -0.718])
    safe_point1 = np.array([0.643, 0.536, -0.501])      # 安全位置1
    safe_point2 = np.array([0.204, 0.288, -0.545])      # 安全位置2
    traject_height = -0.075    # 轨迹高度（相对于中棱面）

    c1_show = True
    c2_show = True
    c3_show = True
    c4_show = True

    time.sleep(1)
    # cdpr.init_cable_length(True, True, True, True)

    cnt = 0

    A1 = np.array([0.759, 0.778, -0.229]) - cdpr.pos_off
    A2 = np.array([0.082, 0.786, -0.230]) - cdpr.pos_off
    A3 = np.array([0.080, 0.097, -0.224]) - cdpr.pos_off
    A4 = np.array([0.769, 0.103, -0.226]) - cdpr.pos_off
    Ot = np.array([0.424, 0.438, -0.449]) - cdpr.pos_off
    Om1 = np.array([0.536, 0.550, -0.612]) - cdpr.pos_off
    Om2 = np.array([0.312, 0.550, -0.612]) - cdpr.pos_off
    Om3 = np.array([0.312, 0.325, -0.612]) - cdpr.pos_off
    Om4 = np.array([0.536, 0.325, -0.612]) - cdpr.pos_off
    Ob1 = np.array([0.536, 0.550, -0.808]) - cdpr.pos_off
    Ob2 = np.array([0.312, 0.550, -0.808]) - cdpr.pos_off
    Ob3 = np.array([0.312, 0.325, -0.808]) - cdpr.pos_off
    Ob4 = np.array([0.536, 0.325, -0.808]) - cdpr.pos_off

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    plt.ion()   # 开启交互模式，进行连续绘图

    # ---------------------- main loop ----------------------

    while cnt < 390:

        print('-----------------------------------------------')
        print('                   run: {}'.format(cnt))

        start_time = time.time()

        # reference pose and cable length of moment k

        if cnt < 30:        # 从初始位置到达安全位置1
            start = np.array([cdpr.xOff, cdpr.yOff, -0.370])
            end = safe_point1
            pos_ref = (end - start) / 30 * cnt + start
        elif cnt < 60:      # 安全位置到达目标位置1上方
            start = safe_point1
            pos_ref = (target1 - start + np.array([0, 0, 0.050])) / 30 * (cnt - 30) + start
        elif cnt < 90:      # 在目标位置1上方下降
            z = 0.050 - 0.050 / 30 * (cnt - 60)
            pos_ref = target1 + np.array([0, 0, z])
        elif cnt < 120:     # 在目标位置1定位
            pos_ref = target1
        elif cnt < 150:     # 从目标位置1到达轨迹初始位置
            start = np.array([0.150 + cdpr.xOff, 0.150 + cdpr.yOff, traject_height + cdpr.zOff])
            pos_ref = (start - target1) / 30 * (cnt - 120) + target1
        elif cnt < 210:     # 轨迹第一段
            y = 0.150 - 0.005 * (cnt - 150)
            pos_ref = np.array([0.150 + cdpr.xOff, y + cdpr.yOff, traject_height + cdpr.zOff])
        elif cnt < 270:     # 轨迹第二段
            x = 0.150 - 0.005 * (cnt - 210)
            pos_ref = np.array([x + cdpr.xOff, -0.150 + cdpr.yOff, traject_height + cdpr.zOff])
        elif cnt < 330:     # 在目标点2上方定位
            pos_ref = np.array([-0.150 + cdpr.xOff, -0.150 + cdpr.yOff, traject_height + cdpr.zOff])
        elif cnt < 360:     # 从目标位置2到达安全位置2
            start = np.array([-0.150 + cdpr.xOff, -0.150 + cdpr.yOff, traject_height + cdpr.zOff])
            end = safe_point2
            pos_ref = (end - start) / 30 * (cnt - 330) + start
        else:               # 从安全位置2到达初始位置
            start = safe_point2
            end = np.array([cdpr.xOff, cdpr.yOff, -0.370])
            pos_ref = (end - start) / 30 * (cnt - 360) + start

        pos_ref = pos_ref - cdpr.pos_off
        [seps1, seps2, seps3, seps4], cable_length_ref = cdpr.update_cable_state(pos_ref)
        cable_length_ref = np.array(cable_length_ref)
        print('pos_ref: {}'.format(pos_ref))
        print('cable_length_ref: {}'.format(cable_length_ref))

        cnt += 1

        x_r_list.append(pos_ref[0])
        y_r_list.append(pos_ref[1])
        z_r_list.append(pos_ref[2])

        cl1_r_list.append(cable_length_ref[0])
        cl2_r_list.append(cable_length_ref[1])
        cl3_r_list.append(cable_length_ref[2])
        cl4_r_list.append(cable_length_ref[3])

        end_time = time.time()
        print(end_time - start_time)

        if not (cnt % 1):
            plt.cla()

            x = np.array([Om4[0], Om1[0], Ot[0], Om1[0], Ob1[0], Om1[0], Om2[0]])
            y = np.array([Om4[1], Om1[1], Ot[1], Om1[1], Ob1[1], Om1[1], Om2[1]])
            z = np.array([Om4[2], Om1[2], Ot[2], Om1[2], Ob1[2], Om1[2], Om2[2]])
            ax.plot(x, y, z, label='parametric curve', color='blue')
            x = np.array([Ot[0], Om4[0], Ob4[0], Om4[0], Om3[0], Ot[0], Om3[0], Ob3[0]])
            y = np.array([Ot[1], Om4[1], Ob4[1], Om4[1], Om3[1], Ot[1], Om3[1], Ob3[1]])
            z = np.array([Ot[2], Om4[2], Ob4[2], Om4[2], Om3[2], Ot[2], Om3[2], Ob3[2]])
            ax.plot(x, y, z, label='parametric curve', color='blue')
            x = np.array([Om2[0], Om3[0], Ot[0], Om2[0], Ob2[0]])
            y = np.array([Om2[1], Om3[1], Ot[1], Om2[1], Ob2[1]])
            z = np.array([Om2[2], Om3[2], Ot[2], Om2[2], Ob2[2]])
            ax.plot(x, y, z, label='parametric curve', color='blue')

            B = pos_ref
            x = [A1[0], B[0]]
            y = [A1[1], B[1]]
            z = [A1[2], B[2]]
            if seps1:
                for each in seps1:
                    x.insert(-1, each[0])
                    y.insert(-1, each[1])
                    z.insert(-1, each[2])
            if c1_show:
                ax.plot(x, y, z, label='parametric curve', color='cyan')

            x = [A2[0], B[0]]
            y = [A2[1], B[1]]
            z = [A2[2], B[2]]
            if seps2:
                for each in seps2:
                    x.insert(-1, each[0])
                    y.insert(-1, each[1])
                    z.insert(-1, each[2])
            if c2_show:
                ax.plot(x, y, z, label='parametric curve', color='red')

            x = [A3[0], B[0]]
            y = [A3[1], B[1]]
            z = [A3[2], B[2]]
            if seps3:
                for each in seps3:
                    x.insert(-1, each[0])
                    y.insert(-1, each[1])
                    z.insert(-1, each[2])
            if c3_show:
                ax.plot(x, y, z, label='parametric curve', color='green')

            x = [A4[0], B[0]]
            y = [A4[1], B[1]]
            z = [A4[2], B[2]]
            if seps4:
                for each in seps4:
                    x.insert(-1, each[0])
                    y.insert(-1, each[1])
                    z.insert(-1, each[2])
            if c4_show:
                ax.plot(x, y, z, label='parametric curve', color='yellow')

            ax.set_xlim(-0.4, 0.4)
            ax.set_ylim(-0.4, 0.4)
            ax.set_zlim(-(Om1[2] - Ob1[2]), 0.3)
            plt.gca().set_box_aspect((1, 1, (0.3+Om1[2] - Ob1[2])/0.8))
            plt.pause(0.1)

    plt.ioff()  # 关闭交互模式
    plt.show()

    # plot
    fig = plt.figure(2)
    x_plot = fig.add_subplot(4, 2, 1)
    y_plot = fig.add_subplot(4, 2, 3)
    z_plot = fig.add_subplot(4, 2, 5)
    c1_plot = fig.add_subplot(4, 2, 2)
    c2_plot = fig.add_subplot(4, 2, 4)
    c3_plot = fig.add_subplot(4, 2, 6)
    c4_plot = fig.add_subplot(4, 2, 8)
    plt.ion()

    x_plot.plot(x_r_list)
    x_plot.plot(x_list)
    y_plot.plot(y_r_list)
    y_plot.plot(y_list)
    z_plot.plot(z_r_list)
    z_plot.plot(z_list)

    c1_plot.plot(cl1_r_list)
    c1_plot.plot(cl1_list)
    c2_plot.plot(cl2_r_list)
    c2_plot.plot(cl2_list)
    c3_plot.plot(cl3_r_list)
    c3_plot.plot(cl3_list)
    c4_plot.plot(cl4_r_list)
    c4_plot.plot(cl4_list)

    x_plot.set_ylabel('x')
    y_plot.set_ylabel('y')
    z_plot.set_ylabel('z')

    plt.ioff()
    plt.show()



    
