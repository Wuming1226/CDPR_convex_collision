#! /usr/bin/env python3

import time
import numpy as np
import math
import rospy
import matplotlib.pyplot as plt
import scipy.io as io

from cdpr import CDPR

folder = 'data/17_first/'
file_order = ''
pos_ref_save_path = folder + 'pos_ref' + file_order +'.txt'
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
    rate = rospy.Rate(1/T)
    
    x_r_list, y_r_list, z_r_list = [], [], []
    x_list, y_list, z_list = [], [], []
    cl1_r_list, cl2_r_list, cl3_r_list, cl4_r_list = [], [], [], []
    cl1_list, cl2_list, cl3_list, cl4_list = [], [], [], []
    pos_list = np.empty((0, 3))
    pos_ref_list = np.empty((0, 3))
    cable_length_ref_list = np.empty((0, 4))
    cable_length_list = np.empty((0, 4))
    length_controller_list = np.empty((0, 4))
    velocity_controller_task_list = np.empty((0, 3))
    velocity_controller_joint_list = np.empty((0, 4))
    motor_velo_list = np.empty((0, 4))

    target1 = np.array([0.651, 0.617, -0.718])
    safe_point1 = np.array([0.643, 0.536, -0.501])  # 安全位置1
    safe_point2 = np.array([0.204, 0.288, -0.545])  # 安全位置2
    traject_height = -0.075  # 轨迹高度（相对于中棱面）

    tighten_flag = True

    ########## main loop ##########

    time.sleep(1)
    cdpr.init_cable_length(True, True, True, True)

    cnt = 0

    # ---------------------- main loop ----------------------

    while not rospy.is_shutdown() and cnt < 390:

        print('-----------------------------------------------')
        print('                   run: {}'.format(cnt))

        start_time = time.time()

        # reference pose and cable length of moment k

        if cnt < 30:  # 从初始位置到达安全位置1
            start = np.array([cdpr.xOff, cdpr.yOff, -0.370])
            end = safe_point1
            pos_ref = (end - start) / 30 * cnt + start
        elif cnt < 60:  # 安全位置到达目标位置1上方
            start = safe_point1
            pos_ref = (target1 - start + np.array([0, 0, 0.050])) / 30 * (cnt - 30) + start
        elif cnt < 90:  # 在目标位置1上方下降
            z = 0.050 - 0.050 / 30 * (cnt - 60)
            pos_ref = target1 + np.array([0, 0, z])
        elif cnt < 120:  # 在目标位置1定位
            pos_ref = target1
        elif cnt < 150:  # 从目标位置1到达轨迹初始位置
            start = np.array([0.150 + cdpr.xOff, 0.150 + cdpr.yOff, traject_height + cdpr.zOff])
            pos_ref = (start - target1) / 30 * (cnt - 120) + target1
        elif cnt < 210:  # 轨迹第一段
            y = 0.150 - 0.005 * (cnt - 150)
            pos_ref = np.array([0.150 + cdpr.xOff, y + cdpr.yOff, traject_height + cdpr.zOff])
        elif cnt < 270:  # 轨迹第二段
            x = 0.150 - 0.005 * (cnt - 210)
            pos_ref = np.array([x + cdpr.xOff, -0.150 + cdpr.yOff, traject_height + cdpr.zOff])
        elif cnt < 330:  # 在目标点2上方定位
            pos_ref = np.array([-0.150 + cdpr.xOff, -0.150 + cdpr.yOff, traject_height + cdpr.zOff])
        elif cnt < 360:  # 从目标位置2到达安全位置2
            start = np.array([-0.150 + cdpr.xOff, -0.150 + cdpr.yOff, traject_height + cdpr.zOff])
            end = safe_point2
            pos_ref = (end - start) / 30 * (cnt - 330) + start
        else:  # 从安全位置2到达初始位置
            start = safe_point2
            end = np.array([cdpr.xOff, cdpr.yOff, -0.370])
            pos_ref = (end - start) / 30 * (cnt - 360) + start

        pos_ref = pos_ref - cdpr.pos_off
        [seps1, seps2, seps3, seps4], cable_length_ref = cdpr.update_cable_state()
        cable_length_ref = np.array(cable_length_ref)
        print('pos_ref: {}'.format(pos_ref))
        print('cable_length_ref: {}'.format(cable_length_ref))

        cnt += 1

        if cnt < 30:  # 从初始位置到达安全位置1
            start = np.array([cdpr.xOff, cdpr.yOff, -0.370])
            end = safe_point1
            pos_ref_next = (end - start) / 30 * cnt + start
        elif cnt < 60:  # 安全位置到达目标位置1上方
            start = safe_point1
            pos_ref_next = (target1 - start + np.array([0, 0, 0.050])) / 30 * (cnt - 30) + start
        elif cnt < 90:  # 在目标位置1上方下降
            z = 0.050 - 0.050 / 30 * (cnt - 60)
            pos_ref_next = target1 + np.array([0, 0, z])
        elif cnt < 120:  # 在目标位置1定位
            pos_ref_next = target1
        elif cnt < 150:  # 从目标位置1到达轨迹初始位置
            start = np.array([0.150 + cdpr.xOff, 0.150 + cdpr.yOff, traject_height + cdpr.zOff])
            pos_ref_next = (start - target1) / 30 * (cnt - 120) + target1
        elif cnt < 210:  # 轨迹第一段
            y = 0.150 - 0.005 * (cnt - 150)
            pos_ref_next = np.array([0.150 + cdpr.xOff, y + cdpr.yOff, traject_height + cdpr.zOff])
        elif cnt < 270:  # 轨迹第二段
            x = 0.150 - 0.005 * (cnt - 210)
            pos_ref_next = np.array([x + cdpr.xOff, -0.150 + cdpr.yOff, traject_height + cdpr.zOff])
        elif cnt < 330:  # 在目标点2上方定位
            pos_ref_next = np.array([-0.150 + cdpr.xOff, -0.150 + cdpr.yOff, traject_height + cdpr.zOff])
        elif cnt < 360:  # 从目标位置2到达安全位置2
            start = np.array([-0.150 + cdpr.xOff, -0.150 + cdpr.yOff, traject_height + cdpr.zOff])
            end = safe_point2
            pos_ref_next = (end - start) / 30 * (cnt - 330) + start
        else:  # 从安全位置2到达初始位置
            start = safe_point2
            end = np.array([cdpr.xOff, cdpr.yOff, -0.370])
            pos_ref_next = (end - start) / 30 * (cnt - 360) + start

        pos_ref_next = pos_ref_next - cdpr.pos_off
        [seps1, seps2, seps3, seps4], cable_length_ref_next = cdpr.update_cable_state()
        cable_length_ref_next = np.array(cable_length_ref_next)
        print('pos_ref_next: {}'.format(pos_ref_next))
        print('cable_length_ref_next: {}'.format(cable_length_ref_next))

        # pose and cable length of moment k  (unit: degree)
        x, y, z, orientation = cdpr.get_moving_platform_pose()
        pos = np.array([x, y, z])
        print('pos: {}'.format(pos))
        cable_length = cdpr.get_cable_length()
        print('cable length: {}'.format(cable_length))

        # error of moment k  (unit: degree)
        pos_err = pos_ref - pos
        print('pos err: {}'.format(pos_err))
        cable_length_err = cable_length_ref - cable_length
        print('cable length err: {}'.format(cable_length_err))

        # output velocity of moment k  (unit: degree/s)
        eps = 0.002
        k = 2.5
        velo_tag1 = (cable_length_ref_next - cable_length_ref) / T + eps * np.sign(cable_length_err) + k * cable_length_err           # control law
        print('veloJoint (linear): {}'.format(velo_tag1))

        # controller on position
        eps = 0.000
        k = 0.8
        velo_task = (pos_ref_next - pos_ref) / T + eps * np.sign(pos_err) + k * pos_err
        pos_tag = pos + velo_task * T
        cable_length_tag = cdpr.update_cable_state(fake_new_leaf=pos_tag)
        eps = 0.002
        k = 2.5
        velo_tag2 = (cable_length_ref_next - cable_length_ref) / T + eps * np.sign(
            cable_length_err) + k * cable_length_err  # control law
        print('veloJoint (linear): {}'.format(velo_tag2))


        # inverse kinematics
        veloJoint2 = np.matmul(J, veloTask.reshape(3, 1))
        veloJoint2 = veloJoint2.reshape(4, )
        print('veloJoint_add: {}'.format(veloJoint2))
        if 30 < cnt < 60 or 210 < cnt < 270:
            veloJoint = veloJoint2
        else:
            veloJoint = veloJoint1

        # convert linear velocities to velocities of motors
        veloJoint = veloJoint*60*10/(0.03*math.pi)      # 10 is the gear ratio, 0.03 is diameter of the coil
        print('veloJoint (motor): {}'.format(veloJoint))

        # set cable velocity in joint space
        velo_limit = 600
        for i in range(4):
            if np.abs(veloJoint[i]) > velo_limit:      # velovity limit
                veloJoint[i] = velo_limit * np.sign(veloJoint[i])
        set_start_time = time.time()
        cdpr.set_motor_velo(int(veloJoint[0]), int(veloJoint[1]), int(veloJoint[2]), int(veloJoint[3]))
        print('motor velo: {}, {}, {}, {}'.format(veloJoint[0], veloJoint[1], veloJoint[2], veloJoint[3]))
        print(time.time() - set_start_time)
        
        
        x_r_list.append(pos_ref[0])
        y_r_list.append(pos_ref[1])
        z_r_list.append(pos_ref[2])
                    
        x_list.append(pos[0])
        y_list.append(pos[1])
        z_list.append(pos[2])

        cl1_r_list.append(cable1_length_ref)
        cl2_r_list.append(cable2_length_ref)
        cl3_r_list.append(cable3_length_ref)
        cl4_r_list.append(cable4_length_ref)

        cl1_list.append(cable_length[0])
        cl2_list.append(cable_length[1])
        cl3_list.append(cable_length[2])
        cl4_list.append(cable_length[3])

        # data 
        pos_list = np.row_stack((pos_list, pos))
        pos_ref_list = np.row_stack((pos_ref_list, pos_ref))
        cable_length_ref_list = np.row_stack((cable_length_ref_list, cable_length_ref))
        cable_length_list = np.row_stack((cable_length_list, cable_length_next))
        length_controller_list = np.row_stack((length_controller_list, veloJoint1))
        velocity_controller_task_list = np.row_stack((velocity_controller_task_list, veloTask))
        velocity_controller_joint_list = np.row_stack((velocity_controller_joint_list, veloJoint2))
        motor_velo_list = np.row_stack((motor_velo_list, veloJoint))

        end_time = time.time()
        print(end_time - start_time)

        rate.sleep()

    cdpr.set_motor_velo(0, 0, 0, 0)


    # save trajectory datas
    np.savetxt(pos_ref_save_path, pos_ref_list)
    np.savetxt(pos_save_path, pos_list)
    np.savetxt(cable_length_ref_save_path, cable_length_ref_list)
    np.savetxt(cable_length_save_path, cable_length_list)
    np.savetxt(length_controller_save_path, length_controller_list)
    np.savetxt(velocity_controller_task_save_path, velocity_controller_task_list)
    np.savetxt(velocity_controller_joint_save_path, velocity_controller_joint_list)
    np.savetxt(motor_velo_save_path, motor_velo_list)
    print('data saved.')
    
    # plot
    fig = plt.figure(1)
    x_plot = fig.add_subplot(4,2,1)
    y_plot = fig.add_subplot(4,2,3)
    z_plot = fig.add_subplot(4,2,5)
    c1_plot = fig.add_subplot(4,2,2)
    c2_plot = fig.add_subplot(4,2,4)
    c3_plot = fig.add_subplot(4,2,6)
    c4_plot = fig.add_subplot(4,2,8)
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
    
