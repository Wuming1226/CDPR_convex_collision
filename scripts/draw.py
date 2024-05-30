#! /usr/bin/env python3

import time
import numpy as np
import math
# import rospy
import matplotlib.pyplot as plt

folder = '../data/beta/'
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

    fig = plt.figure(1)
    plt.ion()

    time.sleep(3)

    while True:
        pos_r_list = np.loadtxt(pos_ref_save_path)
        pos_list = np.loadtxt(pos_save_path)
        x_r_list, y_r_list, z_r_list = [], [], []
        x_list, y_list, z_list = [], [], []
        for each in pos_r_list:
            x_r_list.append(each[0])
            y_r_list.append(each[1])
            z_r_list.append(each[2])
        for each in pos_list:
            x_list.append(each[0])
            y_list.append(each[1])
            z_list.append(each[2])

        cable_length_r_list = np.loadtxt(cable_length_ref_save_path)
        cable_length_list = np.loadtxt(cable_length_save_path)
        cl1_r_list, cl2_r_list, cl3_r_list, cl4_r_list = [], [], [], []
        cl1_list, cl2_list, cl3_list, cl4_list = [], [], [], []
        for each in cable_length_r_list:
            cl1_r_list.append(each[0])
            cl2_r_list.append(each[1])
            cl3_r_list.append(each[2])
            cl4_r_list.append(each[3])
        for each in cable_length_list:
            cl1_list.append(each[0])
            cl2_list.append(each[1])
            cl3_list.append(each[2])
            cl4_list.append(each[3])

        # plot
        fig = plt.figure(1)
        x_plot = fig.add_subplot(4, 2, 1)
        y_plot = fig.add_subplot(4, 2, 3)
        z_plot = fig.add_subplot(4, 2, 5)
        c1_plot = fig.add_subplot(4, 2, 2)
        c2_plot = fig.add_subplot(4, 2, 4)
        c3_plot = fig.add_subplot(4, 2, 6)
        c4_plot = fig.add_subplot(4, 2, 8)

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

        plt.pause(1)
        plt.clf()

