#! /usr/bin/env python3

import rospy
import time
import numpy as np
import copy

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int16MultiArray, Int64MultiArray

from wcs import Edge, WCS
from calculate_separation_v2 import calculate_cable_length


class CDPR:

    def __init__(self):
        # ros settings
        rospy.init_node('cdpr_control', anonymous=True)

        # subscriber and publisher
        self._moving_platform_pose = PoseStamped()
        rospy.Subscriber('/vrpn_client_node/end_effector/pose', PoseStamped, self._pose_callback, queue_size=1)

        self._motor_pos = np.array([0, 0, 0, 0])
        rospy.Subscriber('motor_pos', Int64MultiArray, self._motor_pos_callback, queue_size=1)

        self._veloPub = rospy.Publisher('motor_velo', Int16MultiArray, queue_size=10)

        # 测量数据
        self.middle_level = 0.172
        Om1 = np.array([0.362, 0.264, self.middle_level])
        Om2 = np.array([0.136, 0.264, self.middle_level])
        Om3 = np.array([0.136, 0.039, self.middle_level])
        Om4 = np.array([0.362, 0.039, self.middle_level])
        Ob1 = np.array([0.362, 0.264, 0.000])
        Ob2 = np.array([0.136, 0.264, 0.000])
        Ob3 = np.array([0.136, 0.039, 0.000])
        Ob4 = np.array([0.362, 0.039, 0.000])
        center_x = (Ob1[0] + Ob2[0] + Ob3[0] + Ob4[0]) / 4
        center_y = (Ob1[1] + Ob2[1] + Ob3[1] + Ob4[1]) / 4
        Ot = np.array([0.247, 0.157, 0.337])

        # origin point offset (coordinates in world frame)
        self.xOff = center_x
        self.yOff = center_y
        self.zOff = 0.0
        self.pos_off = np.array([self.xOff, self.yOff, self.zOff])

        # anchor positions in the world frame
        self._anchorA1 = np.array([0.342, 0.342, 0.727])
        self._anchorA2 = np.array([-0.342, 0.342, 0.727])
        self._anchorA3 = np.array([-0.342, -0.342, 0.727])
        self._anchorA4 = np.array([0.342, -0.342, 0.727])

        anchor1 = Edge(self._anchorA1, self._anchorA1)
        anchor2 = Edge(self._anchorA2, self._anchorA2)
        anchor3 = Edge(self._anchorA3, self._anchorA3)
        anchor4 = Edge(self._anchorA4, self._anchorA4)
        edge12 = Edge(Om1 - self.pos_off, Om2 - self.pos_off)
        edge23 = Edge(Om2 - self.pos_off, Om3 - self.pos_off)
        edge34 = Edge(Om3 - self.pos_off, Om4 - self.pos_off)
        edge41 = Edge(Om4 - self.pos_off, Om1 - self.pos_off)
        edge19 = Edge(Om1 - self.pos_off, Ot - self.pos_off)
        edge29 = Edge(Om2 - self.pos_off, Ot - self.pos_off)
        edge39 = Edge(Om3 - self.pos_off, Ot - self.pos_off)
        edge49 = Edge(Om4 - self.pos_off, Ot - self.pos_off)
        edge15 = Edge(Om1 - self.pos_off, Ob1 - self.pos_off)
        edge26 = Edge(Om2 - self.pos_off, Ob2 - self.pos_off)
        edge37 = Edge(Om3 - self.pos_off, Ob3 - self.pos_off)
        edge48 = Edge(Om4 - self.pos_off, Ob4 - self.pos_off)
        all_edges = [edge12, edge23, edge34, edge41, edge15, edge26, edge37, edge48, edge19, edge29, edge39, edge49]

        self.wcs1 = WCS(anchor1, all_edges, [])
        self.wcs2 = WCS(anchor2, all_edges, [])
        self.wcs3 = WCS(anchor3, all_edges, [])
        self.wcs4 = WCS(anchor4, all_edges, [])

        # initial cable lengths and motor positions
        self._ori_cable_lengths = np.array([0., 0., 0., 0.])     # 初始化的类型必须是浮点！！！
        self._ori_motor_pos = np.array([0, 0, 0, 0])

    def init_cable_length(self, cable1_flag, cable2_flag, cable3_flag, cable4_flag):
        # calculate origin cable lengths
        time.sleep(1)
        x0, y0, z0, _ = self.get_moving_platform_pose()
        pos0 = np.array([x0, y0, z0])

        while True:
            if cable1_flag:
                self._ori_cable_lengths[0] = np.linalg.norm(pos0 - self._anchorA1)
                self._ori_motor_pos[0] = self._motor_pos[0]
            if cable2_flag:
                self._ori_cable_lengths[1] = np.linalg.norm(pos0 - self._anchorA2)
                self._ori_motor_pos[1] = self._motor_pos[1]
            if cable3_flag:
                self._ori_cable_lengths[2] = np.linalg.norm(pos0 - self._anchorA3)
                self._ori_motor_pos[2] = self._motor_pos[2]
            if cable4_flag:
                self._ori_cable_lengths[3] = np.linalg.norm(pos0 - self._anchorA4)
                self._ori_motor_pos[3] = self._motor_pos[3]

            if (self._ori_cable_lengths < 1).all():
                break
            else:
                time.sleep(0.5)


    def _pose_callback(self, data):
        # if motion data is lost(999999), do not update
        if (np.abs(data.pose.position.x) > 2000 or np.abs(data.pose.position.y) > 2000
                or np.abs(data.pose.position.z) > 2000):
            pass
        else:
            # pose
            self._moving_platform_pose.pose.position.x = data.pose.position.x / 1000 - self.pos_off[0]
            self._moving_platform_pose.pose.position.y = data.pose.position.y / 1000 - self.pos_off[1]
            self._moving_platform_pose.pose.position.z = data.pose.position.z / 1000 - self.pos_off[2]
            self._moving_platform_pose.pose.orientation = data.pose.orientation

            # header
            self._moving_platform_pose.header.frame_id = data.header.frame_id
            self._moving_platform_pose.header.stamp = data.header.stamp

    def _motor_pos_callback(self, data):
        diff = np.array(data.data) - self._motor_pos
        if (np.abs(diff[0]) > 100000 or np.abs(diff[1]) > 1000000 or np.abs(diff[2]) > 1000000
            or np.abs(diff[3]) > 1000000) \
            and (self._motor_pos[0] != 0 and self._motor_pos[1] != 0 and self._motor_pos[2] != 0
                 and self._motor_pos[3] != 0):
            pass
        else:
            self._motor_pos = np.array(data.data)

    def update_cable_state(self, new_leaf=None, is_fake=False):
        if new_leaf is None:
            new_leaf = np.array([self._moving_platform_pose.pose.position.x, self._moving_platform_pose.pose.position.y,
                                 self._moving_platform_pose.pose.position.z])
            waypoints1 = self.wcs1.update(new_leaf)
            waypoints2 = self.wcs2.update(new_leaf)
            waypoints3 = self.wcs3.update(new_leaf)
            waypoints4 = self.wcs4.update(new_leaf)
        else:
            if not is_fake:
                waypoints1 = self.wcs1.update(new_leaf)
                waypoints2 = self.wcs2.update(new_leaf)
                waypoints3 = self.wcs3.update(new_leaf)
                waypoints4 = self.wcs4.update(new_leaf)
            else:
                fake_wcs1 = copy.deepcopy(self.wcs1)
                fake_wcs2 = copy.deepcopy(self.wcs2)
                fake_wcs3 = copy.deepcopy(self.wcs3)
                fake_wcs4 = copy.deepcopy(self.wcs4)
                waypoints1 = fake_wcs1.update(new_leaf)
                waypoints2 = fake_wcs2.update(new_leaf)
                waypoints3 = fake_wcs3.update(new_leaf)
                waypoints4 = fake_wcs4.update(new_leaf)
                del fake_wcs1, fake_wcs2, fake_wcs3, fake_wcs4

        return [calculate_cable_length(self._anchorA1, waypoints1, new_leaf),
                calculate_cable_length(self._anchorA2, waypoints2, new_leaf),
                calculate_cable_length(self._anchorA3, waypoints3, new_leaf),
                calculate_cable_length(self._anchorA4, waypoints4, new_leaf)]

    def set_motor_velo(self, motor1_velo, motor2_velo, motor3_velo, motor4_velo):
        motor_velo = Int16MultiArray(data=np.array([motor1_velo, motor2_velo, motor3_velo, motor4_velo]))
        self._veloPub.publish(motor_velo)

    def get_moving_platform_pose(self):
        return (self._moving_platform_pose.pose.position.x, self._moving_platform_pose.pose.position.y,
                self._moving_platform_pose.pose.position.z,
                [self._moving_platform_pose.pose.orientation.x, self._moving_platform_pose.pose.orientation.y,
                 self._moving_platform_pose.pose.orientation.z, self._moving_platform_pose.pose.orientation.w])

    def get_cable_length(self):
        r1 = 0.0325 * np.pi / 10000 / 10
        r2 = 0.0324 * np.pi / 10000 / 10
        r3 = 0.0327 * np.pi / 10000 / 10
        r4 = 0.0327 * np.pi / 10000 / 10
        cable_length = np.array([0.0, 0.0, 0.0, 0.0])
        cable_length[0] = (self._motor_pos[0] - self._ori_motor_pos[0]) * r1 + self._ori_cable_lengths[0]
        cable_length[1] = (self._motor_pos[1] - self._ori_motor_pos[1]) * r2 + self._ori_cable_lengths[1]
        cable_length[2] = (self._motor_pos[2] - self._ori_motor_pos[2]) * r3 + self._ori_cable_lengths[2]
        cable_length[3] = (self._motor_pos[3] - self._ori_motor_pos[3]) * r4 + self._ori_cable_lengths[3]
        return cable_length
    
    def pretighten(self, cable1_flag, cable2_flag, cable3_flag, cable4_flag):

        if cable1_flag:
            time.sleep(0.5)
            # cable1 pre-tightening
            print('cable1 pretightening...')
            self.set_motor_velo(-50, 0, 0, 0)
            x0, y0, z0, _ = self.get_moving_platform_pose()
            while True:
                x, y, z, _ = self.get_moving_platform_pose()
                if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.005:
                    self.set_motor_velo(0, 0, 0, 0)
                    break
                else:
                    time.sleep(0.1)

        if cable2_flag:
            time.sleep(0.5)
            # cable2 pre-tightening
            print('cable2 pretightening...')
            self.set_motor_velo(0, -50, 0, 0)
            x0, y0, z0, _ = self.get_moving_platform_pose()
            while True:
                x, y, z, _ = self.get_moving_platform_pose()
                if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.005:
                    self.set_motor_velo(0, 0, 0, 0)
                    break
                else:
                    time.sleep(0.1)

        if cable3_flag:
            time.sleep(0.5)
            # cable3 pre-tightening
            print('cable3 pretightening...')
            self.set_motor_velo(0, 0, -50, 0)
            x0, y0, z0, _ = self.get_moving_platform_pose()
            while True:
                x, y, z, _ = self.get_moving_platform_pose()
                if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.005:
                    self.set_motor_velo(0, 0, 0, 0)
                    break
                else:
                    time.sleep(0.1)

        if cable4_flag:
            time.sleep(0.5)
            # cable4 pre-tightening
            print('cable4 pretightening...')
            self.set_motor_velo(0, 0, 0, -50)
            x0, y0, z0, _ = self.get_moving_platform_pose()
            while True:
                x, y, z, _ = self.get_moving_platform_pose()
                if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.005:
                    self.set_motor_velo(0, 0, 0, 0)
                    break
                else:
                    time.sleep(0.1)

    def loosen(self):
        print('loosening...')
        self.set_motor_velo(600, 600, 600, 600)
        time.sleep(0.2)
        self.set_motor_velo(0, 0, 0, 0)
        time.sleep(0.5)


if __name__ == "__main__":

    cdpr = CDPR()
    rate = rospy.Rate(10)
    # cdpr.pretighten()
    time.sleep(3)
    # cdpr.pretighten(True, True, True, True)
    cdpr.init_cable_length(True, True, True, True)
    time.sleep(1)
    cdpr.set_motor_velo(0, 0, 0, 0)
    start_time = time.time()
    while time.time() - start_time < 5:
        # cdpr.set_motor_velo(0, 100, 0, 0)
        # print(cdpr.get_moving_platform_pose())
        print(cdpr.get_cable_length())
        time.sleep(0.05)
    cdpr.set_motor_velo(0, 0, 0, 0)

