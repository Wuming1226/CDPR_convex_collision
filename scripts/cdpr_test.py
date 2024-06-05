#! /usr/bin/env python3

import time
import numpy as np

from wcs import Edge, WCS
from calculate_separation_v2 import calculate_cable_length


class CDPR:

    def __init__(self):
        # ros settings
        # rospy.init_node('cdpr_control', anonymous=True)

        # subscriber and publisher
        self._moving_platform_pose = np.array([0, 0, 0])
        # rospy.Subscriber('/vrpn_client_node/end_effector/pose', PoseStamped, self._pose_callback, queue_size=1)

        self._motor_pos = np.array([0, 0, 0, 0])
        # rospy.Subscriber('motor_pos', Int64MultiArray, self._motor_pos_callback, queue_size=1)

        # self._veloPub = rospy.Publisher('motor_velo', Int16MultiArray, queue_size=10)

        # 测量数据
        Om1 = np.array([0.363, 0.264, 0.172])
        Om2 = np.array([0.137, 0.264, 0.172])
        Om3 = np.array([0.136, 0.039, 0.172])
        Om4 = np.array([0.362, 0.039, 0.172])
        Ob1 = np.array([0.363, 0.264, 0.040])
        Ob2 = np.array([0.137, 0.264, 0.040])
        Ob3 = np.array([0.136, 0.039, 0.036])
        Ob4 = np.array([0.363, 0.039, 0.036])
        center_x = (Ob1[0] + Ob2[0] + Ob3[0] + Ob4[0]) / 4
        center_y = (Ob1[1] + Ob2[1] + Ob3[1] + Ob4[1]) / 4
        Ot = np.array([0.242, 0.145, 0.336])

        # origin point offset (coordinates in world frame)
        self.xOff = center_x
        self.yOff = center_y
        self.zOff = -0.00
        self.pos_off = np.array([self.xOff, self.yOff, self.zOff])

        # anchor positions in the world frame
        self._anchorA1 = np.array([0.342, 0.342, 0.732])
        self._anchorA2 = np.array([-0.342, 0.342, 0.732])
        self._anchorA3 = np.array([-0.342, -0.342, 0.732])
        self._anchorA4 = np.array([0.342, -0.342, 0.735])

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
        self._ori_cable_length = np.array([0, 0, 0, 0])
        self._motor_pos0 = np.array([0, 0, 0, 0])

    def init_cable_length(self, cable1_flag, cable2_flag, cable3_flag, cable4_flag):
        # calculate origin cable lengths
        time.sleep(1)
        x0, y0, z0, _ = self.get_moving_platform_pose()
        pos0 = np.array([x0, y0, z0])
        motor_pos0 = self._motor_pos
        if cable1_flag:
            self._ori_cable_length[0] = np.linalg.norm(pos0 - self._anchorA1)
            self._motor_pos0[0] = motor_pos0[0]
        if cable2_flag:
            self._ori_cable_length[1] = np.linalg.norm(pos0 - self._anchorA2)
            self._motor_pos0[1] = motor_pos0[1]
        if cable3_flag:
            self._ori_cable_length[2] = np.linalg.norm(pos0 - self._anchorA3)
            self._motor_pos0[2] = motor_pos0[2]
        if cable4_flag:
            self._ori_cable_length[3] = np.linalg.norm(pos0 - self._anchorA4)
            self._motor_pos0[3] = motor_pos0[3]

    def _pose_callback(self, data):
        # if motion data is lost(999999), do not update
        if np.abs(data.pose.position.x) > 2000 or np.abs(data.pose.position.y) > 2000 or np.abs(data.pose.position.z) > 2000:
            pass
        else:
            # pose
            self._moving_platform_pose.pose.position.x = data.pose.position.x / 1000
            self._moving_platform_pose.pose.position.y = data.pose.position.y / 1000
            self._moving_platform_pose.pose.position.z = data.pose.position.z / 1000
            self._moving_platform_pose.pose.orientation = data.pose.orientation

            # header
            self._moving_platform_pose.header.frame_id = data.header.frame_id
            self._moving_platform_pose.header.stamp = data.header.stamp

    def _motor_pos_callback(self, data):
        diff = np.array(data.data) - self._motor_pos
        if (np.abs(diff[0]) > 100000 or np.abs(diff[1]) > 1000000 or np.abs(diff[2]) > 1000000 or np.abs(diff[3]) > 1000000) \
            and (self._motor_pos[0] != 0 and self._motor_pos[1] != 0 and self._motor_pos[2] != 0 and self._motor_pos[3] != 0):
            pass
        else:
            self._motor_pos = np.array(data.data)

    def update_cable_state(self, b_pos):
        waypoints1 = self.wcs1.update(b_pos)
        print("--------")
        waypoints2 = self.wcs2.update(b_pos)
        print("--------")
        waypoints3 = self.wcs3.update(b_pos)
        print("--------")
        waypoints4 = self.wcs4.update(b_pos)
        return ([waypoints1, waypoints2, waypoints3, waypoints4],
                [calculate_cable_length(self._anchorA1, waypoints1, b_pos),
                calculate_cable_length(self._anchorA2, waypoints2, b_pos),
                calculate_cable_length(self._anchorA3, waypoints3, b_pos),
                calculate_cable_length(self._anchorA4, waypoints4, b_pos)])

    def set_motor_velo(self, motor1Velo, motor2Velo, motor3Velo, motor4Velo):
        motor_velo = Int16MultiArray(data=np.array([motor1Velo, motor2Velo, motor3Velo, motor4Velo]))
        self._veloPub.publish(motor_velo)

    def get_moving_platform_pose(self):
        return self._moving_platform_pose.pose.position.x, self._moving_platform_pose.pose.position.y, self._moving_platform_pose.pose.position.z,\
            [self._moving_platform_pose.pose.orientation.x, self._moving_platform_pose.pose.orientation.y, self._moving_platform_pose.pose.orientation.z, self._moving_platform_pose.pose.orientation.w]

    def get_cable_length(self):
        r = 0.03 * np.pi / 10000 / 10
        cable_length = (self._motor_pos - self._motor_pos0) * r + self._cable_length0
        return cable_length
    
    def pretighten(self, cable1_flag, cable2_flag, cable3_flag, cable4_flag):

        if cable1_flag:
            time.sleep(0.5)
            # cable1 pre-tightening
            print('cable1 pretightening...')
            self.set_motor_velo(-100, 0, 0, 0)
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
            self.set_motor_velo(0, -100, 0, 0)
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
            self.set_motor_velo(0, 0, -100, 0)
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
            self.set_motor_velo(0, 0, 0, -100)
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
        self.set_motor_velo(600, 600, 600)
        time.sleep(0.2)
        self.set_motor_velo(0, 0, 0)
        time.sleep(0.5)

    def get_anchorA_poses(self):
        return [self._anchorA1, self._anchorA2, self._anchorA3, self._anchorA4]

    def get_anchorB_poses(self):
        return [self._anchorB1, self._anchorB2, self._anchorB3, self._anchorB4]



if __name__=="__main__":

    cdpr = CDPR()
    # rate = rospy.Rate(10)
    # cdpr.pretighten()
    cdpr.init_cable_length(True, True, True, True)
    print(cdpr._cable_length0)
    time.sleep(1)
    cdpr.set_motor_velo(0, 0, 0, 0)
    start_time = time.time()
    while time.time() - start_time < 1:
        print(cdpr.get_cable_length())
        time.sleep(0.05)
    cdpr.set_motor_velo(0, 0, 0, 0)

