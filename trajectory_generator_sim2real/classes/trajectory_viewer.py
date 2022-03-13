# -*- coding: utf-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import colorsys

class TrajectoryViewer:
    def __init__(self, poses, dot_hz, disp_period, robot = None, speed_profile = None):
        self.poses = poses
        self.dot_hz = dot_hz
        self.disp_period = disp_period
        self.robot = robot
        self.speed_profile = speed_profile

    def createRobot(self, pose, robot=None, ec='#000000', fc='#FFFFFF', fill=False):
        # ロボットの形状定義
        point = []
        point.append((+0.160, +0.120))
        point.append((+0.160, +0.050))
        point.append((+0.200, +0.050))
        point.append((+0.200, -0.050))
        point.append((+0.160, -0.050))
        point.append((+0.160, -0.120))
        point.append((-0.160, -0.120))
        point.append((-0.160, +0.120))

        # 回転変換
        rotated_point = np.empty((0,2), float)
        for i, p in enumerate(point):
            x = pose[0]+(p[0]*np.cos(pose[2]) - p[1]*np.sin(pose[2]))
            y = pose[1]+(p[0]*np.sin(pose[2]) + p[1]*np.cos(pose[2]))
            rotated_point = np.append(rotated_point, np.array([[x, y]]), axis=0)

        return plt.Polygon(rotated_point, ec=ec, fc=fc, fill=fill)

    def createRectangle(self, pose, width, height, ec='#000000', fc='#FFFFFF', fill=False):
        point = []
        point.append((width/2, +height/2))
        point.append((-width/2, +height/2))
        point.append((-width/2, -height/2))
        point.append((+width/2, -height/2))

        for i, p in enumerate(point):
            x = pose[0]+(p[0]*np.cos(pose[2]) - p[1]*np.sin(pose[2]))
            y = pose[1]+(p[0]*np.sin(pose[2]) + p[1]*np.cos(pose[2]))
            point[i] = (x, y)

        return plt.Polygon((point[0], point[1], point[2], point[3]), ec=ec, fc=fc, fill=fill)

    def createRectangleBy2Points(self, pose1, pose2, ec='#000000', fc='#FFFFFF', fill=False):
        w = abs(pose2[0] - pose1[0])
        h = abs(pose2[1] - pose1[1])
        p = pose1
        if p[0] > pose2[0]:
            p[0] = pose2[0]
        if p[1] > pose2[1]:
            p[1] = pose2[1]

        points = [p, p, p, p]
        points[0] = p
        points[1] = [p[0] + w, p[1]]
        points[2] = [p[0] + w, p[1] + h]
        points[3] = [p[0], p[1] + h]

        return plt.Polygon((points[0], points[1], points[2], points[3]), ec=ec, fc=fc, fill=fill)

    def display(self):
        fig = plt.figure()
        ax = plt.axes()

        # フィールド構造物の描画        # フィールド構造物の描画
        field = []
        field.append(plt.Polygon(((-0.320, +3.940),
                                  (+3.680, +3.940),
                                  (+3.680, -1.060),
                                  (-0.320, -1.060)),
                                ec='#000000', fill=False))

        # Starting Zone
        field.append(self.createRectangleBy2Points([-0.320, +0.198], [+0.085, -0.120]))

        # Mineral Zone
        field.append(self.createRectangleBy2Points([-0.320, +3.940], [+0.080, +3.540]))
        field.append(self.createRectangleBy2Points([+0.238, +2.777], [+0.638, +2.377]))
        field.append(self.createRectangleBy2Points([+2.537, +3.224], [+2.937, +2.824]))
        field.append(self.createRectangleBy2Points([+1.659, +0.634], [+2.059, +0.234]))
        field.append(self.createRectangleBy2Points([+3.266, -0.696], [+3.666, -1.096]))

        # Exchnage Station
        field.append(self.createRectangleBy2Points([+1.694, +2.475], [+2.166, +2.339]))
        field.append(self.createRectangleBy2Points([+1.838, +2.339], [+2.011, +1.501]))
        field.append(self.createRectangleBy2Points([+1.694, +1.501], [+2.166, +1.371]))


        # Obstacle
        field.append(plt.Polygon(((+0.638, +2.997),
                                  (+0.878, +2.997),
                                  (+0.878, +2.137),
                                  (+0.278, +2.137),
                                  (+0.278, +2.377),
                                  (+0.638, +2.377)),
                                ec='#000000', fill=False))
        field.append(plt.Polygon(((+1.430, +0.745),
                                  (+1.659, +0.745),
                                  (+1.659, -0.267),
                                  (+1.430, -0.267)),
                                ec='#000000', fill=False))
        field.append(plt.Polygon(((-0.320, -0.307),
                                  (+0.221, -0.307),
                                  (+0.455, -0.542),
                                  (+0.455, -1.060),
                                  (-0.320, -1.060)),
                                ec='#000000', fill=False))

        field.append(plt.Polygon(((-0.320, +3.940),
                                  (+0.116, +3.940),
                                  (+0.116, +3.770),
                                  (-0.053, +3.530),
                                  (-0.320, +3.530)),
                                ec='#000000', fill=False))

        # Hill
        field.append(plt.Polygon(((+1.641, +3.940),
                                  (+1.641, +3.224),
                                  (+2.937, +3.224),
                                  (+2.937, +0.908),
                                  (+2.598, +0.908),
                                  (+2.598, +0.137),
                                  (+2.937, +0.137),
                                  (+2.937, -0.366),
                                  (+3.680, -0.366),
                                  (+3.680, +3.940)),
                                ec='#000000', fill=False))

        for f in field:
            ax.add_patch(f)

        # 速度に応じて打点色を変える
        color_list = []
        max_speed = max(self.speed_profile)
        for speed in self.speed_profile:
            rgb = colorsys.hsv_to_rgb(0.7*(1-(speed / max_speed)), 1, 1)
            color_list.append([rgb[0], rgb[1], rgb[2]])

        for i, pose in enumerate(self.poses):
            ax.scatter(pose[0], pose[1], marker='.', color=color_list[i])
            if i == 0:
                continue
            if (i%int(self.disp_period*self.dot_hz) == 0):
                robot_patch = self.createRobot(
                    [pose[0], pose[1], pose[2]],
                    robot = self.robot,
                    ec='green')
                ax.add_patch(robot_patch)

        # 始点
        robot_patch = self.createRobot(
            [self.poses[0][0], self.poses[0][1], self.poses[0][2]],
            robot = self.robot,
            ec='blue')
        ax.add_patch(robot_patch)

        # 終点
        robot_patch = self.createRobot(
            [self.poses[-1][0], self.poses[-1][1], self.poses[-1][2]],
            robot = self.robot,
            ec='blue')
        ax.add_patch(robot_patch)

        plt.title("robot:" + self.robot + ", draw period:" + str(self.disp_period) + "[sec]")
        plt.axis('scaled')
        ax.set_aspect('equal')

        plt.show()
