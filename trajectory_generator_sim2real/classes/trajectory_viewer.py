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
        point.append((+0.120, +0.160))
        point.append((+0.050, +0.160))
        point.append((+0.050, +0.200))
        point.append((-0.050, +0.200))
        point.append((-0.050, +0.160))
        point.append((-0.120, +0.160))
        point.append((-0.120, -0.160))
        point.append((+0.120, -0.160))

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

        # フィールド構造物の描画
        field = []
        field.append(plt.Polygon(((+0.000, -0.000),
                                  (+5.000, -0.000),
                                  (+5.000, -4.000),
                                  (+0.000, -4.000)),
                                ec='#000000', fill=False))

        # Starting Zone
        field.append(self.createRectangleBy2Points([+3.74, -3.59], [+4.06, -4.00]))

        # Mineral Zone
        field.append(self.createRectangleBy2Points([+0.00, -3.60], [+0.40, -4.00]))
        field.append(self.createRectangleBy2Points([+1.16, -3.04], [+1.56, -3.44]))
        field.append(self.createRectangleBy2Points([+0.71, -1.43], [+1.11, -1.14]))
        field.append(self.createRectangleBy2Points([+3.30, -1.62], [+3.70, -2.02]))
        field.append(self.createRectangleBy2Points([+4.63, -0.13], [+5.03, -0.41]))

        # Exchnage Station
        field.append(self.createRectangleBy2Points([+1.46, -1.51], [+1.60, -1.98]))
        field.append(self.createRectangleBy2Points([+1.60, -1.66], [+2.43, -1.84]))
        field.append(self.createRectangleBy2Points([+2.43, -1.51], [+2.56, -1.98]))
        

        # Obstacle
        field.append(plt.Polygon(((+0.00, -3.35),
                                  (+0.23, -3.35),
                                  (+0.40, -3.73),
                                  (+0.40, -4.00),
                                  (+0.00, -4.00)),
                                ec='#000000', fill=False))
        field.append(plt.Polygon(((+0.94, -2.80),
                                  (+1.80, -2.80),
                                  (+1.80, -3.40),
                                  (+1.56, -3.40),
                                  (+1.56, -3.04),
                                  (+0.94, -3.04)),
                                ec='#000000', fill=False))
        field.append(plt.Polygon(((+3.19, -2.02),
                                  (+4.20, -2.02),
                                  (+4.20, -2.24),
                                  (+3.19, -2.24)),
                                ec='#000000', fill=False))
                                
        field.append(plt.Polygon(((+4.24, -4.00),
                                  (+4.24, -3.45),
                                  (+4.42, -3.22),
                                  (+5.00, -3.22),
                                  (+5.00, -4.00)),
                                ec='#000000', fill=False))

        # Hill
        field.append(plt.Polygon(((+0.00, -0.00),
                                  (+4.30, -0.00),
                                  (+4.30, -0.74),
                                  (+3.80, -0.74),
                                  (+3.80, -1.08),
                                  (+3.03, -1.08),
                                  (+3.03, -0.74),
                                  (+0.71, -0.74),
                                  (+0.71, -2.03),
                                  (+0.00, -2.03)),
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
