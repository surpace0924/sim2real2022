# -*- coding: utf-8 -*-

import os
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication, QMainWindow, QMenu, QVBoxLayout, QSizePolicy, QMessageBox, QWidget, QPushButton, QComboBox, QListView, QLabel, QTableWidgetItem
from PyQt5.QtGui import QIcon
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as patches
import numpy as np
import colorsys

# NHK
# x_range = [-6.1, 6.1]
# y_range = [-6.1, 4.1]

# ABU
x_range = [-0.37, 3.73]
y_range = [-1.11, 3.99]

def createRectangle(pose, width, height, ec='#000000', fc='#FFFFFF', fill=False):
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

def createRectangleBy2Points(pose1, pose2, ec='#000000', fc='#FFFFFF', fill=False):
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

def resizeByRange(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def resizeByScale(x, a, b):
    return a * (x - b)

# [px]から[m]へ単位を変換
def convertToMeter(px, scale, origin):
    result = [0, 0]
    result[0] = scale * (px[0] - origin[0])
    result[1] = scale * (-px[1] + origin[1])
    return result

# [m]から[px]へ単位を変換
def convertToPx(meter, scale, origin):
    result = [0, 0]
    result[0] = origin[0] + (meter[0] / scale)
    result[1] = origin[1] - (meter[1] / scale)
    return result


class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        # 各種パラメータ
        self.parent = parent
        self.origin = [380, 380]                                        # 原点の座標[px]
        self.scale = 0.01593                                            # 縮尺[m/px]
        self.map_path = os.path.dirname(__file__) + "/../map/2021.png"  # map画像のパス
        self.control_point = None
        self.traj = None
        self.origin_point = None

        # 描画エリアの初期設定
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = fig.add_subplot(111)
        self.ax.set_position([0, 0, 1, 1])
        FigureCanvas.__init__(self, fig)
        self.setParent(parent)
        FigureCanvas.setSizePolicy(self,
                                   QSizePolicy.Expanding,
                                   QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        self.add_zoom_func()
        self.plot()

        # キャンバスクリック時のイベント関数を登録
        cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.drawOrigin(self.origin)

    def plot(self):
        # フィールド構造物の描画
        field = []
        field.append(plt.Polygon(((-0.320, +3.940),
                                  (+3.680, +3.940),
                                  (+3.680, -1.060),
                                  (-0.320, -1.060)),
                                ec='#000000', fill=False))

        # Starting Zone
        field.append(createRectangleBy2Points([-0.320, +0.198], [+0.085, -0.120]))

        # Mineral Zone
        field.append(createRectangleBy2Points([-0.320, +3.940], [+0.080, +3.540]))
        field.append(createRectangleBy2Points([+0.238, +2.777], [+0.638, +2.377]))
        field.append(createRectangleBy2Points([+2.537, +3.224], [+2.937, +2.824]))
        field.append(createRectangleBy2Points([+1.659, +0.634], [+2.059, +0.234]))
        field.append(createRectangleBy2Points([+3.266, -0.696], [+3.666, -1.096]))

        # Exchnage Station
        field.append(createRectangleBy2Points([+1.694, +2.475], [+2.166, +2.339]))
        field.append(createRectangleBy2Points([+1.838, +2.339], [+2.011, +1.501]))
        field.append(createRectangleBy2Points([+1.694, +1.501], [+2.166, +1.371]))


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
            self.ax.add_patch(f)

        # 軸を消す
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_xlim(x_range)
        self.ax.set_ylim(y_range)
        self.ax.set_aspect('equal')

        # 描画
        self.draw()

    # 原点の描画
    def drawOrigin(self, point):
        self.origin = point
        # すでに点が描かれている場合はそれを消す
        if self.origin_point != None:
            self.origin_point[0].remove()

        dot_px = []
        plot_x, plot_y = self.__convertToPlot(dot_px)
        self.origin_point = self.ax.plot(0, 0, marker='.', markersize=12, color="yellow", linestyle='None')
        self.draw()

    # mapクリック時の処理
    def onclick(self, event):
        point = [event.xdata, event.ydata]
        point.append(0)# 角度はとりあえず0としておく
        self.parent.via_points.append(point)
        self.parent.via_speed.append(0)

        # tableをクリアし，座標の数だけ行を用意
        items = self.parent.via_points
        # self.parent.tableWidget.clearContents()
        self.parent.tableWidget.setRowCount(len(items))

        # tableへの書き込み
        r = len(items)-1
        self.parent.tableWidget.setItem(
            r, 0, QTableWidgetItem('{0:.3f}'.format(items[r][0])))
        self.parent.tableWidget.setItem(
            r, 1, QTableWidgetItem('{0:.3f}'.format(items[r][1])))
        self.parent.tableWidget.setItem(
            r, 2, QTableWidgetItem('{0:.3f}'.format(items[r][2])))
        self.parent.tableWidget.setItem(
            r, 3, QTableWidgetItem('{0:.3f}'.format(0)))
        # print()


    def drawControlPoint(self, point):
        # すでに点が描かれている場合はそれを消す
        if self.control_point != None:
            self.control_point[0].remove()

        plot_x = [l[0] for l in point]
        plot_y = [l[1] for l in point]
        self.control_point = self.ax.plot(
            plot_x, plot_y, marker='.', markersize=12, color="green", linestyle='None')
        self.draw()

    def drawTrajectory(self, dots, speed_profile):
        ax = self.figure.axes[0]
        dots = np.array(dots)
        dot_px = []

        # すでに経路が描かれている場合はそれを消す
        if self.traj != None:
            self.traj.remove()

        plot_x = [l[0] for l in dots]
        plot_y = [l[1] for l in dots]

        color_list = []
        max_speed = max(speed_profile)
        for speed in speed_profile:
            rgb = colorsys.hsv_to_rgb(0.7*(1-(speed / max_speed)), 1, 1)
            color_list.append([rgb[0], rgb[1], rgb[2]])

        # 描画
        self.traj = self.ax.scatter(plot_x, plot_y, marker='.', c=color_list)
        # print(len(self.traj))
        # self.traj = self.ax.plot(plot_x, plot_y, marker='.', colors="red")
        self.draw()

    # Matplotlibに入れるために座標リストを2つのベクトルに変換する
    # [[x1, y1], [x2, y2], ...] -> [x1, x2, ...], [y1, y2, ...]
    def __convertToPlot(self, matrix):
        x = []
        y = []
        for i in range(len(matrix)):
            x.append(matrix[i][0])
            y.append(matrix[i][1])
        return x, y

    # スクロールで拡大縮小するやつ
    def add_zoom_func(self, base_scale=1.5):
        def zoom_func(event):
            bbox = self.ax.get_window_extent()
            if not(bbox.x0 < event.x < bbox.x1):
                return
            if not(bbox.y0 < event.y < bbox.y1):
                return
            if event.xdata is None or event.ydata is None:
                return
            cur_xlim = self.ax.get_xlim()
            cur_ylim = self.ax.get_ylim()
            cur_xrange = (cur_xlim[1] - cur_xlim[0]) * .5
            cur_yrange = (cur_ylim[1] - cur_ylim[0]) * .5
            xdata = event.xdata  # get event x location
            ydata = event.ydata  # get event y location
            # print(event.button, event.x, event.y, event.xdata, event.ydata)
            if event.button == 'up':
                # deal with zoom in
                scale_factor = base_scale
            elif event.button == 'down':
                # deal with zoom out
                scale_factor = 1 / base_scale
            else:
                # deal with something that should never happen
                scale_factor = 1
                # print(event.button)
            # set new limits
            xlim = [xdata - (xdata - cur_xlim[0]) / scale_factor,
                    xdata + (cur_xlim[1] - xdata) / scale_factor]
            ylim = [ydata - (ydata - cur_ylim[0]) / scale_factor,
                    ydata + (cur_ylim[1] - ydata) / scale_factor]

            self.ax.set_xlim(xlim)
            self.ax.set_ylim(ylim)

            if (abs(xlim[1] - xlim[0]) > x_range[1] - x_range[0]) and (abs(ylim[1] - ylim[0]) > y_range[1] - y_range[0]):
                self.ax.set_xlim(x_range)
                self.ax.set_ylim(y_range)

            self.figure.canvas.draw()
        self.figure.canvas.mpl_connect('scroll_event', zoom_func)
