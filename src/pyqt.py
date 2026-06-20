import sys
import os
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import cv2 as cv
import numpy as np

from AprilTag import AprilTagProcessor
from FakeRobot import FakeRobot

from PyQt6.QtCore import (
    Qt,
    QTimer
)

from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QTextEdit,
    QGridLayout,
    QGroupBox,
    QVBoxLayout
)

from PyQt6.QtGui import (
    QImage,
    QPixmap
)


class Dashboard(QWidget):

    def __init__(self):
        super().__init__()

        self.counter = 0
        self.setWindowTitle("Robot Dashboard")
        self.resize(1280, 800)

        self.layout = QGridLayout()
        self.setStatusBox()
        self.setup_camera()
        # self.setCameraBox()
        self.setControlsBox()
        self.setup_robotXZ()
        self.setup_robotXY()
        self.setRobotXZBox()
        self.setRobotXYBox()
        self.setup_robot3D()
        self.setLogBox()
        self.setDashboardLayout()

    def setStatusBox(self):
        # -------------------------
        # Status Panel
        # -------------------------
        self.status_box = QGroupBox("Status")
        status_layout = QVBoxLayout()

        status_layout.addWidget(QLabel("State: WALKING"))
        status_layout.addWidget(QLabel("Battery: 11.8V"))
        status_layout.addWidget(QLabel("Load Cell: 3.2N"))

        self.status_box.setLayout(status_layout)

    def setup_camera(self):
        self.apriltag = AprilTagProcessor()

        self.camera_box = QGroupBox("Camera Feed")
        camera_layout = QVBoxLayout()
        self.camera_label = QLabel("Camera goes here")
        self.camera_label.setMinimumSize(300, 200)


        camera_layout.addWidget(self.camera_label)
        self.camera_box.setLayout(camera_layout)

        self.camera_timer = QTimer()

        self.camera_timer.timeout.connect(
            self.setCameraBox
        )

        self.camera_timer.start(30)

    def setCameraBox(self):
        # -------------------------
        # Camera Placeholder
        # -------------------------

        frame, results = self.apriltag.process_image()
        frame = self.apriltag.detect_tags(frame, results)
        rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        h, w, ch = rgb.shape

        qimg = QImage(
            rgb.data,
            w,
            h,
            ch * w,
            QImage.Format.Format_RGB888
        )

        qimg = qimg.scaled(
            self.camera_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio
        )

        self.camera_label.setPixmap(QPixmap.fromImage(qimg))

    def restart_program(self):
        os.execv(sys.executable, [sys.executable] + sys.argv)

    def setControlsBox(self):
        # -------------------------
        # Controls
        # -------------------------
        self.control_box = QGroupBox("Controls")
        control_layout = QVBoxLayout()

        control_layout.addWidget(QPushButton("Connect"))
        control_layout.addWidget(QPushButton("Enable"))
        control_layout.addWidget(QPushButton("Disable"))
        self.reload_button = QPushButton("Restart Dashboard")
        self.reload_button.clicked.connect(self.restart_program)

        control_layout.addWidget(self.reload_button)
        self.control_box.setLayout(control_layout)



    def setup_robotXZ(self):
        # -------------------------
        # Robot XZ Visualization Setup
        # -------------------------
        self.robot_box_xz = QGroupBox("Robot XZ Visualization")
        self.robot_plot_xz = pg.PlotWidget()
        self.robot_plot_xz.setAspectLocked(True)
        self.robot_plot_xz.setXRange(-300, 300)
        self.robot_plot_xz.setYRange(-300, 300)

        robot_layout = QVBoxLayout()
        robot_layout.addWidget(self.robot_plot_xz)

        self.robot_box_xz.setLayout(robot_layout)

        self.robot = FakeRobot()
        self.robot_timer = QTimer()

        # Robot Coordinates
        self.x_axis = self.robot_plot_xz.plot(
            [0, 25],
            [0, 0],
            pen=pg.mkPen('g', width=3)
        )

        self.z_axis = self.robot_plot_xz.plot(
            [0, 0],
            [0, 25],
            pen=pg.mkPen('b', width=3)
        )

        self.y_label = pg.TextItem("Y ⊙", color='r')
        self.y_label.setPos(0,0)

        self.robot_plot_xz.addItem(self.y_label)

        # Example robot leg
        self.robot_curve_legA_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legB_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legC_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legD_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legE_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legF_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        
        # neutral position of the servos
        self.coxa_neutral_legA_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legA_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legA_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.coxa_neutral_legD_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legD_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legD_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.coxa_neutral_legB_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legB_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legB_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.coxa_neutral_legC_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legC_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legC_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.coxa_neutral_legF_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legF_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legF_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.coxa_neutral_legE_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legE_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legE_xz = self.robot_plot_xz.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )
        
        
        self.robot_timer.timeout.connect(
            self.setRobotXZBox
        )

        self.robot_timer.start(200)

    def setRobotXZBox(self):
        if self.counter % 52 < 26:
            self.robot.tripodWalkLeft()
            self.counter += 1
        elif self.counter % 52 >= 26:
            self.robot.tripodWalkRight()
            self.counter += 1
        else:
            print("undefined movements")
            print(self.counter)


        legA_x, legA_y, legA_z = self.robot.LegA.LegFK(0)
        legD_x, legD_y, legD_z = self.robot.LegD.LegFK(0)

        legF_x, legF_y, legF_z = self.robot.LegF.LegFK(60)
        legE_x, legE_y, legE_z = self.robot.LegE.LegFK(60)

        legB_x, legB_y, legB_z = self.robot.LegB.LegFK(-60)
        legC_x, legC_y, legC_z = self.robot.LegC.LegFK(-60)

        
        self.robot_curve_legA_xz.setData(legA_x, legA_z)
        self.robot_curve_legD_xz.setData(legD_x, legD_z)

        self.robot_curve_legF_xz.setData(legF_x, legF_z)
        self.robot_curve_legE_xz.setData(legE_x, legE_z)

        self.robot_curve_legB_xz.setData(legB_x, legB_z)
        self.robot_curve_legC_xz.setData(legC_x, legC_z)

        
        # TODO: clean up the arguments here so easier to negate for mirror
        self.coxa_neutral_legA_xz.setData(
            [self.robot.LegA.basePos.x, self.robot.LegA.basePos.x + 20],
            [self.robot.LegA.basePos.z, self.robot.LegA.basePos.z + 0]
        )
        self.femur_neutral_legA_xz.setData(
            [self.robot.LegA.coxaPos.x, self.robot.LegA.coxaPos.x + 20*np.cos(np.deg2rad(self.robot.LegA.femur.midAngle-90))],
            [self.robot.LegA.coxaPos.z, self.robot.LegA.coxaPos.z + 20*np.sin(np.deg2rad(self.robot.LegA.femur.midAngle-90))]
        )
        self.tibia_neutral_legA_xz.setData(
            [self.robot.LegA.femurPos.x, self.robot.LegA.femurPos.x - 20*np.cos(np.deg2rad(self.robot.LegA.tibia.midAngle + self.robot.LegA.femur.angle))],
            [self.robot.LegA.femurPos.z, self.robot.LegA.femurPos.z - 20*np.sin(np.deg2rad(self.robot.LegA.tibia.midAngle + self.robot.LegA.femur.angle))]
        )
        
        self.coxa_neutral_legD_xz.setData(
            [self.robot.LegD.basePos.x, self.robot.LegD.basePos.x + self.robot.LegD.factor*20],
            [self.robot.LegD.basePos.z, self.robot.LegD.basePos.z + 0]
        )
        self.femur_neutral_legD_xz.setData(
            [self.robot.LegD.coxaPos.x, self.robot.LegD.coxaPos.x + self.robot.LegD.factor*20*np.cos(np.deg2rad(self.robot.LegD.femur.midAngle-90))],
            [self.robot.LegD.coxaPos.z, self.robot.LegD.coxaPos.z + 20*np.sin(np.deg2rad(self.robot.LegD.femur.midAngle-90))]
        )
        self.tibia_neutral_legD_xz.setData(
            [self.robot.LegD.femurPos.x, self.robot.LegD.femurPos.x - self.robot.LegD.factor*20*np.cos(np.deg2rad(self.robot.LegD.tibia.midAngle + self.robot.LegD.femur.angle))],
            [self.robot.LegD.femurPos.z, self.robot.LegD.femurPos.z - 20*np.sin(np.deg2rad(self.robot.LegD.tibia.midAngle + self.robot.LegD.femur.angle))]
        )

        # Front corner Leg
        self.coxa_neutral_legB_xz.setData(
            [self.robot.LegB.basePos.x, self.robot.LegB.basePos.x + 20],
            [self.robot.LegB.basePos.z, self.robot.LegB.basePos.z + 0]
        )
        self.femur_neutral_legB_xz.setData(
            [self.robot.LegB.coxaPos.x, self.robot.LegB.coxaPos.x + 20*np.cos(np.deg2rad(self.robot.LegB.femur.midAngle-90))],
            [self.robot.LegB.coxaPos.z, self.robot.LegB.coxaPos.z + 20*np.sin(np.deg2rad(self.robot.LegB.femur.midAngle-90))]
        )
        self.tibia_neutral_legB_xz.setData(
            [self.robot.LegB.femurPos.x, self.robot.LegB.femurPos.x - 20*np.cos(np.deg2rad(self.robot.LegB.tibia.midAngle + self.robot.LegB.femur.angle))],
            [self.robot.LegB.femurPos.z, self.robot.LegB.femurPos.z - 20*np.sin(np.deg2rad(self.robot.LegB.tibia.midAngle + self.robot.LegB.femur.angle))]
        )
        
        self.coxa_neutral_legC_xz.setData(
            [self.robot.LegC.basePos.x, self.robot.LegC.basePos.x + self.robot.LegC.factor*20],
            [self.robot.LegC.basePos.z, self.robot.LegC.basePos.z + 0]
        )
        self.femur_neutral_legC_xz.setData(
            [self.robot.LegC.coxaPos.x, self.robot.LegC.coxaPos.x + self.robot.LegC.factor*20*np.cos(np.deg2rad(self.robot.LegC.femur.midAngle-90))],
            [self.robot.LegC.coxaPos.z, self.robot.LegC.coxaPos.z + 20*np.sin(np.deg2rad(self.robot.LegC.femur.midAngle-90))]
        )
        self.tibia_neutral_legC_xz.setData(
            [self.robot.LegC.femurPos.x, self.robot.LegC.femurPos.x - self.robot.LegC.factor*20*np.cos(np.deg2rad(self.robot.LegC.tibia.midAngle + self.robot.LegC.femur.angle))],
            [self.robot.LegC.femurPos.z, self.robot.LegC.femurPos.z - 20*np.sin(np.deg2rad(self.robot.LegC.tibia.midAngle + self.robot.LegC.femur.angle))]
        )

        # Back corner Leg
        self.coxa_neutral_legF_xz.setData(
            [self.robot.LegF.basePos.x, self.robot.LegF.basePos.x + 20],
            [self.robot.LegF.basePos.z, self.robot.LegF.basePos.z + 0]
        )
        self.femur_neutral_legF_xz.setData(
            [self.robot.LegF.coxaPos.x, self.robot.LegF.coxaPos.x + 20*np.cos(np.deg2rad(self.robot.LegF.femur.midAngle-90))],
            [self.robot.LegF.coxaPos.z, self.robot.LegF.coxaPos.z + 20*np.sin(np.deg2rad(self.robot.LegF.femur.midAngle-90))]
        )
        self.tibia_neutral_legF_xz.setData(
            [self.robot.LegF.femurPos.x, self.robot.LegF.femurPos.x - 20*np.cos(np.deg2rad(self.robot.LegF.tibia.midAngle + self.robot.LegF.femur.angle))],
            [self.robot.LegF.femurPos.z, self.robot.LegF.femurPos.z - 20*np.sin(np.deg2rad(self.robot.LegF.tibia.midAngle + self.robot.LegF.femur.angle))]
        )
        
        self.coxa_neutral_legE_xz.setData(
            [self.robot.LegE.basePos.x, self.robot.LegE.basePos.x + self.robot.LegE.factor*20],
            [self.robot.LegE.basePos.z, self.robot.LegE.basePos.z + 0]
        )
        self.femur_neutral_legE_xz.setData(
            [self.robot.LegE.coxaPos.x, self.robot.LegE.coxaPos.x + self.robot.LegE.factor*20*np.cos(np.deg2rad(self.robot.LegE.femur.midAngle-90))],
            [self.robot.LegE.coxaPos.z, self.robot.LegE.coxaPos.z + 20*np.sin(np.deg2rad(self.robot.LegE.femur.midAngle-90))]
        )
        self.tibia_neutral_legE_xz.setData(
            [self.robot.LegE.femurPos.x, self.robot.LegE.femurPos.x - self.robot.LegE.factor*20*np.cos(np.deg2rad(self.robot.LegE.tibia.midAngle + self.robot.LegE.femur.angle))],
            [self.robot.LegE.femurPos.z, self.robot.LegE.femurPos.z - 20*np.sin(np.deg2rad(self.robot.LegE.tibia.midAngle + self.robot.LegE.femur.angle))]
        )


    def setup_robotXY(self):
        # -------------------------
        # Robot XY Visualization
        # -------------------------
        self.robot_box_xy = QGroupBox("Robot XY Visualization")
        self.robot_plot_xy = pg.PlotWidget()
        self.robot_plot_xy.setAspectLocked(True)
        self.robot_plot_xy.setXRange(-300, 300)
        self.robot_plot_xy.setYRange(-300, 300)

        robot_layout = QVBoxLayout()
        robot_layout.addWidget(self.robot_plot_xy)

        self.robot_box_xy.setLayout(robot_layout)

        # Robot Coordinates
        self.x_axis = self.robot_plot_xy.plot(
            [0, 25],
            [0, 0],
            pen=pg.mkPen('g', width=3)
        )

        self.y_axis = self.robot_plot_xy.plot(
            [0, 0],
            [0, -25],
            pen=pg.mkPen('r', width=3)
        )

        self.y_label = pg.TextItem("z ⊙", color='b')
        self.y_label.setPos(0,0)

        self.robot_plot_xy.addItem(self.y_label)

        # Example robot leg
        self.robot_curve_legA_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legD_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legF_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legE_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legB_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        self.robot_curve_legC_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('w', width = 3)
        )

        # neutral position of the servos
        self.coxa_neutral_legA_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legA_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legA_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.coxa_neutral_legD_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legD_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legD_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )


        self.coxa_neutral_legF_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legF_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legF_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.coxa_neutral_legE_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legE_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legE_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )


        self.coxa_neutral_legB_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
    
        self.femur_neutral_legB_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legB_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.coxa_neutral_legC_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width=3)
        )
        
        self.femur_neutral_legC_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )

        self.tibia_neutral_legC_xy = self.robot_plot_xy.plot(
            [],
            [],
            pen=pg.mkPen('y', width = 3)
        )
        
        
        self.robot_timer.timeout.connect(
            self.setRobotXYBox
        )

        self.robot_timer.start(200)

    def setRobotXYBox(self):
        if self.counter % 52 < 26:
            self.robot.tripodWalkLeft()
            self.counter += 1
        elif self.counter % 52 >= 26:
            self.robot.tripodWalkRight()
            self.counter += 1
        else:
            print("undefined movements")
            print(self.counter)

        legA_x, legA_y, legA_z = self.robot.LegA.LegFK(0)
        legD_x, legD_y, legD_z = self.robot.LegD.LegFK(0)

        legF_x, legF_y, legF_z = self.robot.LegF.LegFK(60)
        legE_x, legE_y, legE_z = self.robot.LegE.LegFK(60)

        legB_x, legB_y, legB_z = self.robot.LegB.LegFK(-60)
        legC_x, legC_y, legC_z = self.robot.LegC.LegFK(-60)


        self.robot_curve_legA_xy.setData(legA_x, legA_y)
        self.robot_curve_legD_xy.setData(legD_x, legD_y)

        self.robot_curve_legF_xy.setData(legF_x, legF_y)
        self.robot_curve_legE_xy.setData(legE_x, legE_y)

        self.robot_curve_legB_xy.setData(legB_x, legB_y)
        self.robot_curve_legC_xy.setData(legC_x, legC_y)


        
        # TODO: clean up the arguments here so easier to call
        self.coxa_neutral_legA_xy.setData(
            [self.robot.LegA.basePos.x, self.robot.LegA.basePos.x + 20*np.cos(np.deg2rad(self.robot.LegA.coxa.midAngle-90))],
            [self.robot.LegA.basePos.y, self.robot.LegA.basePos.y + 20*np.sin(np.deg2rad(self.robot.LegA.coxa.midAngle-90))]
        )
        self.femur_neutral_legA_xy.setData(
            [self.robot.LegA.coxaPos.x, self.robot.LegA.coxaPos.x + 20*np.cos(np.deg2rad(self.robot.LegA.coxa.angle-90))],
            [self.robot.LegA.coxaPos.y, self.robot.LegA.coxaPos.y + 20*np.sin(np.deg2rad(self.robot.LegA.coxa.angle-90))]
        )
        self.tibia_neutral_legA_xy.setData(
            [self.robot.LegA.femurPos.x, self.robot.LegA.femurPos.x - 20*np.cos(np.deg2rad(self.robot.LegA.coxa.angle-90))],
            [self.robot.LegA.femurPos.y, self.robot.LegA.femurPos.y - 20*np.sin(np.deg2rad(self.robot.LegA.coxa.angle-90))]
        )
        
        self.coxa_neutral_legD_xy.setData(
            [self.robot.LegD.basePos.x, self.robot.LegD.basePos.x + self.robot.LegD.factor*20*np.cos(np.deg2rad(self.robot.LegD.coxa.midAngle-90))],
            [self.robot.LegD.basePos.y, self.robot.LegD.basePos.y + 20*np.sin(np.deg2rad(self.robot.LegD.coxa.midAngle-90))]
        )
        self.femur_neutral_legD_xy.setData(
            [self.robot.LegD.coxaPos.x, self.robot.LegD.coxaPos.x + self.robot.LegD.factor*20*np.cos(np.deg2rad(self.robot.LegD.coxa.angle-90))],
            [self.robot.LegD.coxaPos.y, self.robot.LegD.coxaPos.y + 20*np.sin(np.deg2rad(self.robot.LegD.coxa.angle-90))]
        )
        self.tibia_neutral_legD_xy.setData(
            [self.robot.LegD.femurPos.x, self.robot.LegD.femurPos.x - self.robot.LegD.factor*20*np.cos(np.deg2rad(self.robot.LegD.coxa.angle-90))],
            [self.robot.LegD.femurPos.y, self.robot.LegD.femurPos.y - 20*np.sin(np.deg2rad(self.robot.LegD.coxa.angle-90))]
        )
        
        # Front corner legs
        self.coxa_neutral_legF_xy.setData(
            [self.robot.LegF.basePos.x, self.robot.LegF.basePos.x + 20*np.cos(np.deg2rad(self.robot.LegF.coxa.midAngle-30))],
            [self.robot.LegF.basePos.y, self.robot.LegF.basePos.y + 20*np.sin(np.deg2rad(self.robot.LegF.coxa.midAngle-30))]
        )
        self.femur_neutral_legF_xy.setData(
            [self.robot.LegF.coxaPos.x, self.robot.LegF.coxaPos.x + 20*np.cos(np.deg2rad(self.robot.LegF.coxa.angle-30))],
            [self.robot.LegF.coxaPos.y, self.robot.LegF.coxaPos.y + 20*np.sin(np.deg2rad(self.robot.LegF.coxa.angle-30))]
        )
        self.tibia_neutral_legF_xy.setData(
            [self.robot.LegF.femurPos.x, self.robot.LegF.femurPos.x - 20*np.cos(np.deg2rad(self.robot.LegF.coxa.angle-30))],
            [self.robot.LegF.femurPos.y, self.robot.LegF.femurPos.y - 20*np.sin(np.deg2rad(self.robot.LegF.coxa.angle-30))]
        )

        self.coxa_neutral_legE_xy.setData(
            [self.robot.LegE.basePos.x, self.robot.LegE.basePos.x + self.robot.LegE.factor*20*np.cos(np.deg2rad(self.robot.LegE.coxa.midAngle-30))],
            [self.robot.LegE.basePos.y, self.robot.LegE.basePos.y + 20*np.sin(np.deg2rad(self.robot.LegE.coxa.midAngle-30))]
        )
        self.femur_neutral_legE_xy.setData(
            [self.robot.LegE.coxaPos.x, self.robot.LegE.coxaPos.x + self.robot.LegE.factor*20*np.cos(np.deg2rad(self.robot.LegE.coxa.angle-30))],
            [self.robot.LegE.coxaPos.y, self.robot.LegE.coxaPos.y + 20*np.sin(np.deg2rad(self.robot.LegE.coxa.angle-30))]
        )
        self.tibia_neutral_legE_xy.setData(
            [self.robot.LegE.femurPos.x, self.robot.LegE.femurPos.x - self.robot.LegE.factor*20*np.cos(np.deg2rad(self.robot.LegE.coxa.angle-30))],
            [self.robot.LegE.femurPos.y, self.robot.LegE.femurPos.y - 20*np.sin(np.deg2rad(self.robot.LegE.coxa.angle-30))]
        )

        #Back corner legs
        self.coxa_neutral_legB_xy.setData(
            [self.robot.LegB.basePos.x, (self.robot.LegB.basePos.x - 20*np.cos(np.deg2rad(self.robot.LegB.coxa.midAngle+30)))],
            [self.robot.LegB.basePos.y, (self.robot.LegB.basePos.y - 20*np.sin(np.deg2rad(self.robot.LegB.coxa.midAngle+30)))]
        )
        self.femur_neutral_legB_xy.setData(
            [self.robot.LegB.coxaPos.x, (self.robot.LegB.coxaPos.x + 20*np.cos(np.deg2rad(self.robot.LegB.coxa.angle+30)))],
            [self.robot.LegB.coxaPos.y, (self.robot.LegB.coxaPos.y + 20*np.sin(np.deg2rad(self.robot.LegB.coxa.angle+30)))]
        )
        self.tibia_neutral_legB_xy.setData(
            [self.robot.LegB.femurPos.x, (self.robot.LegB.femurPos.x - 20*np.cos(np.deg2rad(self.robot.LegB.coxa.angle+30)))],
            [self.robot.LegB.femurPos.y, (self.robot.LegB.femurPos.y - 20*np.sin(np.deg2rad(self.robot.LegB.coxa.angle+30)))]
        )

        self.coxa_neutral_legC_xy.setData(
            [self.robot.LegC.basePos.x, self.robot.LegC.basePos.x - self.robot.LegC.factor*20*np.cos(np.deg2rad(self.robot.LegC.coxa.midAngle+30))],
            [self.robot.LegC.basePos.y, (self.robot.LegC.basePos.y - 20*np.sin(np.deg2rad(self.robot.LegC.coxa.midAngle+30)))]
        )
        self.femur_neutral_legC_xy.setData(
            [self.robot.LegC.coxaPos.x, self.robot.LegC.coxaPos.x + self.robot.LegC.factor*20*np.cos(np.deg2rad(self.robot.LegC.coxa.angle+30))],
            [self.robot.LegC.coxaPos.y, (self.robot.LegC.coxaPos.y + 20*np.sin(np.deg2rad(self.robot.LegC.coxa.angle+30)))]
        )
        self.tibia_neutral_legC_xy.setData(
            [self.robot.LegC.femurPos.x, self.robot.LegC.femurPos.x - self.robot.LegC.factor*20*np.cos(np.deg2rad(self.robot.LegC.coxa.angle+30))],
            [self.robot.LegC.femurPos.y, (self.robot.LegC.femurPos.y - 20*np.sin(np.deg2rad(self.robot.LegC.coxa.angle+30)))]
        )
        

    def setup_robot3D(self):

        self.robot_box_3d = QGroupBox("Robot 3D Visualization")

        self.robot_view = gl.GLViewWidget()
        self.robot_view.setCameraPosition(
            distance=600,
            elevation=25,
            azimuth=45
        )

        robot_layout = QVBoxLayout()
        robot_layout.addWidget(self.robot_view)
        self.robot_box_3d.setLayout(robot_layout)

        # Ground grid
        grid = gl.GLGridItem()
        grid.scale(20, 20, 20)
        self.robot_view.addItem(grid)

        # XYZ axes
        x_axis = gl.GLLinePlotItem(
            pos=np.array([[0,0,0],[100,0,0]]),
            color=(0,1,0,1),
            width=3
        )

        y_axis = gl.GLLinePlotItem(
            pos=np.array([[0,0,0],[0,100,0]]),
            color=(1,0,0,1),
            width=3
        )

        z_axis = gl.GLLinePlotItem(
            pos=np.array([[0,0,0],[0,0,100]]),
            color=(0,0,1,1),
            width=3
        )

        self.robot_view.addItem(x_axis)
        self.robot_view.addItem(y_axis)
        self.robot_view.addItem(z_axis)

        # One line per leg
        self.leg3d_A = gl.GLLinePlotItem(width=4)
        self.leg3d_B = gl.GLLinePlotItem(width=4)
        self.leg3d_C = gl.GLLinePlotItem(width=4)
        self.leg3d_D = gl.GLLinePlotItem(width=4)
        self.leg3d_E = gl.GLLinePlotItem(width=4)
        self.leg3d_F = gl.GLLinePlotItem(width=4)

        self.robot_view.addItem(self.leg3d_A)
        self.robot_view.addItem(self.leg3d_B)
        self.robot_view.addItem(self.leg3d_C)
        self.robot_view.addItem(self.leg3d_D)
        self.robot_view.addItem(self.leg3d_E)
        self.robot_view.addItem(self.leg3d_F)

    def update_robot3D(self):
        self.robot.LegA.LegFK(0)
        self.robot.LegD.LegFK(0)

        self.robot.LegF.LegFK(60)
        self.robot.LegE.LegFK(60)

        self.robot.LegB.LegFK(-60)
        self.robot.LegC.LegFK(-60)

        def leg_points(leg):
            return np.array([
                [leg.basePos.x,  leg.basePos.y,  leg.basePos.z],
                [leg.coxaPos.x,  leg.coxaPos.y,  leg.coxaPos.z],
                [leg.femurPos.x, leg.femurPos.y, leg.femurPos.z],
                [leg.tibiaPos.x, leg.tibiaPos.y, leg.tibiaPos.z]
            ])

        self.leg3d_A.setData(pos=leg_points(self.robot.LegA))
        self.leg3d_B.setData(pos=leg_points(self.robot.LegB))
        self.leg3d_C.setData(pos=leg_points(self.robot.LegC))
        self.leg3d_D.setData(pos=leg_points(self.robot.LegD))
        self.leg3d_E.setData(pos=leg_points(self.robot.LegE))
        self.leg3d_F.setData(pos=leg_points(self.robot.LegF))


    def setLogBox(self):
        # -------------------------
        # Log Console
        # -------------------------
        self.log_box = QGroupBox("Console")

        self.console = QTextEdit()
        self.console.setReadOnly(True)

        log_layout = QVBoxLayout()
        log_layout.addWidget(self.console)

        self.console.append("Robot Ready")
        # log_layout.addWidget(self.status_label)

        self.log_box.setLayout(log_layout)

    def setDashboardLayout(self):
        # -------------------------
        # Layout
        # -------------------------
        self.layout.addWidget(self.status_box, 0, 0)
        self.layout.addWidget(self.camera_box, 0, 1)
        self.layout.addWidget(self.control_box, 0, 2)

        self.layout.addWidget(self.robot_box_xz, 1, 0)
        self.layout.addWidget(self.robot_box_xy, 1, 1)
        self.layout.addWidget(self.robot_box_3d, 1, 2)

        self.layout.addWidget(self.log_box, 2, 0, 1, 3)

        self.setLayout(self.layout)

app = QApplication(sys.argv)

# Create a Qt Widget, which is our window
window = Dashboard()

window.show() # windows are hidden by default

# start the event loop
sys.exit(app.exec())