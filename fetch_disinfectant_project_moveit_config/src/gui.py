#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton,QGridLayout, QHBoxLayout, QVBoxLayout, QLabel, QSlider, QGroupBox, QRadioButton
from PyQt5.QtCore import Qt
from slider import SliderDisplay


class Interface(QMainWindow):

    def __init__(self):
        super(Interface,self).__init__()
        self.gui_input_pub = rospy.Publisher('gui_input', String, queue_size = 10)
        self.setWindowTitle('Fetch Disinfectant Project')
        self.setGeometry(300, 300, 600, 400)
        self.initUI()


    def initUI(self):
        global app
        self.quit_button = QPushButton('Quit')
        self.plan        = QPushButton('Plan Path')
        self.execute     = QPushButton('Execute Path')
        self.init_pose   = QPushButton('Initial Pose')
        self.tuck_pose   = QPushButton('Tuck Arm')

        self.Attenuation_slider = QSlider(Qt.Horizontal)
        self.Attenuation_slider.setMinimum(1)
        self.Attenuation_slider.setMaximum(100)
        self.Attenuation_label = QLabel('Attenuation: {}'.format(0.01))
        # self.n = 0.01

        self.Power_slider = QSlider(Qt.Horizontal)
        self.Power_slider.setMinimum(1)
        self.Power_slider.setMaximum(50)
        self.Power_label = QLabel('Power Rating (W): {}'.format(1))
        # self.P = 1

        self.Area_slider = QSlider(Qt.Horizontal)
        self.Area_slider.setMinimum(1)
        self.Area_slider.setMaximum(1000)
        self.Area_label = QLabel('Area (m^2): {}'.format(0.01))
        # self.A = 0.01

        self.UV_constant_slider = QSlider(Qt.Horizontal)
        self.UV_constant_slider.setMinimum(1)
        self.UV_constant_slider.setMaximum(9000)
        self.UV_constant_label = QLabel('UV rate constant (m^2/J): {}'.format(0.001))
        # self.k = 0.01

        self.I_label = QLabel('Irradiation (W/m^2): ')
        self.Time_label = QLabel('Time Exposure (sec): ')

        self.S = 0.1


        # A widget to hold everything
        widget = QWidget()
        self.setCentralWidget(widget)


        # A Layout of computing the Irradiation
        irradiation = QGroupBox('Irradiation')
        irradiation_layout = QVBoxLayout()

        hbox = QVBoxLayout()
        hbox.addWidget(self.Attenuation_label)
        hbox.addWidget(self.Attenuation_slider)
        irradiation_layout.addLayout(hbox)

        hbox = QVBoxLayout()
        hbox.addWidget(self.Power_label)
        hbox.addWidget(self.Power_slider)
        irradiation_layout.addLayout(hbox)

        hbox = QVBoxLayout()
        hbox.addWidget(self.Area_label)
        hbox.addWidget(self.Area_slider)
        irradiation_layout.addLayout(hbox)

        # irradiation_layout.setSpacing(100)
        irradiation_layout.addWidget(self.I_label)
        irradiation.setLayout(irradiation_layout)


        # A Layout of computing Time at each waypoint
        time_exposure = QGroupBox('Time Exposure')
        time_exposure_layout = QVBoxLayout()

        sub_layout = QGridLayout()
        self.disinfectant_label = QLabel(' Disinfection Percentage: ')
        sub_layout.addWidget(self.disinfectant_label, 0, 0)

        self.disinfectant_button = QRadioButton('90%')
        self.disinfectant_button.setChecked(True)
        self.disinfectant_button.country = 0.1
        self.disinfectant_button.toggled.connect(self.onClicked)
        sub_layout.addWidget(self.disinfectant_button, 0, 1)

        self.disinfectant_button = QRadioButton('99%')
        self.disinfectant_button.country = 0.01
        self.disinfectant_button.toggled.connect(self.onClicked)
        sub_layout.addWidget(self.disinfectant_button, 0, 2)

        self.disinfectant_button = QRadioButton("99.9%")
        self.disinfectant_button.country = 0.001
        self.disinfectant_button.toggled.connect(self.onClicked)
        sub_layout.addWidget(self.disinfectant_button, 0, 3)

        self.disinfectant_button = QRadioButton("99.99%")
        self.disinfectant_button.country = 0.0001
        self.disinfectant_button.toggled.connect(self.onClicked)
        sub_layout.addWidget(self.disinfectant_button, 0, 4)

        self.disinfectant_button = QRadioButton("99.999%")
        self.disinfectant_button.country = 0.00001
        self.disinfectant_button.toggled.connect(self.onClicked)
        sub_layout.addWidget(self.disinfectant_button, 0, 5)

        time_exposure_layout.addLayout(sub_layout)

        hbox = QVBoxLayout()
        hbox.addWidget(self.UV_constant_label)
        hbox.addWidget(self.UV_constant_slider)
        time_exposure_layout.addLayout(hbox)


        time_exposure_layout.addWidget(self.Time_label)
        time_exposure.setLayout(time_exposure_layout)


        # A Horizontal Layout of arm control for two configurations
        plan_execute = QGroupBox('Plan and Execute')
        h_box = QHBoxLayout()
        h_box.addWidget(self.plan)
        h_box.addWidget(self.execute)
        plan_execute.setLayout(h_box)


        # A Horizontal Layout of arm control for two configurations
        control = QGroupBox('Control')
        h_box = QHBoxLayout()
        h_box.addWidget(self.init_pose)
        h_box.addWidget(self.tuck_pose)
        control.setLayout(h_box)


        # Vertical merging of all the layouts
        layout = QVBoxLayout()
        widget.setLayout(layout)

        layout.addWidget(irradiation)
        layout.addWidget(time_exposure)
        layout.addWidget(plan_execute)
        layout.addWidget(control)
        layout.addWidget(self.quit_button)

        # Buttons and sliders clicked/connect
        self.quit_button.clicked.connect(app.exit)

        self.plan.clicked.connect(self.publish_command)
        self.execute.clicked.connect(self.publish_command_b)
        self.init_pose.clicked.connect(self.publish_command_c)
        self.tuck_pose.clicked.connect(self.publish_command_d)

        self.Attenuation_slider.valueChanged.connect(self.value_change)
        self.Power_slider.valueChanged.connect(self.value_change)
        self.Area_slider.valueChanged.connect(self.value_change)
        self.UV_constant_slider.valueChanged.connect(self.value_change)

        self.show()

    def publish_command(self):
        self.gui_input_pub.publish("0")
        # print(0)
    def publish_command_b(self):
        self.gui_input_pub.publish("1")
        # print(1)
    def publish_command_c(self):
        self.gui_input_pub.publish("2")
        # print(2)
    def publish_command_d(self):
        self.gui_input_pub.publish("3")
        # print(3)

    def onClicked(self):
        radioButton = self.sender()
        if radioButton.isChecked():
            self.S = radioButton.country
        self.value_change()

    def value_change(self):

        self.n = float(self.Attenuation_slider.value())/(100)
        self.Attenuation_label.setText('Attenuation: {}'.format(self.n))

        self.P = float(self.Power_slider.value())/2
        self.Power_label.setText('Power Rating (W): {}'.format(self.P))

        self.A = float(self.Area_slider.value())/(1000)
        self.Area_label.setText('Area (m^2):  {}'.format(self.A))

        self.I = self.n * self.P / self.A
        self.I_label.setText('Irradiation (W/m^2): {0:.2f}'.format(self.I))

        self.k = float(self.UV_constant_slider.value())/(10**5)
        self.UV_constant_label.setText('UV rate constant (m^2/J): {0:.5f}'.format(self.k))

        T = -np.log(self.S)/(self.k * self.I)
        self.Time_label.setText('Time exposure per waypoint (sec): {0:.2f}'.format(T))



def run():
    rospy.init_node('gui')
    global app
    app = QApplication(sys.argv)
    interface = Interface()
    sys.exit(app.exec_())


if __name__ == '__main__':
    run()
