#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from std_msgs.msg import String, Float32
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton,QGridLayout, QHBoxLayout, QVBoxLayout, QLabel, QSlider, QGroupBox, QRadioButton, QInputDialog
from PyQt5.QtCore import Qt



class Interface(QMainWindow):

    def __init__(self):
        super(Interface,self).__init__()
        self.gui_input_pub = rospy.Publisher('gui_input', String,  queue_size = 10)
        self.UV_dosage_pub = rospy.Publisher('UV_dosage', Float32, queue_size = 10)
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
        self.simulation  = QPushButton('Simulation')

        self.UV_constant_slider = QSlider(Qt.Horizontal)
        self.UV_constant_slider.setMinimum(1)
        self.UV_constant_slider.setMaximum(40000)
        self.UV_constant_label = QLabel('UV rate constant (m^2/J): {}'.format(0.001))
        # self.k = 0.01

        self.I_label      = QLabel('Irradiation (mW/cm^2): ')
        self.Dosage_label = QLabel('UV Dosage (J/m^2)')

        #
        self.S = 0.1

        # A widget to hold everything
        widget = QWidget()
        self.setCentralWidget(widget)

        # A Layout of computing Dosage at a waypoint
        UV_dosage = QGroupBox('UV Dose')
        UV_dosage_layout = QVBoxLayout()

        sub_layout = QGridLayout()
        self.disinfectant_label = QLabel(' Disinfection rate: ')
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

        UV_dosage_layout.addLayout(sub_layout)

        hbox = QVBoxLayout()
        hbox.addWidget(self.UV_constant_label)
        hbox.addWidget(self.UV_constant_slider)
        UV_dosage_layout.addLayout(hbox)


        UV_dosage_layout.addWidget(self.Dosage_label)
        UV_dosage.setLayout(UV_dosage_layout)


        # A Horizontal Layout for Plan and Execute
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


        # A Horizontal Layout for simulation after Execute
        simulation = QGroupBox('Irradiance Simulation')
        h_box = QHBoxLayout()
        h_box.addWidget(self.simulation)
        simulation.setLayout(h_box)

        # Vertical merging of all the layouts
        layout = QVBoxLayout()
        widget.setLayout(layout)

        layout.addWidget(UV_dosage)
        layout.addWidget(plan_execute)
        layout.addWidget(control)
        layout.addWidget(simulation)
        layout.addWidget(self.quit_button)

        # Buttons and sliders clicked/connect
        self.quit_button.clicked.connect(app.exit)

        self.plan.clicked.connect(self.publish_command)
        self.execute.clicked.connect(self.publish_command_b)
        self.init_pose.clicked.connect(self.publish_command_c)
        self.tuck_pose.clicked.connect(self.publish_command_d)
        self.simulation.clicked.connect(self.publish_command_e)

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
    def publish_command_e(self):
        self.gui_input_pub.publish("4")

    def onClicked(self):
        radioButton = self.sender()
        if radioButton.isChecked():
            self.S = radioButton.country
        self.value_change()

    def value_change(self):

        self.k = float(self.UV_constant_slider.value())/(10**5)
        self.UV_constant_label.setText('UV rate constant, k (m^2/J): {0:.5f}'.format(self.k))

        D = -np.log(self.S)/(self.k)
        self.Dosage_label.setText('Dose (J/m^2): {0:.2f}'.format(D))

        self.UV_dosage_pub.publish(D)

def run():
    rospy.init_node('gui')
    global app
    app = QApplication(sys.argv)
    interface = Interface()
    sys.exit(app.exec_())


if __name__ == '__main__':
    run()
