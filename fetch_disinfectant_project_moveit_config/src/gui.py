#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QSlider, QGroupBox
from PyQt5.QtCore import Qt
from slider import SliderDisplay


class Interface(QMainWindow):

    def __init__(self):
        super(Interface,self).__init__()
        self.gui_input_pub = rospy.Publisher('gui_input', String, queue_size = 10)
        self.setWindowTitle('Fetch Disinfectant Project')
        self.setGeometry(300, 300, 600, 400)
        # self.s1 = SliderDisplay('Attenuation', 0, 1, 100)
        self.initUI()


    def initUI(self):
        global app
        self.quit_button = QPushButton('Quit')
        self.compute_irr = QPushButton('Compute Irradiantion')
        self.compute_time= QPushButton('Compute Time')
        self.plan        = QPushButton('Plan Path')
        self.execute     = QPushButton('Execute Path')
        self.init_pose   = QPushButton('Initial Pose')
        self.tuck_pose   = QPushButton('Tuck Arm')

        # A widget to hold everything
        widget = QWidget()
        self.setCentralWidget(widget)

        # A Layout of computing the Irradiation
        irradiation = QGroupBox('Irradiation')
        irradiation_layout = QVBoxLayout()
        self.n = SliderDisplay('Attenuation', 0, 1, 100)
        self.P = SliderDisplay('Power Rating (W)',0,50,100)
        self.A = SliderDisplay('Area (m^2)', 0 , 1, 100)
        self.I_label = QLabel('Irradation (W/m^2): ')
        irradiation_layout.addWidget(self.n)
        irradiation_layout.addWidget(self.P)
        irradiation_layout.addWidget(self.A)
        irradiation_layout.addWidget(self.I_label)
        irradiation_layout.addWidget(self.compute_irr)
        irradiation.setLayout(irradiation_layout)

        # A Layout of computing Time at each waypoint
        time_exposure = QGroupBox('Time Exposure')
        time_exposure_layout = QVBoxLayout()
        self.S = SliderDisplay('Survival (%) ',0, 10, 1000)
        self.k = SliderDisplay('Constant',0,1,100000)
        # self.A = SliderDisplay('Area (m^2)', 0 , 1, 100)
        self.time_label = QLabel('Time Exposure (sec): ')
        time_exposure_layout.addWidget(self.S)
        time_exposure_layout.addWidget(self.k)
        # irradiation_layout.addWidget(self.A)
        time_exposure_layout.addWidget(self.time_label)
        time_exposure_layout.addWidget(self.S)
        time_exposure_layout.addWidget(self.k)
        time_exposure_layout.addWidget(self.compute_time)
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



        self.quit_button.clicked.connect(app.exit)
        self.compute_irr.clicked.connect(self.compute_irradiation)
        self.plan.clicked.connect(self.publish_command)
        self.execute.clicked.connect(self.publish_command_b)
        self.init_pose.clicked.connect(self.publish_command_c)
        self.tuck_pose.clicked.connect(self.publish_command_d)
        self.show()

    def publish_command(self):
        # self.gui_input_pub.publish("0")
        print(0)
    def publish_command_b(self):
        # self.gui_input_pub.publish("1")
        print(1)
    def publish_command_c(self):
        # self.gui_input_pub.publish("2")
        print(2)
    def publish_command_d(self):
        # self.gui_input_pub.publish("3")
        print(3)

    def compute_irradiation(self):
        n = self.n.value()
        P = self.P.value()
        A = self.A.value()
        I = n * P / A
        self.I_label.setText('Irradation (W/m^2): {0:.2f}'.format(I))

def run():
    # rospy.init_node('gui_interface')
    global app
    app = QApplication(sys.argv)
    interface = Interface()
    sys.exit(app.exec_())


if __name__ == '__main__':
    run()
