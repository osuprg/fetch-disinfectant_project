#!/usr/bin/env python3


import sys
from PyQt5.QtWidgets import (QWidget, QPushButton,QHBoxLayout, QVBoxLayout, QApplication, QLineEdit, QSlider, QLabel)
from PyQt5.QtCore import Qt
from slider import SliderDisplay

class Example(QWidget):

    def __init__(self):
        super(Example,self).__init__()
        self.initUI()
    def initUI(self):
        self.title = 'Spring-Mass-Damper Playground'
        self.setWindowTitle(self.title)
        self.sM = QSlider(Qt.Horizontal)
        self.sM.setMinimum(1)
        self.sM.setMaximum(10000)
        self.sSp = QSlider(Qt.Horizontal)
        self.sSp.setMinimum(1)
        self.sSp.setMaximum(10000)
        self.sD = QSlider(Qt.Horizontal)
        self.sD.setMinimum(1)
        self.sD.setMaximum(10000)
        self.sT = QSlider(Qt.Horizontal)
        self.sT.setMinimum(1)
        self.sT.setMaximum(10000)
        self.sT_s = QSlider(Qt.Horizontal)
        self.sT_s.setMinimum(1)
        self.sT_s.setMaximum(10000)
        self.b1 = QPushButton('Simulate System')
        self.b2 = QPushButton('Quit')
        self.lM = QLabel('Mass: 0.001')
        self.lSp = QLabel('Spring: 0.001')
        self.lD = QLabel('Damper: 0.001')
        self.lT = QLabel('Time (s): 0.001')
        self.lT_s = QLabel('Time Step (s): 0.001')
        self.lSystem_p = QLabel('System Parameters')
        self.lSimulation_p = QLabel('Simulation Parameters')

        # self.sliderM = SliderDisplay('Mass', 1, 10000, 1000)
        # self.sliderS = SliderDisplay('Spring', 1, 10000, 1000)
        # self.sliderD = SliderDisplay('Damper', 1, 10000, 1000)
        # self.sliderT = SliderDisplay('Time (s)', 1, 10000, 1000)
        # self.sliderT_S = SliderDisplay('Time step (s)', 1, 10000, 1000)



        hbox1 = QHBoxLayout()
        hbox1.addWidget(self.lM)
        hbox1.addWidget(self.sM)
        hbox2 = QHBoxLayout()
        hbox2.addWidget(self.lSp)
        hbox2.addWidget(self.sSp)
        hbox3 = QHBoxLayout()
        hbox3.addWidget(self.lD)
        hbox3.addWidget(self.sD)
        hbox4 = QHBoxLayout()
        hbox4.addWidget(self.lT)
        hbox4.addWidget(self.sT)
        hbox5 = QHBoxLayout()
        hbox5.addWidget(self.lT_s)
        hbox5.addWidget(self.sT_s)

        vbox1 = QVBoxLayout()
        vbox1.addWidget(self.lSystem_p)
        vbox1.addLayout(hbox1)
        vbox1.addLayout(hbox2)
        vbox1.addLayout(hbox3)
        vbox1.addWidget(self.lSimulation_p)
        vbox1.addLayout(hbox4)
        vbox1.addLayout(hbox5)
        vbox1.addWidget(self.b1)
        vbox1.addSpacing(60)
        vbox1.addWidget(self.b2)

        self.setLayout(vbox1)
        self.b1.clicked.connect(self.printer)
        self.b2.clicked.connect(app.exit)
        self.sM.valueChanged.connect(self.v_change)
        self.sD.valueChanged.connect(self.v_change)
        self.sSp.valueChanged.connect(self.v_change)
        self.sT.valueChanged.connect(self.v_change)
        self.sT_s.valueChanged.connect(self.v_change)
        self.setGeometry(300, 300, 250, 300)
        self.show()
    def printer(self):
        val_M = self.sM.value() / 1000
        val_D = self.sD.value() / 1000
        val_Sp = self.sSp.value() / 1000
        val_T = self.sT.value() / 1000
        val_T_s = self.sT_s.value() / 1000
        print('Mass: {:.3f}'.format(val_M))
        print('Spring: {:.3f}'.format(val_Sp))
        print('Damper: {:.3f}'.format(val_D))
        print('Time (s): {:.3f}'.format(val_T))
        print('Time Step (s): {:.3f}'.format(val_T_s))

    def v_change(self):
        val_M = self.sM.value()/1000
        self.lM.setText('Mass: {:.3f}'.format(val_M))

        val_D = self.sD.value() / 1000
        self.lD.setText('Damper: {:.3f}'.format(val_D))

        val_Sp = self.sSp.value() / 1000
        self.lSp.setText('Spring: {:.3f}'.format(val_Sp))

        val_T = self.sT.value() / 1000
        self.lT.setText('Time (s): {:.3f}'.format(val_T))

        val_T_s = self.sT_s.value() / 1000
        self.lT_s.setText('Time Step (s): {:.3f}'.format(val_T_s))



if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())
