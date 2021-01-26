#!/usr/bin/env python3


import sys
from PyQt5.QtWidgets import (QWidget, QPushButton,QHBoxLayout, QVBoxLayout, QApplication,  QSlider, QLabel)
from PyQt5.QtCore import Qt

class SliderDisplay(QWidget):

    def __init__(self, name, min, max, ticks, parent = None):
        super(SliderDisplay, self).__init__(parent)
        self.name  = name
        self.min   = min
        self.max   = max
        self.ticks = ticks




        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(self.min)
        self.slider.setMaximum(self.ticks)
        # self.b1 = QPushButton('Quit')
        self.label = QLabel(self.name + ' {}'.format(1.0/self.ticks))
        hbox = QHBoxLayout()
        hbox.addWidget(self.label)
        hbox.addWidget(self.slider)
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        # vbox.addWidget(self.b1)
        self.setLayout(vbox)
        # self.b1.clicked.connect(app.exit)
        self.slider.valueChanged.connect(self.v_change)
        self.setGeometry(300, 300, 250, 150)
        self.show()

    def v_change(self):
        my_value = float(self.slider.value())/self.ticks*self.max

        self.label.setText('{0}: {1:.3f}'.format(self.name, my_value))
        #' ' + str(my_value))

def main_f(name, min, max, ticks):
    app = QApplication(sys.argv)
    ex = SliderDisplay(name, min, max, ticks)
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main_f('Try', 0, 1, 100)
