#!/usr/bin/env python3


import sys
from PyQt5.QtWidgets import (QWidget, QPushButton,QHBoxLayout, QVBoxLayout, QApplication,  QSlider, QLabel)
from PyQt5.QtCore import Qt

class SliderDisplay(QWidget):

    def __init__(self, name, minnie, maxus, ticks, parent = None):
        super(SliderDisplay, self).__init__(parent)
        self.name = name
        self.minnie = minnie
        self.maxus = maxus
        self.ticks = ticks



        self.title = 'Lab 8'
        self.setWindowTitle(self.title)
        self.s1 = QSlider(Qt.Horizontal)
        self.s1.setMinimum(self.minnie)
        self.s1.setMaximum(self.maxus)
        self.b1 = QPushButton('Quit')
        self.l1 = QLabel(self.name + ' {}'.format(1/self.ticks))
        hbox = QHBoxLayout()
        hbox.addWidget(self.l1)
        hbox.addWidget(self.s1)
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        # vbox.addWidget(self.b1)
        self.setLayout(vbox)
        # self.b1.clicked.connect(app.exit)
        self.s1.valueChanged.connect(self.v_change)
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('SliderDisplay')
        self.show()

    def v_change(self):
        my_value = self.s1.value()/1000
        self.l1.setText(self.name + ' ' + str(my_value))

def main_f(main, minnie, maxus, ticks):
    app = QApplication(sys.argv)
    ex = SliderDisplay(main, minnie, maxus, ticks)
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main_f('foo', 1, 1000, 100)
