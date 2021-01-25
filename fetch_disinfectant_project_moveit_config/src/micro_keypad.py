#!/usr/bin/env python3


from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QVBoxLayout, QLabel, QSlider, QHBoxLayout, QGridLayout
from PyQt5.QtCore import Qt

class Interface(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setWindowTitle('Microwave Keypad Interface')
        self.grid_label = QLabel('EXPRESS COOK')
        self.grid_label.setAlignment(Qt.AlignCenter)
        # A widget to hold everything
        widget = QWidget()
        self.setCentralWidget(widget)

        # A Layout of the first row of options
        hbox1 = QHBoxLayout()
        hbox1.addWidget(QPushButton('POPCORN'))
        hbox1.addWidget(QPushButton('POTATO'))
        hbox1.addWidget(QPushButton('PIZZA'))
        hbox1.addWidget(QPushButton('BEVERAGE'))

       # A Layout of the second row of options
        hbox2 = QHBoxLayout()
        hbox2.addWidget(QPushButton('SOUP'))
        hbox2.addWidget(QPushButton('DINNER PLATE'))
        hbox2.addWidget(QPushButton('FRESH VEGETABLE'))
        hbox2.addWidget(QPushButton('FROZEN VEGETABLE'))

        # A Layout of the last row of options
        hbox3 = QHBoxLayout()
        hbox3.addWidget(QPushButton('STOP/CLEAR'))
        hbox3.addWidget(QPushButton('START +30SEC'))

        # A Layout of the number grid
        grid1 = QGridLayout()
        grid1.addWidget(self.grid_label, 0,1)
        grid1.addWidget(QPushButton('1'), 1, 0)
        grid1.addWidget(QPushButton('2'), 1, 1)
        grid1.addWidget(QPushButton('3'), 1, 2)
        grid1.addWidget(QPushButton('4'), 2, 0)
        grid1.addWidget(QPushButton('5'), 2, 1)
        grid1.addWidget(QPushButton('6'), 2, 2)
        grid1.addWidget(QPushButton('7'), 3, 0)
        grid1.addWidget(QPushButton('8'), 3, 1)
        grid1.addWidget(QPushButton('9'), 3, 2)
        grid1.addWidget(QPushButton('AM/PM'), 4, 0)
        grid1.addWidget(QPushButton('0'), 4, 1)
        grid1.addWidget(QPushButton('CLOCK PRE-SET'), 4, 2)

        # A Layout of the vertical buttons next to the number grid
        vbox1 = QVBoxLayout()
        vbox1.addWidget(QPushButton('POWER'))
        vbox1.addWidget(QPushButton('KITCHEN TIMER'))
        vbox1.addWidget(QPushButton('WEIGHT DEFROST'))
        vbox1.addWidget(QPushButton('TIME DEFROST'))
        vbox1.addWidget(QPushButton('MEMORY'))

        # A Horizontal merging of the vertical button layout (vbox1) with the number grid layout(grid)
        sub_layout = QHBoxLayout()
        sub_layout.addLayout(vbox1)
        sub_layout.addLayout(grid1)

        # A Vertical merging of all the layouts.
        layout = QVBoxLayout()
        layout.addLayout(hbox1)
        layout.addLayout(hbox2)
        layout.addLayout(sub_layout)
        layout.addLayout(hbox3)
        widget.setLayout(layout)

if __name__ == '__main__':
    app = QApplication([])
    interface = Interface()
    interface.show()
    app.exec_()
