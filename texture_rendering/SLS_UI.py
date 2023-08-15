'''
Author: Mingxin Zhang m.zhang@hapis.k.u-tokyo.ac.jp
Date: 2023-06-01 16:46:22
LastEditors: Mingxin Zhang
LastEditTime: 2023-08-15 17:29:54
Copyright (c) 2023 by Mingxin Zhang, All Rights Reserved. 
'''
import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QPen, QPainterPath
import math
import pySequentialLineSearch

class SinusoidWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 200)
        self._amplitude = [1.0, 1.0, 1.0]
        self._frequency = [1.0, 1.0, 1.0]
        self._phase = [0.0, 0.0, 0.0]
        self._offset = [0.5, 0.5, 0.5]
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(self.backgroundRole(), Qt.white)
        self.setPalette(palette)

    def setAmplitude(self, amplitude):
        self._amplitude = amplitude
        self.update()

    def setOffset(self, offset):
        self._offset = offset
        self.update()

    def setFrequency(self, frequency):
        self._frequency = frequency
        self.update()

    def setPhase(self, phase):
        self._phase = phase
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(Qt.blue, 1))

        width = self.width()
        height = self.height()
        x_scale = width / (0.5 * math.pi)
        y_scale = height

        path = QPainterPath()
        path.moveTo(0, height / 2)

        line_thickness = 1
        painter.setPen(QPen(Qt.blue, line_thickness))
        print(self._amplitude)
        print(self._frequency)
        for x in range(width):
            t = x / x_scale
            y = 0.5 * self._amplitude[0] * math.sin(self._frequency[0] * t + self._phase[0]) + self._offset[0]\
              + 0.5 * self._amplitude[1] * math.sin(self._frequency[1] * t + self._phase[1]) + self._offset[1]\
              + 0.5 * self._amplitude[2] * math.sin(self._frequency[2] * t + self._phase[2]) + self._offset[2]
            y = y / 3
            path.lineTo(x, height - y * y_scale)
        painter.drawPath(path)

        # 绘制坐标轴
        axis_thickness = 10
        painter.setPen(QPen(Qt.black, axis_thickness))
        painter.drawLine(0, height, width, height)
        painter.drawLine(0, 0, 0, height)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.horizontal_slider = QSlider(Qt.Horizontal)
        self.horizontal_slider.setRange(0, 999)
        self.horizontal_slider.setSliderPosition(0)

        self.vertical_sliders = []

        self.sinusoid_widget = SinusoidWidget()

        layout = QVBoxLayout()
        layout.addWidget(self.sinusoid_widget)

        horizontal_layout = QHBoxLayout()
        labels = ["F_STM", "R", "F_low", "A_low", "P_low", "F_mid", "A_mid", "P_mid", "F_high", "A_high", "P_high"]
        for i in range(11):
            vertical_slider = QSlider(Qt.Vertical)
            vertical_slider.setRange(0, 100)
            vertical_slider.setEnabled(False)
            self.vertical_sliders.append(vertical_slider)

            label = QLabel(labels[i])

            vertical_box = QVBoxLayout()
            vertical_box.addWidget(label, 1, Qt.AlignCenter | Qt.AlignTop)
            vertical_box.addWidget(vertical_slider, 0, Qt.AlignCenter | Qt.AlignTop)

            horizontal_layout.addLayout(vertical_box)

        layout.addLayout(horizontal_layout)
        layout.addWidget(self.horizontal_slider)

        self.optimizer = pySequentialLineSearch.SequentialLineSearchOptimizer(num_dims=11)

        self.optimizer.set_hyperparams(kernel_signal_var=0.50,
                                kernel_length_scale=0.10,
                                kernel_hyperparams_prior_var=0.10)
        
        self.optimizer.set_gaussian_process_upper_confidence_bound_hyperparam(5.)

        self.horizontal_slider.valueChanged.connect(lambda value: 
                                                    self.updateValues(_update_optimizer_flag=False))

        next_button = QPushButton("Next")
        next_button.clicked.connect(lambda value: self.updateValues(_update_optimizer_flag=True))
        layout.addWidget(next_button)
        self.setLayout(layout)

        self.updateValues(_update_optimizer_flag=False)

    def updateValues(self, _update_optimizer_flag):
        slider_position = self.horizontal_slider.value() / 999.0

        if _update_optimizer_flag:
            self.optimizer.submit_feedback_data(slider_position)
            print('Next')

        optmized_para = self.optimizer.calc_point_from_slider_position(slider_position)

        stm_freq = 3 + optmized_para[0] * 7     # STM_freq: 3~10Hz
        radius = 2 + optmized_para[1] * 3       # STM radius: 2~5mm

        freq_l = int(1 + optmized_para[2] * 7)
        amp_l = optmized_para[3]
        phase_l = optmized_para[4] * 2 * math.pi

        freq_m = int(25 + optmized_para[5] * 40)
        amp_m = optmized_para[6]
        phase_m = optmized_para[7] * 2 * math.pi

        freq_h = int(200 + optmized_para[8] * 250)
        amp_h = optmized_para[9]
        phase_h = optmized_para[10] * 2 * math.pi

        # print('f_STM:', stm_freq, '\tradius: ', radius, '\tf_wave: ', freq, '\tamp: ', amp)

        # offset = -0.5 * amp + 1
        self.sinusoid_widget.setAmplitude([amp_l, amp_m, amp_h])
        # self.sinusoid_widget.setOffset(offset)
        self.sinusoid_widget.setFrequency([freq_l, freq_m, freq_h])
        self.sinusoid_widget.setPhase([phase_l, phase_m, phase_h])

        i = 0
        for vertical_slider in self.vertical_sliders:
            vertical_slider.setValue(int(optmized_para[i] * vertical_slider.maximum()))
            i += 1


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
