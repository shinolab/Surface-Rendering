import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
from PyQt5.QtGui import QPainter, QPen, QPainterPath, QPixmap
from PyQt5 import QtGui
import math
import pySequentialLineSearch
import pyrealsense2 as rs
import cv2

class SinusoidWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 200)
        self._amplitude = 1.0
        self._frequency = 1.0
        self._offset = 0.0
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

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(Qt.blue, 2))

        width = self.width()
        height = self.height()
        x_scale = width / (0.04 * math.pi)
        y_scale = height

        path = QPainterPath()
        path.moveTo(0, height / 2)

        line_thickness = 5
        painter.setPen(QPen(Qt.blue, line_thickness))
        for x in range(width):
            t = x / x_scale
            y = 0.5 * self._amplitude * math.sin(self._frequency * t) + self._offset
            path.lineTo(x, height - y * y_scale)
        painter.drawPath(path)

        # 绘制坐标轴
        axis_thickness = 10
        painter.setPen(QPen(Qt.black, axis_thickness))
        painter.drawLine(0, height, width, height)
        painter.drawLine(0, 0, 0, height)


class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Decide resolutions for both depth and rgb streaming
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)


    def run(self):
        # Start streaming
        self.pipeline.start(self.config)

        while self._run_flag:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            
            W = depth_frame.get_width()
            H = depth_frame.get_height()
            filter = rs.threshold_filter(min_dist=0, max_dist=0.23)
            depth_frame = filter.process(depth_frame)
            depth_img = np.asanyarray(depth_frame.get_data())
            depth_img = depth_img[int(H/2)-50:int(H/2)+50, int(W/2)-50:int(W/2)+50]

            mass_x, mass_y = np.where(depth_img > 0)
            if mass_x.size == 0 or mass_y.size == 0:
                continue

            # mass_x and mass_y are the list of x indices and y indices of mass pixels
            cent_x = int(np.average(mass_x))
            cent_y = int(np.average(mass_y))
            # print(cent_x, cent_y)
            height = depth_img[cent_x, cent_y]

            # depth fov of D435i: 87° x 58°
            # rgb fov of D435i: 69° x 42°
            ang_x = math.radians((cent_x - 50) / (W / 2) * (87 / 2))
            ang_y = math.radians((cent_y - 50) / (H / 2) * (58 / 2))
            x_dis = math.tan(ang_x) * height
            y_dis = math.tan(ang_y) * height

            print('X:', x_dis, 'Y:', y_dis, 'Z:', height)
            
            # put text and highlight the center
            cv2.circle(depth_img, (cent_y, cent_x), 5, (255, 255, 255), -1)
            depth_img = cv2.applyColorMap(cv2.convertScaleAbs(depth_img), cv2.COLORMAP_JET)

            self.change_pixmap_signal.emit(depth_img)

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sequential Line Search")
        self.image_disp_w_h = 320

        self.image_label = QLabel(self)
        self.image_label.resize(self.image_disp_w_h, self.image_disp_w_h)

        whole_hbox = QHBoxLayout()
        whole_hbox.addWidget(self.image_label)

        self.horizontal_slider = QSlider(Qt.Horizontal)
        self.horizontal_slider.setRange(0, 999)
        self.horizontal_slider.setSliderPosition(0)

        self.vertical_sliders = []

        self.sinusoid_widget = SinusoidWidget()

        layout = QVBoxLayout()
        layout.addWidget(self.sinusoid_widget)

        horizontal_layout = QHBoxLayout()
        labels = ["f_STM", "radius", "f_wave", "amplitude"]
        for i in range(4):
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

        self.optimizer = pySequentialLineSearch.SequentialLineSearchOptimizer(num_dims=4)

        self.optimizer.set_hyperparams(kernel_signal_var=0.50,
                                kernel_length_scale=0.10,
                                kernel_hyperparams_prior_var=0.10)
        
        self.optimizer.set_gaussian_process_upper_confidence_bound_hyperparam(5.)

        self.horizontal_slider.valueChanged.connect(lambda value: self.updateValues())

        next_button = QPushButton("Next")
        next_button.clicked.connect(lambda value: self.updateOptimizer())
        layout.addWidget(next_button)

        whole_hbox.addLayout(layout)
        self.setLayout(whole_hbox)

        self.updateValues()
        self.thread = VideoThread()
        # connect its signal to the update_image slot
        self.thread.change_pixmap_signal.connect(self.update_image)
        # start the thread
        self.thread.start()

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)
    
    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.image_disp_w_h, self.image_disp_w_h, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)
    
    def updateOptimizer(self):
        slider_position = self.horizontal_slider.value() / 999.0
        self.optimizer.submit_feedback_data(slider_position)
        print('Next')

        # optmized_para = self.optimizer.get_maximizer()
        optmized_para = self.optimizer.calc_point_from_slider_position(slider_position)

        stm_freq = 3 + optmized_para[0] * 7 # STM_freq: 3~10Hz
        radius = 2 + optmized_para[1] * 3   # STM radius: 2~5mm
        freq = 50 + optmized_para[2] * 150  # wave freq: 50~200Hz
        amp = optmized_para[3]
        print('f_STM:', stm_freq, '\tradius: ', radius, '\tf_wave: ', freq, '\tamp: ', amp)
        
        offset = -0.5 * amp + 1
        self.sinusoid_widget.setAmplitude(amp)
        self.sinusoid_widget.setOffset(offset)
        self.sinusoid_widget.setFrequency(freq)

        i = 0
        for vertical_slider in self.vertical_sliders:
            vertical_slider.setValue(int(optmized_para[i] * vertical_slider.maximum()))
            i += 1

    def updateValues(self):
        t = self.horizontal_slider.value() / 999.0
        optmized_para = self.optimizer.calc_point_from_slider_position(t)

        stm_freq = 1 + optmized_para[0] * 9 # STM_freq: 1~10Hz
        radius = 1 + optmized_para[1] * 4   # STM radius: 1~5mm
        freq = 50 + optmized_para[2] * 150  # wave freq: 50~200Hz
        amp = optmized_para[3]
        print('f_STM:', stm_freq, '\tradius: ', radius, '\tf_wave: ', freq, '\tamp: ', amp)
        
        offset = -0.5 * amp + 1
        self.sinusoid_widget.setAmplitude(amp)
        self.sinusoid_widget.setOffset(offset)
        self.sinusoid_widget.setFrequency(freq)

        i = 0
        for vertical_slider in self.vertical_sliders:
            vertical_slider.setValue(int(optmized_para[i] * vertical_slider.maximum()))
            i += 1


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
