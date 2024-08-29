import os, sys, struct
import time
from datetime import datetime
import serial # pip install pyserial
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.backends.backend_qtagg import \
    NavigationToolbar2QT as NavigationToolbar
from matplotlib.backends.qt_compat import QtWidgets
from matplotlib.figure import Figure

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QCheckBox,
    QGridLayout,
    QFormLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QTextEdit,
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont

mpl.use('Qt5Agg')

class WorkerThread(QThread):
    dataReceived = pyqtSignal(str)

    def __init__(self, serial, debug=False):
        super().__init__()
        self.serial = serial
        self.debug = debug
        self._running = False
    
    def run(self):
        self._running = True
        while self._running:
            try:
                if self.serial.in_waiting > 0:
                    if self.debug:
                        data = self.serial.readline()
                        self.dataReceived.emit(','.join(map(str, [0, data.decode().strip()])))

                    else:
                        temp = self.serial.read()
                        if temp[0] == 255:
                            cmd = self.serial.read()[0]

                            # vr log
                            if (cmd < 40):
                                data = self.serial.read(10)
                                t = struct.unpack('<I', data[:4])[0]
                                y = struct.unpack('<H', data[4:6])[0]

                                self.dataReceived.emit(','.join(map(str, [cmd, t, y])))

                            # sync
                            elif (cmd < 50):
                                data = self.serial.read(6)
                                t = struct.unpack('<I', data[0:4])[0]

                                self.dataReceived.emit(','.join(map(str,[cmd, t])))
                            
                            # trial
                            elif (cmd < 70):
                                data = self.serial.read(8)
                                t = struct.unpack('<I', data[:4])[0]
                                i_trial = int(struct.unpack('<H', data[4:6])[0])
                                i_state = int(cmd - 60)
                                self.dataReceived.emit(','.join(map(str,[cmd, t, i_state, i_trial])))
                            
                            # laser
                            elif (cmd < 80):
                                data = self.serial.read(6)
                                t = struct.unpack('<I', data[:4])[0]
                                self.dataReceived.emit(','.join(map(str,[cmd, t])))
                            
                            # reward
                            elif (cmd < 90):
                                data = self.serial.read(4)
                                t = struct.unpack('<I', data[:4])[0]
                                self.dataReceived.emit(','.join(map(str,[cmd, t])))


                            elif (cmd == 99):
                                self.dataReceived.emit(str(cmd))

                        else:
                            data = (temp + self.serial.readline()).decode().strip()
                            self.dataReceived.emit(','.join(map(str,[0, data])))
            except Exception as e:
                print(e)

    def stop(self):
        self._running = False



class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.serial = None
        self.reading_thread = None
        self.fn = None
        self.file = None

        self.setWindowTitle("Avoidance Task")

        layout_main = QVBoxLayout()

        layout_upper = QHBoxLayout()
        layout_lower = QVBoxLayout()

        layout_control = QVBoxLayout()
        layout_setup = QVBoxLayout()
        layout_result = QGridLayout()

        # layout_control
        self.setup_control_layout(layout_control)

        # layout_setup
        self.setup_setup_layout(layout_setup)

        # layout_result
        self.setup_result_layout(layout_result)

        layout_upper.addLayout(layout_control, 1)
        layout_upper.addLayout(layout_setup, 1)
        layout_upper.addLayout(layout_result, 1)

        # layout_lower: figure
        self.setup_figure(layout_lower)

        layout_main.addLayout(layout_upper, 2)
        layout_main.addLayout(layout_lower, 1)

        widget = QWidget()
        widget.setLayout(layout_main)

        self.set_styles()

        # 7 inch LCD display (800 x 480)
        self.setGeometry(0, 80, 800, 420)
        # self.showMaximized()
        self.setCentralWidget(widget)

    def setup_control_layout(self, layout):
        self.start_btn = self.create_button("Start", True, False, self.start, (120, 50))
        self.laser_btn = self.create_button("Laser", True, False, self.laser, (120, 50))
        self.punishment_btn = self.create_button("Punishment", False, False, self.punishment, (60, 50))
        self.reward_btn = self.create_button("Reward", False, False, self.reward, (60, 50))

        layout.addWidget(self.start_btn)
        layout.addWidget(self.laser_btn)
        layout_lower = QHBoxLayout()
        layout_lower.addWidget(self.punishment_btn) 
        layout_lower.addWidget(self.reward_btn)
        layout.addLayout(layout_lower)

    def setup_setup_layout(self, layout):
        self.mouse = self.create_spinbox(1, 100, 1, (120, 40))
        self.n_trial = self.create_spinbox(10, 1000, 200, (120, 40), 20)
        self.serial_port = QLineEdit("/dev/ttyACM0")
        self.serial_btn = self.create_button("Connect", True, True, self.serial_start, (120, 40))
        self.debug = QCheckBox('Debug mode')

        layout_setup = QFormLayout()
        layout_setup.addRow('Mouse num', self.mouse)
        layout_setup.addRow('n trial', self.n_trial)
        layout.addLayout(layout_setup)
        layout.addWidget(self.debug)
        layout.addWidget(self.serial_port)
        layout.addWidget(self.serial_btn)

    def setup_result_layout(self, layout):
        self.i_state = self.create_readonly_spinbox()
        self.i_trial = self.create_readonly_spinbox()
        self.i_correct = self.create_readonly_spinbox()
        self.i_reward = self.create_readonly_spinbox()
        self.text = self.create_text_edit()

        layout_left = QFormLayout()
        layout_left.addRow('i state', self.i_state)
        layout_left.addRow('i trial', self.i_trial)

        layout_right = QFormLayout()
        layout_right.addRow('i correct', self.i_correct)
        layout_right.addRow('i reward', self.i_reward)

        layout.addLayout(layout_left, 0, 0)
        layout.addLayout(layout_right, 0, 1)
        layout.addWidget(self.text, 1, 0, 1, 2)

    def setup_figure(self, layout):
        canvas = FigureCanvas(Figure())
        self.ax = canvas.figure.subplots()
        self.speed_idx = 0
        self.speed = np.zeros(500)
        self.t = np.linspace(0, 5, 500)
        self.plot, = self.ax.plot(self.t, self.speed, 'k')
        self.ax.set_xlim([0, 5])
        self.ax.set_ylim([0, 40])
        self.ax.set_ylabel('Speed')
        layout.addWidget(canvas)

    def create_button(self, text, checkable, enabled, connect_func, size):
        btn = QPushButton(text)
        btn.setCheckable(checkable)
        btn.setEnabled(enabled)
        btn.clicked.connect(connect_func)
        btn.setMinimumSize(*size)
        return btn

    def create_spinbox(self, min_val, max_val, value, size, step=1):
        spinbox = QSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(value)
        spinbox.setMinimumSize(*size)
        spinbox.setSingleStep(step)
        return spinbox

    def create_readonly_spinbox(self):
        spinbox = QSpinBox()
        spinbox.setReadOnly(True)
        return spinbox

    def create_text_edit(self):
        text_edit = QTextEdit()
        text_edit.setReadOnly(True)
        text_edit.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        text_edit.setFontPointSize(10)
        return text_edit

    def set_styles(self):
        self.setStyleSheet("""
            QSpinBox {
                qproperty-alignment: AlignCenter;
            }
            QLabel {
                qproperty-alignment: AlignCenter;
            }
            QLineEdit {
                qproperty-alignment: AlignCenter;
            }
            QPushButton {
                text-align: center;
            }
        """)

    def reset_plot(self):
        self.speed_idx = 0
        self.speed[:] = 0 # 5 seconds
        self.plot.set_ydata(self.speed)
        self.plot.figure.canvas.draw()

    def serial_start(self, checked):
        if checked:
            if 'COM' in self.serial_port.text() or 'tty' in self.serial_port.text():
                try:
                    self.serial = serial.Serial(port=self.serial_port.text(), baudrate=115200, timeout=0)
                    self.serial.flushInput()
                    self.serial.reset_output_buffer()

                    self.start_btn.setEnabled(True)
                    self.laser_btn.setEnabled(True)
                    self.punishment_btn.setEnabled(True)
                    self.reward_btn.setEnabled(True)
                    self.serial_btn.setText('Disconnect')
                    self.serial_btn.setStyleSheet("background-color: yellow")

                    self.reading_flag = True
                    self.reading_thread = WorkerThread(self.serial, self.debug.checkState())
                    self.reading_thread.dataReceived.connect(self.parse_data)
                    self.reading_thread.start()

                    print('Connected to {}'.format(self.serial_port.text()))

                except Exception as e:
                    print(e)
                    self.serial = None
                    print('Please enter the correct serial port name')
                    return
            else:
                self.serial = None
                print('Please enter the serial port name')
                return
        else:
            if self.serial is not None:
                try:
                    self.reading_flag = False
                    if self.reading_thread is not None:
                        self.reading_thread.stop()

                    self.serial.close()

                    self.start_btn.setEnabled(False)
                    self.punishment_btn.setEnabled(False)
                    self.reward_btn.setEnabled(False)
                    self.serial_btn.setText('Connect')
                    self.serial_btn.setStyleSheet("background-color: light gray")
                    print('Serial disconnected')
                
                except Exception as e:
                    print(e)
    
    def start(self, checked):
        if checked:
            # open file
            if not os.path.exists('./data'):
                os.mkdir('./data')
            self.fn = './data/A{:03d}_{}.txt'.format(self.mouse.value(), datetime.now().strftime("%Y%m%d_%H%M%S"))
            self.file = open(self.fn, "w")

            # plot
            self.reset_plot()

            # start task
            self.serial.write(("n" + str(self.n_trial.value())).encode())
            if self.debug.checkState():
                self.serial.write(b'd')
            else:
                self.serial.write(b'D')
            self.serial.write(b's')
            self.start_btn.setText('End')
            self.start_btn.setStyleSheet("background-color: green")

            self.i_trial.setValue(0)
            self.i_state.setValue(0)
            self.i_correct.setvalue(0)
            self.i_reward.setValue(0)
            self.laser_btn.setChecked(False)

        else:
            # end task
            self.serial.write(b'e')
            self.start_btn.setText('Start')
            self.start_btn.setStyleSheet("background-color: light gray")

            self.laser_btn.setChecked(True)

            # close file
            time.sleep(0.2)
            if self.file is not None:
                self.file.close()
                self.fn = None
                self.file = None

    def laser(self, checked):
        if checked:
            # open file
            if not os.path.exists('./data'):
                os.mkdir('./data')
            self.fn = './data/A{:03d}_{}_laser.txt'.format(self.mouse.value(), datetime.now().strftime("%Y%m%d_%H%M%S"))
            self.file = open(self.fn, "w")

            # start task
            self.serial.write(b'l')
            self.laser_btn.setText('Laser off')
            self.laser_btn.setStyleSheet("background-color: green")

            self.start_btn.setEnabled(False)
            self.text.append("Tagging started")
        else:
            self.serial.write(b'L')
            self.laser_btn.setText('Laser on')
            self.laser_btn.setStyleSheet("background-color: light gray")

            self.start_btn.setEnabled(True)
            self.text.append("Tagging finished (forced)")

            time.sleep(0.2)
            if self.file is not None:
                self.file.close()
                self.fn = None
                self.file = None
    
    def punishment(self):
        self.serial.write(b'p')
        
    def reward(self):
        self.serial.write(b'r')
        
    def closeEvent(self, e):
        print('Closing ...')
        if self.serial is not None:
            try:
                self.serial.write(b'f')
            except:
                pass

            if self.reading_thread is not None:
                self.reading_thread.stop()
            self.serial.close()

    def parse_data(self, data):
        cmd, *msgs = data.split(',')
        cmd = int(cmd)

        if self.file is not None:
            self.file.write(data + '\n')
            self.file.flush()

        # vr
        if (cmd >= 30 and cmd < 40):
            y = float(msgs[1]) / 0.082
            self.speed[self.speed_idx] = 0.1 * y + 0.9 * self.speed[self.speed_idx - 1]
            self.speed[self.speed_idx+1:self.speed_idx+9] = np.nan

            self.speed_idx += 1
            if self.speed_idx >= 500:
                self.speed_idx = 0
            
            if self.speed_idx % 5 == 0:
                self.plot.set_ydata(self.speed)
                self.plot.figure.canvas.draw()

        # sync
        elif (cmd >=40 and cmd < 50):
            pass
        
        # trial
        elif (cmd >= 60 and cmd < 70):
            t = float(msgs[0]) / 1e6
            i_state, i_trial = map(int, msgs[1:])

            self.i_trial.setValue(i_trial)
            self.i_state.setValue(i_state)

            msg = ''
            if i_state == 1:
                msg = "{:.1f}: {}, trial start".format(t, i_trial)
            elif i_state == 2:
                msg = "{:.1f}: {}, trial end".format(t, i_trial)
            elif i_state == 3:
                msg = "{:.1f}: {}, trial success".format(t, i_trial)
                self.i_correct.setValue(self.i_correct.value() + 1)
            elif i_state == 4 and i_trial > 0:
                msg = "{:.1f}: {}, trial fail".format(t, i_trial)

            if msg:
                self.text.append(msg)
        
        elif (cmd >= 70 and cmd < 80):
            if (cmd == 79):
                self.text.append("Tagging finished")

        elif (cmd >= 80 and cmd < 90):
            t = float(msgs[0]) / 1e6
            self.i_reward.setValue(self.i_reward.value() + 1)
            msg = "{:.1f}: reward".format(t)
            self.text.append(msg)

        elif (cmd == 99):
            msg = "Task finished"
            self.text.append(msg)

        else:
            self.text.append(msgs[0])


if __name__ == "__main__":
    app = QApplication(sys.argv)

    font = QFont()
    font.setPointSize(10)
    app.setFont(font)

    window = MainWindow()
    window.show()

    app.exec()