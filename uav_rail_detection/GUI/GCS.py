# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GCS.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_GCS(object):
    def setupUi(self, GCS):
        GCS.setObjectName("GCS")
        GCS.setEnabled(True)
        GCS.resize(965, 663)
        GCS.setMaximumSize(QtCore.QSize(2000, 1500))
        GCS.setWindowOpacity(1.0)
        self.video_frame = QtWidgets.QLabel(GCS)
        self.video_frame.setGeometry(QtCore.QRect(70, 50, 480, 360))
        self.video_frame.setMaximumSize(QtCore.QSize(640, 480))
        font = QtGui.QFont()
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.video_frame.setFont(font)
        self.video_frame.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.video_frame.setText("")
        self.video_frame.setPixmap(QtGui.QPixmap("UI/icon.png"))
        self.video_frame.setScaledContents(False)
        self.video_frame.setAlignment(QtCore.Qt.AlignCenter)
        self.video_frame.setObjectName("video_frame")
        self.Drone_state_groupBox = QtWidgets.QGroupBox(GCS)
        self.Drone_state_groupBox.setGeometry(QtCore.QRect(10, 420, 941, 231))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(14)
        self.Drone_state_groupBox.setFont(font)
        self.Drone_state_groupBox.setObjectName("Drone_state_groupBox")
        self.Location_lable = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.Location_lable.setGeometry(QtCore.QRect(10, 30, 151, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setBold(False)
        font.setWeight(50)
        self.Location_lable.setFont(font)
        self.Location_lable.setObjectName("Location_lable")
        self.Altitude_label = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.Altitude_label.setGeometry(QtCore.QRect(10, 70, 161, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        self.Altitude_label.setFont(font)
        self.Altitude_label.setObjectName("Altitude_label")
        self.Location = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.Location.setEnabled(True)
        self.Location.setGeometry(QtCore.QRect(200, 30, 721, 31))
        self.Location.setObjectName("Location")
        self.Altitude = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.Altitude.setGeometry(QtCore.QRect(200, 70, 741, 31))
        self.Altitude.setObjectName("Altitude")
        self.Attitude_label = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.Attitude_label.setGeometry(QtCore.QRect(10, 150, 141, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        self.Attitude_label.setFont(font)
        self.Attitude_label.setObjectName("Attitude_label")
        self.Attitude = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.Attitude.setGeometry(QtCore.QRect(200, 150, 731, 31))
        self.Attitude.setObjectName("Attitude")
        self.VehiclaMode_label = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.VehiclaMode_label.setGeometry(QtCore.QRect(10, 190, 141, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        self.VehiclaMode_label.setFont(font)
        self.VehiclaMode_label.setObjectName("VehiclaMode_label")
        self.VehiclaMode = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.VehiclaMode.setGeometry(QtCore.QRect(200, 190, 751, 31))
        self.VehiclaMode.setObjectName("VehiclaMode")
        self.Battery_label = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.Battery_label.setGeometry(QtCore.QRect(10, 110, 151, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        self.Battery_label.setFont(font)
        self.Battery_label.setObjectName("Battery_label")
        self.Battery = QtWidgets.QLabel(self.Drone_state_groupBox)
        self.Battery.setGeometry(QtCore.QRect(200, 110, 741, 31))
        self.Battery.setObjectName("Battery")
        self.groupBox_3 = QtWidgets.QGroupBox(GCS)
        self.groupBox_3.setGeometry(QtCore.QRect(610, 40, 319, 381))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(12)
        self.groupBox_3.setFont(font)
        self.groupBox_3.setObjectName("groupBox_3")
        self.Port = QtWidgets.QLineEdit(self.groupBox_3)
        self.Port.setGeometry(QtCore.QRect(60, 60, 113, 22))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.Port.setFont(font)
        self.Port.setObjectName("Port")
        self.Port_label = QtWidgets.QLabel(self.groupBox_3)
        self.Port_label.setGeometry(QtCore.QRect(20, 60, 58, 15))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(12)
        self.Port_label.setFont(font)
        self.Port_label.setObjectName("Port_label")
        self.IP_label = QtWidgets.QLabel(self.groupBox_3)
        self.IP_label.setGeometry(QtCore.QRect(20, 30, 58, 15))
        self.IP_label.setObjectName("IP_label")
        self.IP = QtWidgets.QLineEdit(self.groupBox_3)
        self.IP.setGeometry(QtCore.QRect(60, 30, 113, 22))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.IP.setFont(font)
        self.IP.setObjectName("IP")
        self.Connect_button = QtWidgets.QPushButton(self.groupBox_3)
        self.Connect_button.setGeometry(QtCore.QRect(190, 30, 121, 61))
        self.Connect_button.setObjectName("Connect_button")
        self.groupBox_2 = QtWidgets.QGroupBox(self.groupBox_3)
        self.groupBox_2.setGeometry(QtCore.QRect(20, 100, 271, 80))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        self.groupBox_2.setFont(font)
        self.groupBox_2.setObjectName("groupBox_2")
        self.Anomaly = QtWidgets.QLabel(self.groupBox_2)
        self.Anomaly.setGeometry(QtCore.QRect(10, 30, 251, 41))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.Anomaly.setFont(font)
        self.Anomaly.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.Anomaly.setObjectName("Anomaly")
        self.open_cam_button = QtWidgets.QPushButton(self.groupBox_3)
        self.open_cam_button.setEnabled(False)
        self.open_cam_button.setGeometry(QtCore.QRect(20, 190, 271, 51))
        self.open_cam_button.setObjectName("open_cam_button")
        self.open_folder_button = QtWidgets.QPushButton(self.groupBox_3)
        self.open_folder_button.setEnabled(True)
        self.open_folder_button.setGeometry(QtCore.QRect(20, 250, 271, 51))
        self.open_folder_button.setObjectName("open_folder_button")

        self.retranslateUi(GCS)
        QtCore.QMetaObject.connectSlotsByName(GCS)

    def retranslateUi(self, GCS):
        _translate = QtCore.QCoreApplication.translate
        GCS.setWindowTitle(_translate("GCS", "Ground Control monitor"))
        self.Drone_state_groupBox.setTitle(_translate("GCS", "Drone state"))
        self.Location_lable.setText(_translate("GCS", "Location"))
        self.Altitude_label.setText(_translate("GCS", "Altitude"))
        self.Location.setText(_translate("GCS", "-"))
        self.Altitude.setText(_translate("GCS", "-"))
        self.Attitude_label.setText(_translate("GCS", "Atttitude"))
        self.Attitude.setText(_translate("GCS", "-"))
        self.VehiclaMode_label.setText(_translate("GCS", "VehiclaMode"))
        self.VehiclaMode.setText(_translate("GCS", "-"))
        self.Battery_label.setText(_translate("GCS", "Battery"))
        self.Battery.setText(_translate("GCS", "-"))
        self.groupBox_3.setTitle(_translate("GCS", "Drone connection"))
        self.Port.setText(_translate("GCS", "8888"))
        self.Port_label.setText(_translate("GCS", "Port:"))
        self.IP_label.setText(_translate("GCS", "IP:"))
        self.IP.setText(_translate("GCS", "192.168.50.25"))
        self.Connect_button.setText(_translate("GCS", "Connect"))
        self.groupBox_2.setTitle(_translate("GCS", "Anomaly"))
        self.Anomaly.setText(_translate("GCS", " -"))
        self.open_cam_button.setText(_translate("GCS", "open camera"))
        self.open_folder_button.setText(_translate("GCS", "open folder"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    GCS = QtWidgets.QDialog()
    ui = Ui_GCS()
    ui.setupUi(GCS)
    GCS.show()
    sys.exit(app.exec_())
