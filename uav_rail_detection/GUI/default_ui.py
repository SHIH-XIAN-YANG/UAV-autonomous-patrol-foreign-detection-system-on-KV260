# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GCS.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_GCS(object):
    # def __init__(self) -> None:
    #     self.server_IP = '140.113.148.64'
    #     self.connected:bool = False
    #     self.udp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     # Create a socket object and connect to the server
    #     self.tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     self.drone_info = {}


    def setupUi(self, GCS):
        GCS.setObjectName("GCS")
        GCS.resize(1080, 432)
        GCS.setMaximumSize(QtCore.QSize(1080, 720))
        self.video_frame = QtWidgets.QLabel(GCS)
        self.video_frame.setGeometry(QtCore.QRect(40, 30, 531, 311))
        self.video_frame.setMaximumSize(QtCore.QSize(640, 480))
        self.video_frame.setText("")
        self.video_frame.setObjectName("video_frame")
        self.verticalLayoutWidget = QtWidgets.QWidget(GCS)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(690, 20, 311, 381))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.groupBox_3 = QtWidgets.QGroupBox(self.verticalLayoutWidget)
        self.groupBox_3.setObjectName("groupBox_3")
        self.Port = QtWidgets.QLineEdit(self.groupBox_3)
        self.Port.setGeometry(QtCore.QRect(60, 60, 113, 22))
        self.Port.setObjectName("Port")
        self.Port_label = QtWidgets.QLabel(self.groupBox_3)
        self.Port_label.setGeometry(QtCore.QRect(20, 60, 58, 15))
        self.Port_label.setObjectName("Port_label")
        self.IP_label = QtWidgets.QLabel(self.groupBox_3)
        self.IP_label.setGeometry(QtCore.QRect(20, 30, 58, 15))
        self.IP_label.setObjectName("IP_label")
        self.IP = QtWidgets.QLineEdit(self.groupBox_3)
        self.IP.setGeometry(QtCore.QRect(60, 30, 113, 22))
        self.IP.setObjectName("IP")
        self.Connect_button = QtWidgets.QPushButton(self.groupBox_3)
        self.Connect_button.setGeometry(QtCore.QRect(200, 30, 61, 61))
        self.Connect_button.setObjectName("Connect_button")
        self.groupBox_2 = QtWidgets.QGroupBox(self.groupBox_3)
        self.groupBox_2.setGeometry(QtCore.QRect(0, 100, 271, 80))
        self.groupBox_2.setObjectName("groupBox_2")
        self.Anomaly_label = QtWidgets.QLabel(self.groupBox_2)
        self.Anomaly_label.setGeometry(QtCore.QRect(10, 30, 58, 15))
        self.Anomaly_label.setObjectName("Anomaly_label")
        self.Anomaly = QtWidgets.QLabel(self.groupBox_2)
        self.Anomaly.setGeometry(QtCore.QRect(90, 30, 58, 15))
        self.Anomaly.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.Anomaly.setObjectName("Anomaly")
        self.groupBox = QtWidgets.QGroupBox(self.groupBox_3)
        self.groupBox.setGeometry(QtCore.QRect(0, 190, 271, 151))
        self.groupBox.setObjectName("groupBox")
        self.GPS_lable = QtWidgets.QLabel(self.groupBox)
        self.GPS_lable.setGeometry(QtCore.QRect(10, 30, 58, 15))
        self.GPS_lable.setObjectName("GPS_lable")
        self.Yaw_label = QtWidgets.QLabel(self.groupBox)
        self.Yaw_label.setGeometry(QtCore.QRect(10, 50, 58, 15))
        self.Yaw_label.setObjectName("Yaw_label")
        self.Vel_label = QtWidgets.QLabel(self.groupBox)
        self.Vel_label.setGeometry(QtCore.QRect(10, 70, 58, 15))
        self.Vel_label.setObjectName("Vel_label")
        self.GPS = QtWidgets.QLabel(self.groupBox)
        self.GPS.setGeometry(QtCore.QRect(70, 30, 58, 15))
        self.GPS.setObjectName("GPS")
        self.Yaw = QtWidgets.QLabel(self.groupBox)
        self.Yaw.setGeometry(QtCore.QRect(70, 50, 58, 15))
        self.Yaw.setObjectName("Yaw")
        self.Vel = QtWidgets.QLabel(self.groupBox)
        self.Vel.setGeometry(QtCore.QRect(70, 70, 58, 15))
        self.Vel.setObjectName("Vel")
        self.label = QtWidgets.QLabel(self.groupBox)
        self.label.setGeometry(QtCore.QRect(10, 90, 58, 15))
        self.label.setObjectName("label")
        self.label_3 = QtWidgets.QLabel(self.groupBox)
        self.label_3.setGeometry(QtCore.QRect(70, 90, 58, 15))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.groupBox)
        self.label_4.setGeometry(QtCore.QRect(10, 110, 71, 16))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.groupBox)
        self.label_5.setGeometry(QtCore.QRect(80, 110, 58, 15))
        self.label_5.setObjectName("label_5")
        self.verticalLayout.addWidget(self.groupBox_3)

        
        # self.Connect_button.clicked.connect(self.connect_to_drone)


        self.retranslateUi(GCS)
        QtCore.QMetaObject.connectSlotsByName(GCS)

    


    def retranslateUi(self, GCS):
        _translate = QtCore.QCoreApplication.translate
        GCS.setWindowTitle(_translate("GCS", "Dialog"))
        self.groupBox_3.setTitle(_translate("GCS", "Drone connection"))
        self.Port.setText(_translate("GCS", "8888"))
        self.Port_label.setText(_translate("GCS", "Port:"))
        self.IP_label.setText(_translate("GCS", "IP:"))
        self.IP.setText(_translate("GCS", "140.113.148.64"))
        self.Connect_button.setText(_translate("GCS", "Connect"))
        self.groupBox_2.setTitle(_translate("GCS", "Anomaly"))
        self.Anomaly_label.setText(_translate("GCS", "Anomaly"))
        self.Anomaly.setText(_translate("GCS", " -"))
        self.groupBox.setTitle(_translate("GCS", "Drone state"))
        self.GPS_lable.setText(_translate("GCS", "GPS"))
        self.Yaw_label.setText(_translate("GCS", "Yaw"))
        self.Vel_label.setText(_translate("GCS", "Velocity"))
        self.GPS.setText(_translate("GCS", "-"))
        self.Yaw.setText(_translate("GCS", "-"))
        self.Vel.setText(_translate("GCS", "-"))
        self.label.setText(_translate("GCS", "Altitude"))
        self.label_3.setText(_translate("GCS", "-"))
        self.label_4.setText(_translate("GCS", "drone state"))
        self.label_5.setText(_translate("GCS", "-"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    GCS = QtWidgets.QDialog()
    ui = Ui_GCS()
    ui.setupUi(GCS)
    GCS.show()
    sys.exit(app.exec_())
