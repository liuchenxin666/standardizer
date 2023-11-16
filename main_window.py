# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main_window.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
import pic_rc
import time
import glob, os
import datetime
from BME285 import BME285_Class


class UI_MAIN(QtWidgets.QMainWindow):
    def __init__(self, ADC):
        super(UI_MAIN, self).__init__()
        self.DATA = {} # 存放当前页面的数据信息
        self.path = './outputs' # 实验结果保存路径
        self.Hardware_BME285 = BME285_Class()  # 温湿度硬件
        self.ADC = ADC # 压力传感器
        self.setupUi(self)

    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(800, 400)
        self.tabWidget = QtWidgets.QTabWidget(Form)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 800, 400))
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.formLayoutWidget = QtWidgets.QWidget(self.tab)
        self.formLayoutWidget.setGeometry(QtCore.QRect(60, 140, 251, 221))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setContentsMargins(8, 8, 0, 0)
        self.formLayout.setHorizontalSpacing(8)
        self.formLayout.setVerticalSpacing(10)
        self.formLayout.setObjectName("formLayout")
        self.Label_unit = QtWidgets.QLabel(self.formLayoutWidget)
        self.Label_unit.setObjectName("Label_unit")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.Label_unit)
        self.LineEdit_unit = QtWidgets.QLineEdit(self.formLayoutWidget)
        self.LineEdit_unit.setObjectName("LineEdit_unit")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.LineEdit_unit)
        self.Label_title = QtWidgets.QLabel(self.formLayoutWidget)
        self.Label_title.setObjectName("Label_title")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.Label_title)
        self.LineEdit_title = QtWidgets.QLineEdit(self.formLayoutWidget)
        self.LineEdit_title.setObjectName("LineEdit_title")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.LineEdit_title)
        self.Label_type = QtWidgets.QLabel(self.formLayoutWidget)
        self.Label_type.setObjectName("Label_type")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.Label_type)
        self.LineEdit_type = QtWidgets.QLineEdit(self.formLayoutWidget)
        self.LineEdit_type.setObjectName("LineEdit_type")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.LineEdit_type)
        self.Label_manufacturer = QtWidgets.QLabel(self.formLayoutWidget)
        self.Label_manufacturer.setObjectName("Label_manufacturer")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.Label_manufacturer)
        self.LineEdit_manufacturer = QtWidgets.QLineEdit(self.formLayoutWidget)
        self.LineEdit_manufacturer.setObjectName("LineEdit_manufacturer")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.LineEdit_manufacturer)
        self.Label_fac_label = QtWidgets.QLabel(self.formLayoutWidget)
        self.Label_fac_label.setObjectName("Label_fac_label")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.Label_fac_label)
        self.LineEdit_fac_label = QtWidgets.QLineEdit(self.formLayoutWidget)
        self.LineEdit_fac_label.setObjectName("LineEdit_fac_label")
        self.formLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.LineEdit_fac_label)
        self.Label_channel = QtWidgets.QLabel(self.formLayoutWidget)
        self.Label_channel.setObjectName("Label_channel")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.LabelRole, self.Label_channel)
        self.ComboBox_channel = QtWidgets.QComboBox(self.formLayoutWidget)
        self.ComboBox_channel.setObjectName("ComboBox_channel")
        self.ComboBox_channel.addItem("")
        self.ComboBox_channel.addItem("")
        self.ComboBox_channel.addItem("")
        self.formLayout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.ComboBox_channel)
        self.Label_automation = QtWidgets.QLabel(self.formLayoutWidget)
        self.Label_automation.setObjectName("Label_automation")
        self.formLayout.setWidget(6, QtWidgets.QFormLayout.LabelRole, self.Label_automation)
        self.ComboBox_automation = QtWidgets.QComboBox(self.formLayoutWidget)
        self.ComboBox_automation.setObjectName("ComboBox_automation")
        self.ComboBox_automation.addItem("")
        self.ComboBox_automation.addItem("")
        self.formLayout.setWidget(6, QtWidgets.QFormLayout.FieldRole, self.ComboBox_automation)
        self.formLayoutWidget_2 = QtWidgets.QWidget(self.tab)
        self.formLayoutWidget_2.setGeometry(QtCore.QRect(480, 5, 251, 311))
        self.formLayoutWidget_2.setObjectName("formLayoutWidget_2")
        self.formLayout_2 = QtWidgets.QFormLayout(self.formLayoutWidget_2)
        self.formLayout_2.setContentsMargins(8, 8, 0, 0)
        self.formLayout_2.setHorizontalSpacing(8)
        self.formLayout_2.setVerticalSpacing(10)
        self.formLayout_2.setObjectName("formLayout_2")
        self.Label_name = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_name.setObjectName("Label_name")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.Label_name)
        self.LineEdit_name = QtWidgets.QLineEdit(self.formLayoutWidget_2)
        self.LineEdit_name.setObjectName("LineEdit_name")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.LineEdit_name)
        self.Label_calibration = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_calibration.setObjectName("Label_calibration")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.Label_calibration)
        self.ComboBox_calibration = QtWidgets.QComboBox(self.formLayoutWidget_2)
        self.ComboBox_calibration.setObjectName("ComboBox_calibration")
        self.ComboBox_calibration.addItem("")
        self.ComboBox_calibration.addItem("")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.ComboBox_calibration)
        self.Label_temperature = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_temperature.setObjectName("Label_temperature")
        self.formLayout_2.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.Label_temperature)
        self.Label_humidity = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_humidity.setObjectName("Label_humidity")
        self.formLayout_2.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.Label_humidity)
        self.Label_number = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_number.setObjectName("Label_number")
        self.formLayout_2.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.Label_number)
        self.LineEdit_number = QtWidgets.QLineEdit(self.formLayoutWidget_2)
        self.LineEdit_number.setObjectName("LineEdit_number")
        self.formLayout_2.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.LineEdit_number)
        self.Label_moment = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_moment.setObjectName("Label_moment")
        self.formLayout_2.setWidget(5, QtWidgets.QFormLayout.LabelRole, self.Label_moment)
        self.Label_location = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_location.setObjectName("Label_location")
        self.formLayout_2.setWidget(6, QtWidgets.QFormLayout.LabelRole, self.Label_location)
        self.LineEdit_location = QtWidgets.QLineEdit(self.formLayoutWidget_2)
        self.LineEdit_location.setObjectName("LineEdit_location")
        self.formLayout_2.setWidget(6, QtWidgets.QFormLayout.FieldRole, self.LineEdit_location)
        self.Label_norm = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_norm.setObjectName("Label_norm")
        self.formLayout_2.setWidget(7, QtWidgets.QFormLayout.LabelRole, self.Label_norm)
        self.Label_machine = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_machine.setObjectName("Label_machine")
        self.formLayout_2.setWidget(8, QtWidgets.QFormLayout.LabelRole, self.Label_machine)
        self.Label_17 = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.Label_17.setObjectName("Label_17")
        self.formLayout_2.setWidget(9, QtWidgets.QFormLayout.LabelRole, self.Label_17)
        self.comboBox = QtWidgets.QComboBox(self.formLayoutWidget_2)
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.formLayout_2.setWidget(7, QtWidgets.QFormLayout.FieldRole, self.comboBox)
        self.label_form = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_form.setMinimumSize(QtCore.QSize(0, 20))
        self.label_form.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_form.setObjectName("label_form")
        self.formLayout_2.setWidget(8, QtWidgets.QFormLayout.FieldRole, self.label_form)
        self.label_form_num = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_form_num.setMinimumSize(QtCore.QSize(0, 20))
        self.label_form_num.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_form_num.setText("")
        self.label_form_num.setObjectName("label_form_num")
        self.formLayout_2.setWidget(9, QtWidgets.QFormLayout.FieldRole, self.label_form_num)
        self.label_tem = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_tem.setMinimumSize(QtCore.QSize(0, 20))
        self.label_tem.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_tem.setText("")
        self.label_tem.setObjectName("label_tem")
        self.formLayout_2.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.label_tem)
        self.label_hum = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_hum.setMinimumSize(QtCore.QSize(0, 20))
        self.label_hum.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_hum.setText("")
        self.label_hum.setObjectName("label_hum")
        self.formLayout_2.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.label_hum)
        self.label_time = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_time.setMinimumSize(QtCore.QSize(0, 20))
        self.label_time.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_time.setText("")
        self.label_time.setObjectName("label_time")
        self.formLayout_2.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.label_time)
        self.pushButton_1_over = QtWidgets.QPushButton(self.tab)
        self.pushButton_1_over.setGeometry(QtCore.QRect(570, 330, 75, 30))
        self.pushButton_1_over.setObjectName("pushButton_1_over")
        self.textBrowser_1_remind = QtWidgets.QTextBrowser(self.tab)
        self.textBrowser_1_remind.setGeometry(QtCore.QRect(10, 0, 421, 31))
        self.textBrowser_1_remind.setStyleSheet("background-color:yellow")
        self.textBrowser_1_remind.setObjectName("textBrowser_1_remind")
        self.frame_3 = QtWidgets.QFrame(self.tab)
        self.frame_3.setGeometry(QtCore.QRect(50, 36, 271, 91))
        self.frame_3.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setLineWidth(1)
        self.frame_3.setMidLineWidth(0)
        self.frame_3.setObjectName("frame_3")
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.tab)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(60, 40, 251, 61))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_1_p_max = QtWidgets.QLabel(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_1_p_max.sizePolicy().hasHeightForWidth())
        self.label_1_p_max.setSizePolicy(sizePolicy)
        self.label_1_p_max.setObjectName("label_1_p_max")
        self.gridLayout_2.addWidget(self.label_1_p_max, 0, 0, 1, 1)
        self.label_1_p_min = QtWidgets.QLabel(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_1_p_min.sizePolicy().hasHeightForWidth())
        self.label_1_p_min.setSizePolicy(sizePolicy)
        self.label_1_p_min.setObjectName("label_1_p_min")
        self.gridLayout_2.addWidget(self.label_1_p_min, 1, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.label_3.setText("")
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 0, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.label_4.setText("")
        self.label_4.setObjectName("label_4")
        self.gridLayout_2.addWidget(self.label_4, 1, 1, 1, 1)
        self.frame_4 = QtWidgets.QFrame(self.tab)
        self.frame_4.setGeometry(QtCore.QRect(50, 135, 270, 231))
        self.frame_4.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setLineWidth(1)
        self.frame_4.setMidLineWidth(0)
        self.frame_4.setObjectName("frame_4")
        self.frame_5 = QtWidgets.QFrame(self.tab)
        self.frame_5.setGeometry(QtCore.QRect(470, 0, 270, 320))
        self.frame_5.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setLineWidth(1)
        self.frame_5.setMidLineWidth(0)
        self.frame_5.setObjectName("frame_5")
        self.pushButton_8 = QtWidgets.QPushButton(self.tab)
        self.pushButton_8.setGeometry(QtCore.QRect(330, 60, 75, 30))
        self.pushButton_8.setObjectName("pushButton_8")
        self.radioButton_1_right = QtWidgets.QRadioButton(self.tab)
        self.radioButton_1_right.setGeometry(QtCore.QRect(60, 110, 90, 16))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.radioButton_1_right.sizePolicy().hasHeightForWidth())
        self.radioButton_1_right.setSizePolicy(sizePolicy)
        self.radioButton_1_right.setObjectName("radioButton_1_right")
        self.radioButton_1_wrong = QtWidgets.QRadioButton(self.tab)
        self.radioButton_1_wrong.setGeometry(QtCore.QRect(130, 110, 90, 16))
        self.radioButton_1_wrong.setObjectName("radioButton_1_wrong")
        self.frame_5.raise_()
        self.frame_4.raise_()
        self.frame_3.raise_()
        self.formLayoutWidget.raise_()
        self.formLayoutWidget_2.raise_()
        self.pushButton_1_over.raise_()
        self.textBrowser_1_remind.raise_()
        self.gridLayoutWidget_2.raise_()
        self.pushButton_8.raise_()
        self.radioButton_1_right.raise_()
        self.radioButton_1_wrong.raise_()
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.pushButton_2_outer = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_2_outer.setGeometry(QtCore.QRect(60, 20, 120, 50))
        self.pushButton_2_outer.setObjectName("pushButton_2_outer")
        self.pushButton_2_normal = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_2_normal.setGeometry(QtCore.QRect(240, 20, 130, 50))
        self.pushButton_2_normal.setObjectName("pushButton_2_normal")
        self.pushButton_2_anti = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_2_anti.setGeometry(QtCore.QRect(240, 120, 130, 50))
        self.pushButton_2_anti.setObjectName("pushButton_2_anti")
        self.pushButton_2_hemostasis = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_2_hemostasis.setGeometry(QtCore.QRect(60, 120, 120, 50))
        self.pushButton_2_hemostasis.setObjectName("pushButton_2_hemostasis")
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.pushButton = QtWidgets.QPushButton(self.tab_3)
        self.pushButton.setGeometry(QtCore.QRect(59, 30, 140, 50))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.tab_3)
        self.pushButton_2.setGeometry(QtCore.QRect(250, 130, 180, 50))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_5 = QtWidgets.QPushButton(self.tab_3)
        self.pushButton_5.setGeometry(QtCore.QRect(250, 30, 180, 50))
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_4 = QtWidgets.QPushButton(self.tab_3)
        self.pushButton_4.setGeometry(QtCore.QRect(59, 130, 140, 50))
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_3 = QtWidgets.QPushButton(self.tab_3)
        self.pushButton_3.setGeometry(QtCore.QRect(140, 230, 180, 50))
        self.pushButton_3.setObjectName("pushButton_3")
        self.tabWidget.addTab(self.tab_3, "")
        self.tab_4 = QtWidgets.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.textEdit = QtWidgets.QTextEdit(self.tab_4)
        self.textEdit.setGeometry(QtCore.QRect(409, 19, 380, 351))
        self.textEdit.setObjectName("textEdit")
        self.gridLayoutWidget = QtWidgets.QWidget(self.tab_4)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(0, 30, 371, 61))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.label = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label.setMinimumSize(QtCore.QSize(0, 40))
        self.label.setMaximumSize(QtCore.QSize(16777215, 40))
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget)
        self.lineEdit.setMinimumSize(QtCore.QSize(0, 30))
        self.lineEdit.setObjectName("lineEdit")
        self.gridLayout.addWidget(self.lineEdit, 0, 1, 1, 1)
        self.pushButton_6 = QtWidgets.QPushButton(self.tab_4)
        self.pushButton_6.setGeometry(QtCore.QRect(0, 0, 150, 23))
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_7 = QtWidgets.QPushButton(self.tab_4)
        self.pushButton_7.setGeometry(QtCore.QRect(260, 0, 110, 23))
        self.pushButton_7.setObjectName("pushButton_7")
        self.listWidget = QtWidgets.QListWidget(self.tab_4)
        self.listWidget.setGeometry(QtCore.QRect(0, 100, 370, 271))
        self.listWidget.setObjectName("listWidget")
        self.label_2 = QtWidgets.QLabel(self.tab_4)
        self.label_2.setGeometry(QtCore.QRect(413, 0, 70, 20))
        self.label_2.setObjectName("label_2")
        self.tabWidget.addTab(self.tab_4, "")

        self.retranslateUi(Form)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle("主界面")
        self.Label_unit.setText(_translate("Form", "单位"))
        self.Label_title.setText(_translate("Form", "名称"))
        self.Label_type.setText(_translate("Form", "型号"))
        self.Label_manufacturer.setText(_translate("Form", "生产厂家"))
        self.Label_fac_label.setText(_translate("Form", "出厂标号"))
        self.Label_channel.setText(_translate("Form", "通道数"))
        self.ComboBox_channel.setItemText(0, _translate("Form", "单通道"))
        self.ComboBox_channel.setItemText(1, _translate("Form", "双通道"))
        self.ComboBox_channel.setItemText(2, _translate("Form", "三通道"))
        self.Label_automation.setText(_translate("Form", "自动化"))
        self.ComboBox_automation.setItemText(0, _translate("Form", "半自动"))
        self.ComboBox_automation.setItemText(1, _translate("Form", "全自动"))
        self.Label_name.setText(_translate("Form", "姓名"))
        self.Label_calibration.setText(_translate("Form", "校准/核验"))
        self.ComboBox_calibration.setItemText(0, _translate("Form", "校准"))
        self.ComboBox_calibration.setItemText(1, _translate("Form", "核验"))
        self.Label_temperature.setText(_translate("Form", "环境温度"))
        self.Label_humidity.setText(_translate("Form", "环境湿度"))
        self.Label_number.setText(_translate("Form", "证书编号"))
        self.Label_moment.setText(_translate("Form", "检测时间"))
        self.Label_location.setText(_translate("Form", "检测地点"))
        self.Label_norm.setText(_translate("Form", "依据规范"))
        self.Label_machine.setText(_translate("Form", "标准器"))
        self.Label_17.setText(_translate("Form", "证书号"))
        self.comboBox.setItemText(0, _translate("Form", "1"))
        self.comboBox.setItemText(1, _translate("Form", "2"))
        self.comboBox.setItemText(2, _translate("Form", "3"))
        self.comboBox.setItemText(3, _translate("Form", "4"))
        self.comboBox.setItemText(4, _translate("Form", "5"))
        self.comboBox.setItemText(5, _translate("Form", "6"))
        self.comboBox.setItemText(6, _translate("Form", "7"))
        self.comboBox.setItemText(7, _translate("Form", "8"))
        self.comboBox.setItemText(8, _translate("Form", "9"))
        self.label_form.setText(_translate("Form", "压力标准器"))
        self.pushButton_1_over.setText(_translate("Form", "确认"))
        self.textBrowser_1_remind.setHtml(_translate("Form",
                                                     "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                                     "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                                     "p, li { white-space: pre-wrap; }\n"
                                                     "</style></head><body style=\" font-family:\'SimSun\'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
                                                     "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">开机自检，包括指示灯、液晶显示器及标准器，并测一段时间压强。</span></p></body></html>"))
        self.label_1_p_max.setText(_translate("Form", "压强最大值"))
        self.label_1_p_min.setText(_translate("Form", "压强最小值"))
        self.pushButton_8.setText(_translate("Form", "自检"))
        self.radioButton_1_right.setText(_translate("Form", "正常"))
        self.radioButton_1_wrong.setText(_translate("Form", "不正常"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("Form", "系统主界面"))
        self.pushButton_2_outer.setText(_translate("Form", "外观检查"))
        self.pushButton_2_normal.setText(_translate("Form", "一般工作\n"
                                                            "正常性检查"))
        self.pushButton_2_anti.setText(_translate("Form", "压力控制\n"
                                                          "抗干扰能力检查"))
        self.pushButton_2_hemostasis.setText(_translate("Form", "止血时间检查"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("Form", "校准前检查界面"))
        self.pushButton.setText(_translate("Form", "加压上升时间检测"))
        self.pushButton_2.setText(_translate("Form", "袖带压力示值误差检测"))
        self.pushButton_5.setText(_translate("Form", "压力设置值与袖带\n"
                                                     "压力示值的一致性检测"))
        self.pushButton_4.setText(_translate("Form", "气密性检测"))
        self.pushButton_3.setText(_translate("Form", "压力波动度检测"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("Form", "校准检测界面"))
        self.label.setText(_translate("Form", "输入查询关键词\n"
                                              "(多个关键词用空格断开)"))
        self.pushButton_6.setText(_translate("Form", "保存本次实验结果"))
        self.pushButton_7.setText(_translate("Form", "查询"))
        self.label_2.setText(_translate("Form", "内容显示"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), _translate("Form", "数据查询及通讯界面"))

        # 系统主界面 不对应，不知道咋写
        self.pushButton_1_over.clicked.connect(self.over_1)
        self.pushButton_8.clicked.connect(self.check_self)

        self.Hardware_BME285.readBME280All()  # 温湿度、气压 硬件输入
        self.label_tem.setText(str(format(self.Hardware_BME285.temperature, '.1f')))
        self.label_hum.setText(str(format(self.Hardware_BME285.humidity, '.1f')))
        self.label_time.setText(str(datetime.datetime.today().date()))

        # 第四个界面
        self.pushButton_6.clicked.connect(self.save_all)
        self.pushButton_7.clicked.connect(self.search_file)
        self.listWidget.doubleClicked.connect(self.display_file)

        # 显示
        for file in glob.glob(self.path + r"/*.txt"):
            name = file.strip().split('/')[-1]
            self.listWidget.addItem(name)

        # 创建一个临时文件
        if os.path.exists('./tmp/main.txt'):
            os.remove('./tmp/main.txt')
        file = open('./tmp/main.txt', 'w')
        file.close()

    def over_1(self):
        content = []
        if self.radioButton_1_right.isChecked():
            self.DATA['压强'] = '正常'
        elif self.radioButton_1_wrong.isChecked():
            self.DATA['压强'] = '不正常'
        else:
            QMessageBox.information(self, 'info', '请勾选压强是否正常')
            return
        if self.LineEdit_unit.text() == '':
            QMessageBox.information(self, 'info', '请填写被校单位')
            return
        else:
            self.DATA['被校单位'] = self.LineEdit_unit.text()
            content.append("被校单位: " + self.LineEdit_unit.text() + '\n')
        if self.LineEdit_title.text() == '':
            QMessageBox.information(self, 'info', '请填写被校仪器名称')
            return
        else:
            self.DATA['被校仪器名称'] = self.LineEdit_title.text()
            content.append("被校仪器名称: " + self.LineEdit_title.text() + '\n')
        self.DATA['通道数'] = self.ComboBox_channel.currentText()
        self.DATA['自动化'] = self.ComboBox_automation.currentText()
        content.append(self.ComboBox_channel.currentText() + ',' + self.ComboBox_automation.currentText() + '\n')
        if self.LineEdit_type.text() == '':
            QMessageBox.information(self, 'info', '请填写型号')
            return
        else:
            self.DATA['型号'] = self.LineEdit_type.text()
            content.append("型号: " + self.LineEdit_type.text() + '\n')
        if self.LineEdit_manufacturer.text() == '':
            QMessageBox.information(self, 'info', '请填写生产厂家')
            return
        else:
            self.DATA['生产厂家'] = self.LineEdit_manufacturer.text()
            content.append("生产厂家: " + self.LineEdit_manufacturer.text() + '\n')
        if self.LineEdit_fac_label.text() == '':
            QMessageBox.information(self, 'info', '请填写出厂编号')
            return
        else:
            self.DATA['出厂编号'] = self.LineEdit_fac_label.text()
            content.append("出厂编号: " + self.LineEdit_fac_label.text() + '\n')

        self.label_form_num.setText(self.label_time.text() + '_' + self.DATA['被校单位'] + '_'
                                    + self.DATA['被校仪器名称'] + '_' + self.DATA['型号'])
        content.append("压力标准器: \n")
        content.append("    名称: " + self.label_form.text() + '\n')
        content.append("    证书号: " + self.label_form_num.text() + '\n')

        if self.ComboBox_calibration.currentText() == '校准':
            content.append("模式: 校准\n")
        else:
            content.append("模式: 核验\n")

        content.append("环境温度: " + self.label_tem.text() + '\n')
        content.append("环境湿度: " + self.label_hum.text() + '\n')

        if self.LineEdit_number.text() == '':
            QMessageBox.information(self, 'info', '请填写证书编号')
            return
        else:
            self.DATA['证书编号'] = self.LineEdit_number.text()
            content.append("证书编号: " + self.LineEdit_number.text() + '\n')

        content.append("检测时间: " + self.label_time.text() + '\n')
        if self.LineEdit_location.text() == '':
            QMessageBox.information(self, 'info', '请填写检测地点')
            return
        else:
            self.DATA['检测地点'] = self.LineEdit_location.text()
            content.append("检测地点: " + self.LineEdit_location.text() + '\n')

        content.append("所依据的规范: " + self.comboBox.currentText() + '\n')

        # 保存信息
        with open('./tmp/main.txt', 'w', encoding='utf-8') as writers:
            writers.writelines(content)

    def check_self(self):
        t = 1
        pmax = -10000
        pmin = 10000
        while t <= 10:
            time.sleep(0.3)
            v = self.ADC.ADS1256_GetChannalValue(6) * 500 / 0x7fffff  # 高精度传感器
            pmax = max(pmax, v)
            pmin = min(pmin, v)
            t += 1

        self.label_3.setText(f'{pmax:.2f}' + "kPa")
        self.label_4.setText(f'{pmin:.2f}' + "kPa")

    def save_all(self):
        # 用证书编号命名
        with open('./outputs/' + self.label_time.text() + '.txt', 'w', encoding='utf-8') as writers:
            file = open('./tmp/main.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请填写系统主界面的详细信息')
            #     return
            writers.write(data)
            writers.write('\n\n')
            file.close()

            file = open('./tmp/a.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请进行外观检查')
            #     return
            writers.writelines(['1 校准前检查\n', '  1.1 外观\n'])
            writers.write(data)
            writers.write('\n\n')
            file.close()

            file = open('./tmp/b.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请进行一般工作正常性检查')
            #     return
            writers.write('  1.2 一般工作正常性检查\n')
            writers.write(data)
            writers.write('\n\n')
            file.close()

            file = open('./tmp/c.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请进行止血时间检查')
            #     return
            writers.write('  1.3 止血时间检查\n')
            writers.write(data)
            writers.write('\n\n')
            file.close()

            file = open('./tmp/d.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请进行止血时间检查')
            #     return
            writers.write('  1.4 压力控制抗干扰能力检查\n')
            writers.write(data)
            writers.write('\n\n')
            file.close()

            file = open('./tmp/e.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请进行加压上升时间检测')
            #     return
            writers.write('2 加压上升时间\n')
            writers.write(data)
            writers.write('\n\n')
            file.close()

            file = open('./tmp/f.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请进行压力设置值与袖带压力示值的一致性')
            #     return
            writers.write('3 压力设置值与袖带压力示值的一致性\n')
            writers.write(data)
            writers.write('\n\n')
            file.close()

            file = open('./tmp/g.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请进行气密性检测')
            #     return
            writers.write('4 气密性\n')
            writers.write(data)
            writers.write('\n\n')
            file.close()

            file = open('./tmp/h.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请进行气密性检测')
            #     return
            writers.write('5 袖带压力示值误差\n')
            writers.write(data)
            writers.write('\n\n')
            file.close()

            file = open('./tmp/i.txt', 'r', encoding='utf-8')
            data = file.read()
            # if data == '':
            #     QMessageBox.information(self, 'info', '请进行压力波动度检测')
            #     return
            writers.write('6 压力波动度\n')
            writers.write(data)
            writers.write('\n\n')
            file.close()

        self.listWidget.clear()
        for file in glob.glob(self.path + r"/*.txt"):
            name = file.strip().split('/')[-1]
            self.listWidget.addItem(name)

    def search_file(self):
        self.listWidget.clear()
        self.textEdit.clear()
        if self.lineEdit.text() == '':
            for file in glob.glob(self.path + r"/*.txt"):
                name = file.strip().split('/')[-1]
                self.listWidget.addItem(name)
        else:
            keys = self.lineEdit.text().strip().split(' ')
            for file in glob.glob(self.path + r"/*.txt"):
                name = file.strip().split('/')[-1]
                is_valid = True
                for k in keys:
                    if k not in name:
                        is_valid = False
                        break
                if is_valid:
                    self.listWidget.addItem(name)

    def display_file(self):
        self.textEdit.clear()
        name = self.listWidget.currentItem().text()
        file_path = self.path + '/' + name
        if os.path.isfile(file_path):
            # 读取文件内容并在文本框中显示
            with open(file_path, "r", encoding='utf-8') as f:
                content = f.read()
                self.textEdit.setText(content)
