# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'e.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
import pic_rc
import time, os


class UI_E(QtWidgets.QMainWindow):
    def __init__(self, fa, ADC):
        super(UI_E, self).__init__()
        self.DATA = {} # 保存当前页面的数据信息
        self.father = fa # 父界面，即系统主界面
        self.path = './tmp/e.txt' # 当前功能实验结果临时文件
        self.ADC = ADC # 压力传感器
        self.setupUi(self)

    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(800, 410)
        self.textBrowser_1 = QtWidgets.QTextBrowser(Form)
        self.textBrowser_1.setGeometry(QtCore.QRect(10, 0, 381, 71))
        self.textBrowser_1.setStyleSheet("background-color:yellow\n"
                                         "")
        self.textBrowser_1.setObjectName("textBrowser_1")
        self.frame_1 = QtWidgets.QFrame(Form)
        self.frame_1.setGeometry(QtCore.QRect(10, 75, 381, 130))
        self.frame_1.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_1.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_1.setLineWidth(1)
        self.frame_1.setMidLineWidth(0)
        self.frame_1.setObjectName("frame_1")
        self.gridLayoutWidget = QtWidgets.QWidget(Form)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(20, 80, 361, 121))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.label_6 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 3, 0, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.gridLayout.addWidget(self.label_7, 0, 0, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.gridLayout.addWidget(self.label_8, 1, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 2, 0, 1, 1)
        self.lineEdit_3 = QtWidgets.QLineEdit(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_3.sizePolicy().hasHeightForWidth())
        self.lineEdit_3.setSizePolicy(sizePolicy)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.gridLayout.addWidget(self.lineEdit_3, 0, 1, 1, 1)
        self.lineEdit_4 = QtWidgets.QLineEdit(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_4.sizePolicy().hasHeightForWidth())
        self.lineEdit_4.setSizePolicy(sizePolicy)
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.gridLayout.addWidget(self.lineEdit_4, 1, 1, 1, 1)
        self.lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit.sizePolicy().hasHeightForWidth())
        self.lineEdit.setSizePolicy(sizePolicy)
        self.lineEdit.setObjectName("lineEdit")
        self.gridLayout.addWidget(self.lineEdit, 2, 1, 1, 1)
        self.pushButton = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout.addWidget(self.pushButton, 3, 1, 1, 1)
        self.textBrowser_2 = QtWidgets.QTextBrowser(Form)
        self.textBrowser_2.setGeometry(QtCore.QRect(10, 220, 381, 61))
        self.textBrowser_2.setStyleSheet("background-color:yellow")
        self.textBrowser_2.setObjectName("textBrowser_2")
        self.gridLayoutWidget_2 = QtWidgets.QWidget(Form)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(20, 295, 361, 101))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_10 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.gridLayout_2.addWidget(self.label_10, 0, 0, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.gridLayout_2.addWidget(self.label_11, 1, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 2, 0, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.gridLayout_2.addWidget(self.label_9, 3, 0, 1, 1)
        self.lineEdit_5 = QtWidgets.QLineEdit(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_5.sizePolicy().hasHeightForWidth())
        self.lineEdit_5.setSizePolicy(sizePolicy)
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.gridLayout_2.addWidget(self.lineEdit_5, 0, 1, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.label_3.setText("")
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 1, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.label_4.setText("")
        self.label_4.setObjectName("label_4")
        self.gridLayout_2.addWidget(self.label_4, 2, 1, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout_2.addWidget(self.pushButton_2, 3, 1, 1, 1)
        self.frame_2 = QtWidgets.QFrame(Form)
        self.frame_2.setGeometry(QtCore.QRect(10, 290, 381, 111))
        self.frame_2.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setLineWidth(1)
        self.frame_2.setMidLineWidth(0)
        self.frame_2.setObjectName("frame_2")
        self.label_png = QtWidgets.QLabel(Form)
        self.label_png.setGeometry(QtCore.QRect(440, 0, 340, 210))
        self.label_png.setText("")
        self.label_png.setPixmap(QtGui.QPixmap(":/resources/device.png"))
        self.label_png.setScaledContents(True)
        self.label_png.setObjectName("label_png")
        self.textBrowser = QtWidgets.QTextBrowser(Form)
        self.textBrowser.setGeometry(QtCore.QRect(440, 250, 341, 111))
        self.textBrowser.setObjectName("textBrowser")
        self.pushButton_clear = QtWidgets.QPushButton(Form)
        self.pushButton_clear.setGeometry(QtCore.QRect(440, 370, 80, 30))
        self.pushButton_clear.setObjectName("pushButton_clear")
        self.pushButton_return = QtWidgets.QPushButton(Form)
        self.pushButton_return.setGeometry(QtCore.QRect(700, 370, 80, 30))
        self.pushButton_return.setObjectName("pushButton_return")
        self.pushButton_3 = QtWidgets.QPushButton(Form)
        self.pushButton_3.setGeometry(QtCore.QRect(440, 220, 130, 30))
        font = QtGui.QFont()
        font.setFamily("Adobe Devanagari")
        font.setPointSize(14)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setStyleSheet("")
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_save = QtWidgets.QPushButton(Form)
        self.pushButton_save.setGeometry(QtCore.QRect(580, 370, 80, 30))
        self.pushButton_save.setObjectName("pushButton_save")
        self.frame_2.raise_()
        self.frame_1.raise_()
        self.textBrowser_1.raise_()
        self.gridLayoutWidget.raise_()
        self.textBrowser_2.raise_()
        self.gridLayoutWidget_2.raise_()
        self.label_png.raise_()
        self.textBrowser.raise_()
        self.pushButton_clear.raise_()
        self.pushButton_return.raise_()
        self.pushButton_3.raise_()
        self.pushButton_save.raise_()

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle("加压上升时间检测")
        self.textBrowser_1.setHtml(_translate("Form",
                                              "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                              "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                              "p, li { white-space: pre-wrap; }\n"
                                              "</style></head><body style=\" font-family:\'SimSun\'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">提示：(1)选择大号或中号止血带，按图连接止血带机和标准器</p>\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">(2)打开500mL硬质容器、压力标准器和止血袖带气路开关2、3和4，关闭其他开关</p></body></html>"))
        self.label_6.setText(_translate("Form", "输入相关参数"))
        self.label_7.setText(_translate("Form", "最大工作压力数值(kPa)"))
        self.label_8.setText(_translate("Form", "最长加压上升时间(默认60s)"))
        self.label.setText(_translate("Form", "通道总数(默认为1)"))
        self.pushButton.setText(_translate("Form", "确认"))
        self.textBrowser_2.setHtml(_translate("Form",
                                              "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                              "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                              "p, li { white-space: pre-wrap; }\n"
                                              "</style></head><body style=\" font-family:\'SimSun\'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">提示：(1)调整止血带机检测通道数</p>\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">(2)按下止血带机放气键，等待示值显示压力≤2kPa</p>\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">(3)同时按压止血带机加压键和标准器计时器启动键</p></body></html>"))
        self.label_10.setText(_translate("Form", "当前测试的通道序号"))
        self.label_11.setText(_translate("Form", "加压上升时间(s)"))
        self.label_2.setText(_translate("Form", "压力测量值(kPa)"))
        self.label_9.setText(_translate("Form", "输入相关参数"))
        self.pushButton_2.setText(_translate("Form", "计时器启动键"))
        self.pushButton_clear.setText(_translate("Form", "还原"))
        self.pushButton_return.setText(_translate("Form", "返回"))
        self.pushButton_3.setText(_translate("Form", "检测结果"))
        self.pushButton_save.setText(_translate("Form", "保存"))

        # -----------------------------------初始化，调整文本框可视-------------------------------------------
        self.textBrowser.hide()
        # --------------------------------------------------------------------------------------------------

        # ----------------------------------------绑定按钮槽函数---------------------------------------------
        self.pushButton.clicked.connect(self.e_1)
        self.pushButton_2.clicked.connect(self.e_2)
        self.pushButton_3.clicked.connect(self.e_over)
        self.pushButton_clear.clicked.connect(self.e_clear)
        self.pushButton_save.clicked.connect(self.save_file)
        # --------------------------------------------------------------------------------------------------

        # 返回主程序
        self.pushButton_return.clicked.connect(lambda: {self.father.show(), self.close()})

        # 创建一个临时txt
        if os.path.exists(self.path):
            os.remove(self.path)
        file = open(self.path, 'w')
        file.close()

    def e_1(self):
        # 输入相应参数
        # 如果是数字，先判断是否填写了，如果填写了再判断是否为数字形式
        # 不符合要求进行对话框提醒
        if self.lineEdit_3.text() == '':
            QMessageBox.information(self, 'info', '请输入相关参数')
            return
        else:
            if self.lineEdit_3.text().replace(".", "").isdigit():
                self.DATA['最大工作压力'] = float(self.lineEdit_3.text())
            else:
                QMessageBox.information(self, 'info', '请输入正确参数')
                return

        if self.lineEdit_4.text() == '':
            self.DATA['最长加压上升时间'] = 60.0
            self.lineEdit_4.setText("60")
        else:
            if self.lineEdit_4.text().replace(".", "").isdigit():
                self.DATA['最长加压上升时间'] = float(self.lineEdit_4.text())
            else:
                QMessageBox.information(self, 'info', '请输入正确参数')
                return

        if self.lineEdit.text() == '':
            self.DATA['通道总数'] = 1
            self.lineEdit.setText("1")
        else:
            if self.lineEdit.text().replace(".", "").isdigit():
                self.DATA['通道总数'] = int(self.lineEdit.text())
            else:
                QMessageBox.information(self, 'info', '请输入正确参数')
                return

    def e_2(self):
        # 输入当前通道序号，开始检测加压上升时间
        if self.lineEdit_5.text() == '':
            QMessageBox.information(self, 'info', '请先输入当前通道序号')
            return
        else:
            if self.lineEdit_5.text().replace(".", "").isdigit():
                crt_num = int(self.lineEdit_5.text())
            else:
                QMessageBox.information(self, 'info', '请输入正确的通道序号')
                return
        time_start = time.time()
        # 实时显示
        while True:
            time.sleep(0.5)
            time_end = time.time()
            t_m = time_end - time_start
            # 硬件读一个压力值
            p_sn = self.ADC.ADS1256_GetChannalValue(6) * 500 / 0x7fffff  # 高精度传感器
            self.label_3.setText(f'{t_m:.2f}')
            self.label_3.repaint()
            self.label_4.setText(f'{p_sn:.2f}')
            self.label_4.repaint()

            # 加压时间超时
            if t_m >= self.DATA['最长加压上升时间']:
                self.textBrowser.append(
                    "通道" + str(crt_num) + "加压时间超时,压力值为" + f'{p_sn:.2f}' + "kPa,计时时间为" + f'{t_m:.2f}' + "s")
                # self.label_3.setText("")
                # self.label_3.repaint()
                # self.label_4.setText("")
                # self.label_4.repaint()
                break

            # 加压完成
            if p_sn >= 0.9 * self.DATA['最大工作压力']:
                self.textBrowser.append("通道" + str(crt_num) + "压力值为" + f'{p_sn:.2f}' + "kPa,计时时间为" + f'{t_m:.2f}' + "s")
                break

    def e_over(self):
        # 查询检测结果
        self.textBrowser.show()

    def e_clear(self):
        # 还原按钮
        self.lineEdit_3.clear()
        self.lineEdit_4.clear()
        self.lineEdit.clear()
        self.lineEdit_5.clear()
        self.label_3.setText("")
        self.label_4.setText("")
        self.textBrowser.clear()
        self.textBrowser.hide()
    
    def save_file(self):
        # 保存文件
        with open(self.path, 'w', encoding='utf-8') as writers:
            
            writers.write('    最大工作压力: ' + self.lineEdit.text() + "kPa\n      ")
            content = self.textBrowser.toPlainText().replace('\n', ';\n      ')
            writers.write(content)
    
    def closeEvent(self, event):
        # 重载界面关闭事件
        self.father.resize(800, 400)
        event.accept()


# if __name__ == "__main__":
#     import sys

#     app = QtWidgets.QApplication(sys.argv)
#     gui = UI_E()
#     gui.show()
#     sys.exit(app.exec_())
