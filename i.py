# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'i.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
import pic_rc
import time, os

global P


class UI_I(QtWidgets.QMainWindow):
    def __init__(self, fa, ADC):
        super(UI_I, self).__init__()
        self.DATA = {} # 保存当前页面的数据信息
        self.father = fa # 父界面，即系统主界面
        self.path = './tmp/i.txt' # 当前功能实验结果临时文件
        self.ADC = ADC # 压力传感器
        self.setupUi(self)

    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(800, 410)
        self.textBrowser_1 = QtWidgets.QTextBrowser(Form)
        self.textBrowser_1.setGeometry(QtCore.QRect(10, 10, 441, 81))
        self.textBrowser_1.setStyleSheet("background-color:yellow\n"
                                         "")
        self.textBrowser_1.setObjectName("textBrowser_1")
        self.textBrowser_2 = QtWidgets.QTextBrowser(Form)
        self.textBrowser_2.setGeometry(QtCore.QRect(10, 129, 151, 91))
        self.textBrowser_2.setStyleSheet("background-color:yellow")
        self.textBrowser_2.setObjectName("textBrowser_2")
        self.gridLayoutWidget_3 = QtWidgets.QWidget(Form)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(190, 138, 255, 51))
        self.gridLayoutWidget_3.setObjectName("gridLayoutWidget_3")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.gridLayoutWidget_3)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit.sizePolicy().hasHeightForWidth())
        self.lineEdit.setSizePolicy(sizePolicy)
        self.lineEdit.setMinimumSize(QtCore.QSize(70, 0))
        self.lineEdit.setMaximumSize(QtCore.QSize(70, 16777215))
        self.lineEdit.setObjectName("lineEdit")
        self.gridLayout_3.addWidget(self.lineEdit, 0, 1, 1, 1)
        self.lineEdit_3 = QtWidgets.QLineEdit(self.gridLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_3.sizePolicy().hasHeightForWidth())
        self.lineEdit_3.setSizePolicy(sizePolicy)
        self.lineEdit_3.setMaximumSize(QtCore.QSize(70, 16777215))
        self.lineEdit_3.setSizeIncrement(QtCore.QSize(70, 0))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.gridLayout_3.addWidget(self.lineEdit_3, 1, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.gridLayoutWidget_3)
        self.label_4.setObjectName("label_4")
        self.gridLayout_3.addWidget(self.label_4, 0, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.gridLayoutWidget_3)
        self.label_6.setObjectName("label_6")
        self.gridLayout_3.addWidget(self.label_6, 1, 0, 1, 1)
        self.frame_3 = QtWidgets.QFrame(Form)
        self.frame_3.setGeometry(QtCore.QRect(180, 130, 271, 90))
        self.frame_3.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setLineWidth(1)
        self.frame_3.setMidLineWidth(0)
        self.frame_3.setObjectName("frame_3")
        self.pushButton = QtWidgets.QPushButton(self.frame_3)
        self.pushButton.setGeometry(QtCore.QRect(32, 64, 201, 23))
        self.pushButton.setObjectName("pushButton")
        self.textBrowser_3 = QtWidgets.QTextBrowser(Form)
        self.textBrowser_3.setGeometry(QtCore.QRect(10, 270, 151, 61))
        self.textBrowser_3.setStyleSheet("background-color:yellow\n"
                                         "")
        self.textBrowser_3.setObjectName("textBrowser_3")
        self.frame_4 = QtWidgets.QFrame(Form)
        self.frame_4.setGeometry(QtCore.QRect(180, 270, 271, 90))
        self.frame_4.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setLineWidth(1)
        self.frame_4.setMidLineWidth(0)
        self.frame_4.setObjectName("frame_4")
        self.gridLayoutWidget_4 = QtWidgets.QWidget(Form)
        self.gridLayoutWidget_4.setGeometry(QtCore.QRect(189, 274, 255, 84))
        self.gridLayoutWidget_4.setObjectName("gridLayoutWidget_4")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.gridLayoutWidget_4)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.label_8 = QtWidgets.QLabel(self.gridLayoutWidget_4)
        self.label_8.setObjectName("label_8")
        self.gridLayout_4.addWidget(self.label_8, 0, 0, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.gridLayoutWidget_4)
        self.label_10.setObjectName("label_10")
        self.gridLayout_4.addWidget(self.label_10, 2, 0, 1, 1)
        self.lineEdit_2 = QtWidgets.QLineEdit(self.gridLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_2.sizePolicy().hasHeightForWidth())
        self.lineEdit_2.setSizePolicy(sizePolicy)
        self.lineEdit_2.setMaximumSize(QtCore.QSize(70, 16777215))
        self.lineEdit_2.setSizeIncrement(QtCore.QSize(70, 0))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.gridLayout_4.addWidget(self.lineEdit_2, 0, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.gridLayoutWidget_4)
        self.label_2.setMaximumSize(QtCore.QSize(70, 16777215))
        self.label_2.setSizeIncrement(QtCore.QSize(70, 0))
        self.label_2.setText("")
        self.label_2.setObjectName("label_2")
        self.gridLayout_4.addWidget(self.label_2, 2, 1, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.gridLayoutWidget_4)
        self.label_9.setObjectName("label_9")
        self.gridLayout_4.addWidget(self.label_9, 1, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.gridLayoutWidget_4)
        self.label.setMaximumSize(QtCore.QSize(70, 16777215))
        self.label.setSizeIncrement(QtCore.QSize(70, 0))
        self.label.setText("")
        self.label.setObjectName("label")
        self.gridLayout_4.addWidget(self.label, 1, 1, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(Form)
        self.pushButton_2.setGeometry(QtCore.QRect(34, 330, 101, 30))
        self.pushButton_2.setObjectName("pushButton_2")
        self.label_png = QtWidgets.QLabel(Form)
        self.label_png.setGeometry(QtCore.QRect(470, 10, 310, 210))
        self.label_png.setText("")
        self.label_png.setPixmap(QtGui.QPixmap(":/resources/device.png"))
        self.label_png.setScaledContents(True)
        self.label_png.setObjectName("label_png")
        self.frame_5 = QtWidgets.QFrame(Form)
        self.frame_5.setGeometry(QtCore.QRect(470, 266, 311, 90))
        self.frame_5.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setLineWidth(1)
        self.frame_5.setMidLineWidth(0)
        self.frame_5.setObjectName("frame_5")
        self.gridLayoutWidget_5 = QtWidgets.QWidget(Form)
        self.gridLayoutWidget_5.setGeometry(QtCore.QRect(480, 270, 291, 84))
        self.gridLayoutWidget_5.setObjectName("gridLayoutWidget_5")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.gridLayoutWidget_5)
        self.gridLayout_5.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.label_5 = QtWidgets.QLabel(self.gridLayoutWidget_5)
        self.label_5.setMinimumSize(QtCore.QSize(100, 0))
        self.label_5.setMaximumSize(QtCore.QSize(100, 16777215))
        self.label_5.setText("")
        self.label_5.setObjectName("label_5")
        self.gridLayout_5.addWidget(self.label_5, 0, 1, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.gridLayoutWidget_5)
        self.label_3.setText("")
        self.label_3.setObjectName("label_3")
        self.gridLayout_5.addWidget(self.label_3, 1, 1, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.gridLayoutWidget_5)
        self.label_11.setObjectName("label_11")
        self.gridLayout_5.addWidget(self.label_11, 0, 0, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.gridLayoutWidget_5)
        self.label_12.setObjectName("label_12")
        self.gridLayout_5.addWidget(self.label_12, 1, 0, 1, 1)
        self.pushButton_clear = QtWidgets.QPushButton(Form)
        self.pushButton_clear.setGeometry(QtCore.QRect(470, 370, 80, 30))
        self.pushButton_clear.setObjectName("pushButton_clear")
        self.pushButton_return = QtWidgets.QPushButton(Form)
        self.pushButton_return.setGeometry(QtCore.QRect(700, 370, 80, 30))
        self.pushButton_return.setObjectName("pushButton_return")
        self.pushButton_3 = QtWidgets.QPushButton(Form)
        self.pushButton_3.setGeometry(QtCore.QRect(470, 230, 111, 30))
        font = QtGui.QFont()
        font.setFamily("Adobe Devanagari")
        font.setPointSize(14)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_save = QtWidgets.QPushButton(Form)
        self.pushButton_save.setGeometry(QtCore.QRect(590, 370, 80, 30))
        self.pushButton_save.setObjectName("pushButton_save")
        self.frame_3.raise_()
        self.textBrowser_1.raise_()
        self.textBrowser_2.raise_()
        self.gridLayoutWidget_3.raise_()
        self.textBrowser_3.raise_()
        self.frame_4.raise_()
        self.gridLayoutWidget_4.raise_()
        self.pushButton_2.raise_()
        self.label_png.raise_()
        self.frame_5.raise_()
        self.gridLayoutWidget_5.raise_()
        self.pushButton_clear.raise_()
        self.pushButton_return.raise_()
        self.pushButton_3.raise_()
        self.pushButton_save.raise_()

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle("压力波动度检测")
        self.textBrowser_1.setHtml(_translate("Form",
                                              "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                              "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                              "p, li { white-space: pre-wrap; }\n"
                                              "</style></head><body style=\" font-family:\'SimSun\'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">(1)按图连接止血带机通道1和标准器气路，将小号止血袖带卷扎在合适尺寸硬质圆柱上</p>\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">(2)打开压力标准器和止血袖带气路开关3和4，关闭其他开关</p>\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">(3)设置止血带机工作压力为最大工作压力80%，止血时间为20min</p></body></html>"))
        self.textBrowser_2.setHtml(_translate("Form",
                                              "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                              "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                              "p, li { white-space: pre-wrap; }\n"
                                              "</style></head><body style=\" font-family:\'SimSun\'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">提示：在标准器上输入止血带机最大工作压力和压力控制精度，启动止血带机工作并等待止血带机加压结束</p></body></html>"))
        self.label_4.setText(_translate("Form", "最大工作压力kPa"))
        self.label_6.setText(_translate("Form", "压力控制精度kPa"))
        self.pushButton.setText(_translate("Form", "输入完成且加压结束"))
        self.textBrowser_3.setHtml(_translate("Form",
                                              "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                              "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                              "p, li { white-space: pre-wrap; }\n"
                                              "</style></head><body style=\" font-family:\'SimSun\'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
                                              "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">提示：保持现有状态，共进行10次测量，等待测量结束</p></body></html>"))
        self.label_8.setText(_translate("Form", "测量次数"))
        self.label_10.setText(_translate("Form", "压力测量值kPa"))
        self.label_9.setText(_translate("Form", "60s测量时间倒计时s"))
        self.pushButton_2.setText(_translate("Form", "开始测量"))
        self.label_11.setText(_translate("Form", "压力设置值kPa"))
        self.label_12.setText(_translate("Form", "10min压力波动kPa"))
        self.pushButton_clear.setText(_translate("Form", "还原"))
        self.pushButton_return.setText(_translate("Form", "返回"))
        self.pushButton_3.setText(_translate("Form", "检测结果"))
        self.pushButton_save.setText(_translate("Form", "保存"))

        # ----------------------------------------绑定按钮槽函数---------------------------------------------
        self.pushButton.clicked.connect(self.i_1)
        self.pushButton_2.clicked.connect(self.i_2)
        self.pushButton_3.clicked.connect(self.i_over)
        self.pushButton_clear.clicked.connect(self.i_clear)
        self.pushButton_save.clicked.connect(self.save_file)
        # --------------------------------------------------------------------------------------------------

        # 返回主程序
        self.pushButton_return.clicked.connect(lambda: {self.father.show(), self.close()})

        # 创建一个临时txt
        if os.path.exists(self.path):
            os.remove(self.path)
        file = open(self.path, 'w')
        file.close()


    def i_1(self):
        # 输入相应参数
        # 如果是数字，先判断是否填写了，如果填写了再判断是否为数字形式
        # 不符合要求进行对话框提醒
        if self.lineEdit.text() == '':
            QMessageBox.information(self, 'info', '请输入最大工作压力')
            return
        else:
            if self.lineEdit.text().replace(".", "").isdigit():
                self.DATA['最大工作压力'] = float(self.lineEdit.text())
            else:
                QMessageBox.information(self, 'info', '请输入正确参数')
                return

        if self.lineEdit_3.text() == '':
            QMessageBox.information(self, 'info', '请输入压力控制精度')
            return
        else:
            if self.lineEdit_3.text().replace(".", "").isdigit():
                self.DATA['压力控制精度'] = float(self.lineEdit_3.text())
            else:
                QMessageBox.information(self, 'info', '请输入正确参数')
                return

    def i_2(self):
        global P # 保存10min的压力测量值，用于后续计算
        P = []
        if self.lineEdit_2.text() == '':
            QMessageBox.information(self, 'info', '请输入当前测量次序')
            return
        else:
            if self.lineEdit_2.text().replace(".", "").isdigit():
                # 判断是否测量超过10次
                if len(P) > 10:
                    QMessageBox.information(self, 'info', '已测量10次,点击检测结果按钮查看结果')
                    return

                time_start = time.time()
                while True:
                    time.sleep(0.5)
                    time_end = time.time()
                    tm = time_end - time_start
                    # 读一个当前压力
                    p_s = self.ADC.ADS1256_GetChannalValue(6) * 500 / 0x7fffff  # 高精度传感器

                    self.label.setText(f'{tm:.2f}')
                    self.label.repaint()
                    self.label_2.setText(f'{p_s:.2f}')
                    self.label_2.repaint()

                    if tm >= 60:
                        P.append(p_s)
                        break
            else:
                QMessageBox.information(self, 'info', '请输入正确参数')
                return

    def i_over(self):
        # 计算检测结果
        global P
        if '最大工作压力' in self.DATA:
            self.label_5.setText(str(self.DATA['最大工作压力']))
            ps_max = max(P)
            ps_min = min(P)
            delta_p = (ps_max - ps_min) / 2
            self.label_3.setText(f'{delta_p:.2f}')
        else:
            QMessageBox.information(self, 'info', '请确认输入参数')
            return

    def i_clear(self):
        # 还原按钮
        global P
        P = []
        self.lineEdit.clear()
        self.lineEdit_3.clear()
        self.lineEdit_2.clear()
        self.label.clear()
        self.label_2.clear()
        self.label_5.clear()
        self.label_3.clear()

    def save_file(self):
        # 保存文件
        with open(self.path, 'w', encoding='utf-8') as writers:
            
            writers.write('    10分钟压力波动: ' + self.label_3.text() + "kPa\n")

    def closeEvent(self, event):
        # 重载界面关闭事件
        self.father.resize(800, 400)
        event.accept()


# if __name__ == "__main__":
#     import sys

#     app = QtWidgets.QApplication(sys.argv)
#     gui = UI_I()
#     gui.show()
#     sys.exit(app.exec_())
