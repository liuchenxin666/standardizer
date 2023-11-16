import sys
from PyQt5 import QtWidgets
from a import UI_A
from b import UI_B
from c import UI_C
from d import UI_D
from e import UI_E
from f import UI_F
from g import UI_G
from h import UI_H
from i import UI_I
from main_window import UI_MAIN

import ADS1256
import os
import config


ADC = ADS1256.ADS1256()
ADC.ADS1256_init()
ADC.ADS1256_SetMode(0)


# -----------------------------------重载a~i子界面类-------------------------------------------
class main_window(UI_MAIN):
    def __init__(self, ADC):
        super(main_window, self).__init__(ADC)
        self.setupUi(self)


class a_window(UI_A):
    def __init__(self, fa):
        super(a_window, self).__init__(fa)
        self.setupUi(self)


class b_window(UI_B):
    def __init__(self, fa, ADC):
        super(b_window, self).__init__(fa, ADC)
        self.setupUi(self)

class c_window(UI_C):
    def __init__(self, fa):
        super(c_window, self).__init__(fa)
        self.setupUi(self)

class d_window(UI_D):
    def __init__(self, fa, ADC):
        super(d_window, self).__init__(fa, ADC)
        self.setupUi(self)
    
class e_window(UI_E):
    def __init__(self, fa, ADC):
        super(e_window, self).__init__(fa, ADC)
        self.setupUi(self)

class f_window(UI_F):
    def __init__(self, fa, ADC):
        super(f_window, self).__init__(fa, ADC)
        self.setupUi(self)

class g_window(UI_G):
    def __init__(self, fa, ADC):
        super(g_window, self).__init__(fa, ADC)
        self.setupUi(self)

class h_window(UI_H):
    def __init__(self, fa, ADC):
        super(h_window, self).__init__(fa, ADC)
        self.setupUi(self)

class i_window(UI_I):
    def __init__(self, fa, ADC):
        super(i_window, self).__init__(fa, ADC)
        self.setupUi(self)
# --------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    # -----------------------------------声明a~i子界面对象-------------------------------------------
    gui = main_window(ADC)
    gui_a = a_window(gui)
    gui_b = b_window(gui, ADC)
    gui_c = c_window(gui)
    gui_d = d_window(gui, ADC)
    gui_e = e_window(gui, ADC)
    gui_f = f_window(gui, ADC)
    gui_g = g_window(gui, ADC)
    gui_h = h_window(gui, ADC)
    gui_i = i_window(gui, ADC)
    # -----------------------------------------------------------------------------------------------
    # 主界面显示
    gui.show()

    # -----------------------------------绑定槽函数，调度a~i子界面-------------------------------------
    gui.pushButton_2_outer.clicked.connect(lambda: {gui_a.show(), gui.resize(0, 0)})
    gui.pushButton_2_normal.clicked.connect(lambda: {gui_b.show(), gui.resize(0, 0)})
    gui.pushButton_2_hemostasis.clicked.connect(lambda: {gui_c.show(), gui.resize(0, 0)})
    gui.pushButton_2_anti.clicked.connect(lambda: {gui_d.show(), gui.resize(0, 0)})
    gui.pushButton.clicked.connect(lambda: {gui_e.show(), gui.resize(0, 0)})
    gui.pushButton_5.clicked.connect(lambda: {gui_f.show(), gui.resize(0, 0)})
    gui.pushButton_4.clicked.connect(lambda: {gui_g.show(), gui.resize(0, 0)})
    gui.pushButton_2.clicked.connect(lambda: {gui_h.show(), gui.resize(0, 0)})
    gui.pushButton_3.clicked.connect(lambda: {gui_i.show(), gui.resize(0, 0)})
    # -----------------------------------------------------------------------------------------------


    if not os.path.exists('./outputs/'):
        os.makedirs('./outputs/')
    if not os.path.exists('./tmp/'):
        os.makedirs('./tmp/')
    sys.exit(app.exec_())
