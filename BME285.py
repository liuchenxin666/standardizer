import smbus  # 导入I2C的SMBus模
import time  # 导入延时函数
from ctypes import c_short
from ctypes import c_byte
from ctypes import c_ubyte
import math

class BME285_Class():
    def __init__(self):  # MPU 6050 初始化工作
        self.DEVICE = 0x76  # Default device I2C address
        self.REG_DATA = 0xF7
        self.REG_CONTROL = 0xF4
        self.REG_CONFIG = 0xF5

        self.REG_CONTROL_HUM = 0xF2
        self.REG_HUM_MSB = 0xFD
        self.REG_HUM_LSB = 0xFE

        # Oversample setting - page 27
        self.OVERSAMPLE_TEMP = 2
        self.OVERSAMPLE_PRES = 2
        self.MODE = 1

        # Oversample setting for humidity register - page 26
        self.OVERSAMPLE_HUM = 2
        self.REG_ID = 0xD0

        self.env_bus = smbus.SMBus(1)  # Rev 2 Pi, Pi 2 & Pi 3 uses bus 1


        # Rev 1 Pi uses bus 0

    def getShort(self,data, index):
            # return two bytes from data as a signed 16-bit value
        return c_short((data[index + 1] << 8) + data[index]).value

    def getUShort(self,data, index):
            # return two bytes from data as an unsigned 16-bit value
        return (data[index + 1] << 8) + data[index]

    def getChar(self,data, index):
            # return one byte from data as a signed char
        result = data[index]
        if result > 127:
            result -= 256
        return result

    def getUChar(self,data, index):
            # return one byte from data as an unsigned char
        result = data[index] & 0xFF
        return result

    def readBME280ID(self):
            # Chip ID Register Address
        addr=self.DEVICE           
        (chip_id, chip_version) = bus.read_i2c_block_data(addr, self.REG_ID, 2)
        return (chip_id, chip_version)
        '''
        # 一些MPU6050寄存器及其地址
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.INT_ENABLE = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47
        self.makerobo_bus = smbus.SMBus(1)  # 或bus = smbus.SMBus(0)用于较老的版本板
        self.makerobo_Device_Address = 0x68  # MPU6050设备地址
        # 写入抽样速率寄存器
        self.makerobo_bus.write_byte_data(self.makerobo_Device_Address, self.SMPLRT_DIV, 7)

        # 写入电源管理寄存器
        self.makerobo_bus.write_byte_data(self.makerobo_Device_Address, self.PWR_MGMT_1, 1)

        # 写入配置寄存器
        self.makerobo_bus.write_byte_data(self.makerobo_Device_Address, self.CONFIG, 0)

        # 写入陀螺配置寄存器
        self.makerobo_bus.write_byte_data(self.makerobo_Device_Address, self.GYRO_CONFIG, 24)

        # 写中断使能寄存器
        self.makerobo_bus.write_byte_data(self.makerobo_Device_Address, self.INT_ENABLE, 1)


    # 读取MPU6050数据寄存器
    def makerobo_read_raw_data(self,addr):
        # 加速度值和陀螺值为16位
        high = self.makerobo_bus.read_byte_data(self.makerobo_Device_Address, addr)
        low = self.makerobo_bus.read_byte_data(self.makerobo_Device_Address, addr + 1)

        # 连接更高和更低的值
        value = ((high << 8) | low)

        # 从mpu6050获取有符号值
        if (value > 32768):
            value = value - 65536
        return value
    last_pitch=0
    last_roll=0
    
    def get_data(self):
        # 读取加速度计原始值
        acc_x = self.makerobo_read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.makerobo_read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.makerobo_read_raw_data(self.ACCEL_ZOUT_H)

        # 读陀螺仪原始值
        gyro_x = self.makerobo_read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.makerobo_read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.makerobo_read_raw_data(self.GYRO_ZOUT_H)

        # 全刻度范围+/- 250度/℃，根据灵敏度刻度系数
        self.ax = acc_x / 16384.0
        self.ay = acc_y / 16384.0
        self.az = acc_z / 16384.0

        self.gx = gyro_x / 131.0
        self.gy = gyro_y / 131.0
        self.gz = gyro_z / 131.0
        #########################################一阶互补滤波姿态角解算###############################
        
        #k=0.1
        #dt=0.5#dt为采样周期，0.01说明要10ms执行一次数据采集

        #acc_pitch=math.atan(self.ax/self.az)*57.2974
        #acc_roll=math.atan(self.ay/self.az)*57.2974
        #self.pitch=k*acc_pitch+(1-k)*(self.last_pitch+self.gx*dt)
        #self.last_pitch=self.pitch
        #self.roll=k*acc_roll+(1-k)*(self.last_roll+self.gy*dt)
        #self.last_roll=self.roll
        
        #acc_pitch=math.atan(self.ax/self.az)*57.2974
        #acc_roll=math.atan(self.ay/self.az)*57.2974
        #self.pitch=acc_pitch
        #self.roll=acc_roll
'''

    def readBME280All(self):
            # Register Addresses
            addr=self.DEVICE
            self.env_bus.write_byte_data(addr, self.REG_CONTROL_HUM, self.OVERSAMPLE_HUM)

            control = self.OVERSAMPLE_TEMP << 5 | self.OVERSAMPLE_PRES << 2 | self.MODE
            self.env_bus.write_byte_data(addr, self.REG_CONTROL, control)

            # Read blocks of calibration data from EEPROM
            # See Page 22 data sheet
            cal1 = self.env_bus.read_i2c_block_data(addr, 0x88, 24)
        
            cal2 = self.env_bus.read_i2c_block_data(addr, 0xA1, 1)
            cal3 = self.env_bus.read_i2c_block_data(addr, 0xE1, 7)

            # Convert byte data to word values
            dig_T1 = self.getUShort(cal1, 0)
            dig_T2 = self.getShort(cal1, 2)
            dig_T3 =  self.getShort(cal1, 4)

            dig_P1 = self. getUShort(cal1, 6)
            dig_P2 =  self.getShort(cal1, 8)
            dig_P3 =  self.getShort(cal1, 10)
            dig_P4 =  self.getShort(cal1, 12)
            dig_P5 =  self.getShort(cal1, 14)
            dig_P6 =  self.getShort(cal1, 16)
            dig_P7 =  self.getShort(cal1, 18)
            dig_P8 =  self.getShort(cal1, 20)
            dig_P9 =  self.getShort(cal1, 22)

            dig_H1 =  self.getUChar(cal2, 0)
            dig_H2 =  self.getShort(cal3, 0)
            dig_H3 =  self.getUChar(cal3, 2)

            dig_H4 =  self.getChar(cal3, 3)
            dig_H4 = (dig_H4 << 24) >> 20
            dig_H4 = dig_H4 | (self.getChar(cal3, 4) & 0x0F)

            dig_H5 =  self.getChar(cal3, 5)
            dig_H5 = (dig_H5 << 24) >> 20
            dig_H5 = dig_H5 | (self.getUChar(cal3, 4) >> 4 & 0x0F)

            dig_H6 =  self.getChar(cal3, 6)

            # Wait in ms (Datasheet Appendix B: Measurement time and current calculation)
            wait_time = 1.25 + (2.3 * self.OVERSAMPLE_TEMP) + ((2.3 * self.OVERSAMPLE_PRES) + 0.575) + (
                        (2.3 * self.OVERSAMPLE_HUM) + 0.575)
            time.sleep(wait_time / 1000)  # Wait the required time

            # Read temperature/pressure/humidity
            data = self.env_bus.read_i2c_block_data(addr, self.REG_DATA, 8)
            pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
            temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
            hum_raw = (data[6] << 8) | data[7]

            # Refine temperature
            var1 = ((((temp_raw >> 3) - (dig_T1 << 1))) * (dig_T2)) >> 11
            var2 = (((((temp_raw >> 4) - (dig_T1)) * ((temp_raw >> 4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
            t_fine = var1 + var2
            temperature = float(((t_fine * 5) + 128) >> 8);

            # Refine pressure and adjust for temperature
            var1 = t_fine / 2.0 - 64000.0
            var2 = var1 * var1 * dig_P6 / 32768.0
            var2 = var2 + var1 * dig_P5 * 2.0
            var2 = var2 / 4.0 + dig_P4 * 65536.0
            var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
            var1 = (1.0 + var1 / 32768.0) * dig_P1
            if var1 == 0:
                pressure = 0
            else:
                pressure = 1048576.0 - pres_raw
                pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
                var1 = dig_P9 * pressure * pressure / 2147483648.0
                var2 = pressure * dig_P8 / 32768.0
                pressure = pressure + (var1 + var2 + dig_P7) / 16.0

            # Refine humidity
            humidity = t_fine - 76800.0
            humidity = (hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (dig_H2 / 65536.0 * (
                        1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
            humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
            if humidity > 100:
                humidity = 100
            elif humidity < 0:
                humidity = 0

            self.temperature =temperature/ 100.0
            self.pressure=pressure / 100.0
            self.humidity=humidity

'''

if __name__ == "__main__":
    fle_bme2825 = BME285_Class()
    #print("%.2f"%fle_mpu6050.Ax)
    while True:
        # 打印出MPU相关信息
       
       fle_bme2825.readBME280All()

       print("Temperature: %.2f" %fle_bme2825.temperature,"C;Pressure : %.2f" %fle_bme2825.pressure,
             "hPa;Humidity : %.2f" %fle_bme2825.humidity)
       time.sleep(1)  # 延时10ms
'''



