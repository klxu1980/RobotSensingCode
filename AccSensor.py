import math
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
import numpy as np
import serial
import time


class AccChannel(object):
    def __init__(self):
        self.acc = 0.0                    # 上一采样周期的加速度
        self.zero = 0.0                   # 加速度零点
        self.speed = 0.0                  # 速度
        self.position = 0.0               # 位置
        self.filter_k = 0.95              # 一阶滞后滤波系数
        self.cali_k = 0.995               # 零点标定一阶之后滤波系数
        self.stall_enabled = True         # 强制零速
        self.stall_time = 0.0             # 零速时间
        self.max_stall_time = 1.0         # 最大零速时间
        self.min_acc = 0.1                # 最小加速度

    def accumulate(self, acc, time):
        """
        通过加速度梯形积分，得到速度和位置
        :param acc:
        :param time:
        :return:
        """
        speed = self.speed + (self.acc + acc) * time * 0.5
        self.position += (self.speed + speed) * time * 0.5
        self.acc = acc
        self.speed = speed
        return self.speed, self.position

    def calibrate(self, acc):
        """
        标定加速度传感器零点
        :param acc:
        :return:
        """
        zero = self.zero * self.cali_k + acc * (1.0 - self.cali_k)
        delta = math.fabs(zero - self.zero)
        self.zero = zero
        return self.zero, delta

    def filter(self, acc):
        """
        对输入加速度进行处理，减去传感器零点，并进行一阶滞后滤波
        如果加速度小于死区阈值，则置为0
        :param acc: 传感器采样获得的原始加速度信号
        :return: 经过滤波后的加速度
        """
        acc -= self.zero
        acc = self.acc * self.filter_k + acc * (1.0 - self.filter_k)
        if math.fabs(acc) < self.min_acc:
            acc = 0.0
        return acc

    def refresh(self, acc, time):
        """
        更新速度、位置
        :param acc:
        :param time:
        :return:
        """
        # 如果加速度在一定时间(stall_time)内持续为0，则令速度为0
        acc = self.filter(acc)
        if self.stall_enabled:
            if acc == 0.0:
                self.stall_time += time
                if self.stall_time > self.max_stall_time:
                    self.speed = 0.0
            else:
                self.stall_time = 0.0
        self.accumulate(acc, time)


class AccSensor(object):
    def __init__(self, comm_port):
        # baud = int(input('please input baudrate(115200 for JY61 or 9600 for JY901):'))
        self.seriel = serial.Serial(comm_port, 9600, timeout=0.5)  # 串口通信
        self.seriel_buffer = []                                    # 串口通信字节缓冲
        self.interval = 0.075                                      # 采样间隔，单位秒
        self.a = None                                              # 实测加速度
        self.w = None                                              # 实测角速度
        self.angle = None                                          # 实测角度

        # x, y, z方向的加速度积分处理
        self.sensor_x = AccChannel()
        self.sensor_y = AccChannel()
        self.sensor_z = AccChannel()

    @staticmethod
    def get_float_values(data_hex, k):
        """
        从串行端口读取到的原始报文中读取三个短整型数，转换为浮点数。乘以量程，得到加速度、角速度或角度值。
        :param data_hex: 原始报文数据
        :param k: 量程
        :return: 转换结果
        """
        # 将报文中的字节拼接为短整型数，转换为0.0~2.0之间的浮点数
        v1 = (data_hex[1] << 8 | data_hex[0]) / 32768.0
        v2 = (data_hex[3] << 8 | data_hex[2]) / 32768.0
        v3 = (data_hex[5] << 8 | data_hex[4]) / 32768.0

        # 转换为-1.0~1.0之间的浮点数，乘以量程，得到结果
        v1 = (v1 - 2.0) * k if v1 > 1.0 else v1 * k
        v2 = (v2 - 2.0) * k if v2 > 1.0 else v2 * k
        v3 = (v3 - 2.0) * k if v3 > 1.0 else v3 * k

        return v1, v2, v3

    def read_from_sensor(self):
        """
        从串口中获取传感器输出数据
        :return: True: 读取成功, False: 读取失败
        """
        while True:
            # 从串口中读取一个字节
            # 如果超时读不到，则返回False
            pkg = self.seriel.read(1)
            if len(pkg) == 0:
                self.seriel_buffer.clear()
                return False

            # 将读到的字节写入缓冲区以便于后续处理
            # 传感器上电后会持续上传数据报文，程序刚启动时，很可能收到的报文不完整。
            # 因此，需要确保缓冲区的头两个字节分别是0x55、0x51，即一个包含了加速度、角速度和角度数据的完整报文的起始字节。
            # 当接收到一个33字节完整报文后，检查加速度、角速度和角度数据的校验和是否正确，之后从中提取数据。
            self.seriel_buffer.append(pkg[0])
            if len(self.seriel_buffer) == 1 and self.seriel_buffer[0] != 0x55 or \
                    len(self.seriel_buffer) == 2 and self.seriel_buffer[1] != 0x51:
                self.seriel_buffer.clear()
            elif len(self.seriel_buffer) == 33:
                if sum(self.seriel_buffer[0:10]) & 0x0FF == self.seriel_buffer[10] and \
                        sum(self.seriel_buffer[11:21]) & 0x0FF == self.seriel_buffer[21] and \
                        sum(self.seriel_buffer[22:32]) & 0x0FF == self.seriel_buffer[32]:
                    ax, ay, az = self.get_float_values(self.seriel_buffer[2:8], k=16.0 * 9.8)
                    wx, wy, wz = self.get_float_values(self.seriel_buffer[13:19], k=2000.0)
                    tx, ty, tz = self.get_float_values(self.seriel_buffer[24:30], k=180.0)
                    self.a = (ax, ay, az)
                    self.w = (wx, wy, wz)
                    self.angle = (tx, ty, tz)
                self.seriel_buffer.clear()
                return True

    def show_sensor_data(self):
        #print("sample time", time.asctime(time.localtime(self.time_stamp)))
        """
        if self.lst_time_stamp is not None:
            print("sample time", round((self.time_stamp - self.lst_time_stamp) * 1000))
            print("acc = (%8.5f, %8.5f, %8.5f)" % self.a)
        """
        print("acc = (%8.5f, %8.5f), spd = (%8.5f, %8.5f), pos = (%8.5f, %8.5f)" %
              (self.sensor_x.acc, self.sensor_y.acc, self.sensor_x.speed, self.sensor_y.speed, self.sensor_x.position, self.sensor_y.position))

    def calibrate(self):
        avg_x, times = self.sensor_x.calibrate(self.a[0])
        avg_y, _ = self.sensor_y.calibrate(self.a[1])
        avg_z, _ = self.sensor_z.calibrate(self.a[2])
        return avg_x, avg_y, avg_z, times

    def refresh(self):
        self.read_from_sensor()
        """
        self.sensor_x.refresh(self.a[0], self.interval)
        self.sensor_y.refresh(self.a[1], self.interval)
        self.sensor_z.refresh(self.a[2], self.interval)
        """


sensor = AccSensor("com4")

"""
cur_time = 0.0
dtime = 0.01
freq = 0.1
position_x = list()
position_y = list()


fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(autoscale_on=False, xlim=(-10, 10), ylim=(-10, 10))
ax.grid()
trace, = ax.plot([], [])
"""

"""
def update(i):
    cur_time = i * dtime
    acc_x = math.sin(cur_time * freq * math.pi * 2.0)
    acc_y = math.sin(cur_time * freq * math.pi * 4.0)
    sensor.refresh(acc_x, acc_y, dtime)
    position_x.append(sensor.sensor_x.position)
    position_y.append(sensor.sensor_y.position)
    if len(position_x) > 100:
        position_x.pop(0)
        position_y.pop(0)
    trace.set_data(position_x, position_y)


    x = np.linspace(0, 1, 100)
    y = np.sin(x + i * 0.01) * 10
    trace.set_data(x, y)


    return trace,
"""


#ani = animation.FuncAnimation(fig, update, frames=None, interval=10, blit=True)
#plt.show()

if __name__ == '__main__':
    """
    _, _, _, times = sensor.calibrate()

    time1 = time.time()
    while times < 100:
        sensor.refresh()
        _, _, _, times = sensor.calibrate()
    time2 = time.time()
    ms = round((time2 - time1) * 1000)
    print("ms = %f" % ms)
    print("sample time", time.asctime(time.localtime(time2 - time1)))

    while True:
        sensor.refresh()
        sensor.show_sensor_data()
    """
    acc_list = list()
    times = 0
    while times < 50:
        sensor.refresh()
        acc_list.append(sensor.a[0])
        acc_list.append(sensor.a[1])
        times += 1
    print("Cali finished")

    while times < 500:
        sensor.refresh()
        acc_list.append(sensor.a[0])
        acc_list.append(sensor.a[1])
        times += 1

    np.save("加速度.npy", np.array(acc_list))

    #plt.show()

    """
    sensor = AccSensor()
    time = 0.0
    dtime = 0.01
    freq = 0.1
    while True:
        acc_x = math.sin(time * freq * math.pi * 2.0)
        acc_y = math.sin(time * freq * math.pi * 4.0)
        time += dtime
        sensor.refresh(acc_x, acc_y, dtime)
        print(sensor.sensor_x.position)
    """

