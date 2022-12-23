import numpy as np


class AccChannel(object):
    def __init__(self):
        self.acc = 0.0
        self.acc_sum = [0.0, 0.0]
        self.zero = 0.0
        self.speed = 0.0
        self.position = 0.0
        self.filter_buffer = list()
        self.filter_size = 4
        self.stall_time = 0.0
        self.max_stall_time = 1.0
        self.min_acc = 0.1

    def accumulate(self, acc, time):
        speed = self.speed + (self.acc + acc) * time * 0.5
        self.position += (self.speed + speed) * time * 0.5
        self.acc = acc
        self.speed = speed

    def calibrate(self, acc):
        self.acc_sum[0] += acc
        self.acc_sum[1] += 1.0
        self.zero = self.acc_sum[0] / self.acc_sum[1]
        return self.zero, self.acc_sum[1]

    def filter(self, acc):
        if len(self.filter_buffer) >= self.filter_size:
            self.filter_buffer.pop(0)
        self.filter_buffer.append(acc)
        return sum(self.filter_buffer) / len(self.filter_buffer)

    def refresh(self, acc, time):
        acc = self.filter(acc) - self.zero
        self.stall_time = 0.0 if math.fabs(acc) > self.min_acc else self.stall_time + time
        if self.stall_time > self.max_stall_time:
            acc = 0.0
            self.speed = 0.0
        self.accumulate(acc, time)