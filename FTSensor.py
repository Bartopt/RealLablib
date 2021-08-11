# from NetFT import *
import numpy as np
import socket
from multiprocessing import Process


class ATISensor():
    def __init__(self, ip="192.168.50.101"):
        self.sensor = Sensor(ip)
        self.biasValue = None
        self.BiasSensor()

    def BiasSensor(self):
        self.biasValue = self.GetValue(raw=True)

    def GetValue(self, n=10, raw=False):
        if raw:
            return np.around(np.array(self.sensor.tare(n)) / 1000000)
        else:
            return np.around(np.array(self.sensor.tare(n)) / 1000000) - self.biasValue


class FT300Sensor():
    def __init__(self, ip='192.168.1.102', port=63351):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((ip, port))
        self.start_threading()  # use to clear the buffer to make the force data is latest
        self.biasValue = None
        self.BiasSensor()

    def ReadValue(self, n=10):
        ret = []
        for _ in range(n):
            data = self._receive()
            # print(len(data))
            data = data.replace("b\'(", "")
            data = data.replace(")\'", "")
            data = data.replace(",", " ")
            data = data.split()
            try:
                data = list(map(float, data))
                ret.append(data)
            except:
                print('receiving wrong data')

        ret = np.array(ret)
        ave_ret = np.mean(ret, axis=0)

        return ave_ret

    def BiasSensor(self):
        self.biasValue = self.GetValue(raw=True)

    def GetValue(self, n=10, raw=False):
        if raw:
            return np.around(np.array(self.ReadValue(n)))
        else:
            return np.around(np.array(self.ReadValue(n))) - self.biasValue

    def _receive(self):
        return str(self.s.recv(1024))

    def _callback(self):
        """
        To record the data by a single thread
        :return:
        """
        while self.stream:
            self._receive()

    def start_threading(self):
        """

        :param data:
        :return:
        """
        self.stream = True
        self.thread = Process(target=self._callback, args=())
        self.thread.daemon = True
        # self._send()
        self.thread.start()

    def stop_threading(self):
        self.stream = False
        self._send(self.STOP_COMMAND)  # Stop receiving data
        self.thread.join()


if __name__ == '__main__':
    fts = FT300Sensor()

    while True:
        r = fts.GetValue()
        print(r)
