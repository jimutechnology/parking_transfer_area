#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import threading
import time
import rospy
import copy

from area_common import ServiceNode, RunService
from car_scanner.msg import CarInfo
from std_msgs.msg import Float64
'''
# 十六进制数据
TX_HEAD = [0x50, 0x00]
RX_HEAD = [0xD0, 0x00]
NETWORK_NUM = [0x00, 0x00]
PLC_NUM = [0xFF, 0xFF]
IO_NUM = [0x03, 0xFF]
STATION_NUM = [0x00, 0x00]
CPU_TIMER = [0x00, 0x01]
RX_END = [0x00, 0x00]

READ_COMMAND = [0x04, 0x01]
SUB_COMMAND = [0x00, 0x00]
'''

# 字符串数据
TX_HEAD = '5000'
RX_HEAD = 'D000'
NETWORK_NUM = '00'
PLC_NUM = 'FF'
IO_NUM = '03FF'
STATION_NUM = '00'
CPU_TIMER = '0010'
RX_END = '0000'

READ_COMMAND = '0401'
WRITE_COMMAND = '1401'

SUB_COMMAND = '0000'
M_UNIT = 'M*'
D_UNIT = 'D*'

FLAG_ADD = '009991'
DATA_ADD = '009990'

FLAG_LEN = '0001'
DATA_LEN = '0001'
class PLC_Communication(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self)
        self.setRate(50) # no delay
        self.socketAddress = ('192.168.13.41', 1100)
        self.socketHandel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socketHandel.connect(self.socketAddress)
        print("socket connect successful!")

        self.lock_rx_car_info = threading.Lock()
        self.car_info_data = CarInfo()
        self.Subscriber("car_info", CarInfo, self.rx_car_info)
        self.car_scanner_successful = False

        self.length_wheelbase_buffer = []
        self.data_length_wheelbase_filtered = Float64()
        # For debug purpose only, filtered output, which PLC requests 
        self.pub_length_wheelbase = self.Publisher("length_wheelbase_filtered", Float64, queue_size=5)
    
    def rx_car_info(self, data):
        with self.lock_rx_car_info:
            self.car_info_data = copy.deepcopy(data)
            self.car_scanner_successful = True

    def read_data(self, unit, start_end, data_len):
        message_command = READ_COMMAND + SUB_COMMAND + unit + start_end + data_len
        command_len = '0018'
        output_data = TX_HEAD + NETWORK_NUM + PLC_NUM + IO_NUM + STATION_NUM + command_len + CPU_TIMER + message_command
        self.socketHandel.send(str.upper(output_data))
        rx_data = self.socketHandel.recv(512)
        print (rx_data)
        recv_command = RX_HEAD + NETWORK_NUM + PLC_NUM + IO_NUM + STATION_NUM
        if recv_command == rx_data[0:14]:
            recv_len = int(rx_data[14:18], 16)
            if recv_len == len(rx_data)-18:
                recv_data=[]
                for i in range((recv_len-4)/4):
                    recv_data.append(int(rx_data[22+i*4:26+i*4],16))
                print('receive data successful')
                return recv_data
        print ('receive data failed')
        return ''
    
    def write_data(self, unit, start_end, data_len, payload):
        message_command = WRITE_COMMAND + SUB_COMMAND + unit + start_end + data_len + payload
        command_len = hex(int(data_len, 16) * 4 + 24)
        command_len = command_len[2:len(command_len)]
        command_len = command_len.zfill(4)
        output_data = TX_HEAD + NETWORK_NUM + PLC_NUM + IO_NUM + STATION_NUM + command_len + CPU_TIMER + message_command
        self.socketHandel.send(str.upper(output_data))
        read_data = self.socketHandel.recv(512)
        print (read_data)
        recv_command = RX_HEAD + NETWORK_NUM + PLC_NUM + IO_NUM + STATION_NUM + '00040000'
        if recv_command == read_data:
            return True
        else:
            return False
            
    def float2str(self, data):
        if data>65535:
            data = 65535
        elif data<0:
            data = 0
        a=hex(int(data))
        b=a[2:len(a)]
        c=b.zfill(4)
        return c

    # Read n samples of length_wheelbase and return median
    # This function is blocking
    def wait_for_length_wheelbase_reading(self, n_samples):
        self.length_wheelbase_buffer = []
        while (len(self.length_wheelbase_buffer) < n_samples):
            with self.lock_rx_car_info:
                if self.car_scanner_successful:
                    self.length_wheelbase_buffer.append(self.car_info_data.length_wheelbase)
                    self.car_scanner_successful = False
            time.sleep(0.01)
        
        self.length_wheelbase_buffer.sort()
        return self.length_wheelbase_buffer[n_samples / 2]


    def loop(self):
        if self.car_scanner_successful == True:
            rxd = self.read_data(D_UNIT, FLAG_ADD, FLAG_LEN)
            if rxd[0] == 1:
                length_wheel_base_filtered = self.wait_for_length_wheelbase_reading(5)
                wb_data = self.float2str(length_wheel_base_filtered*1000)
                ret = self.write_data(D_UNIT, DATA_ADD, DATA_LEN, wb_data)
                if ret==True:
                    print ("write data successful!")
                else:
                    print ("write data failed!")
                #ret = self.write_data(D_UNIT, FLAG_ADD, FLAG_LEN, self.float2str(0.0))
                self.car_scanner_successful = False

                # For debug purpose only
                self.data_length_wheelbase_filtered.data = self.wait_for_length_wheelbase_reading(5)
                self.pub_length_wheelbase.publish(self.data_length_wheelbase_filtered)

    def cleanup(self):
        self.socketHandel.close()



if __name__ == '__main__':
    time.sleep(5)
    RunService(PLC_Communication)