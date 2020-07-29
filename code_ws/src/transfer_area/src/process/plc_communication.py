#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import threading
import time
import rospy
import copy

from area_common import ServiceNode, RunService
from car_scanner.msg import CarInfo
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

class PLC_Communication(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self)
        self.setRate(1) # no delay
        self.socketAddress = ('127.0.0.1', 31500)
        self.socketHandel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socketHandel.connect(self.socketAddress)
        print("socket connect successful!")

        self.lock_rx_car_info = threading.Lock()
        self.car_info_data = CarInfo()
        self.Subscriber("car_info", CarInfo, self.rx_car_info)
        self.car_scanner_successful = False
    
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
            recv_len = int(rx_data[14:18], 16) + 18
            if recv_len == len(rx_data):
                print (int(data_len,16))
                recv_data=[3]
                for i in range((recv_len-22)/4):
                    recv_data[i] = int(rx_data[22+i*4:26+i*4],16)
                print('receive data:', recv_data)
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


    def loop(self):
        if self.car_scanner_successful == True:
            wb_data = self.float2str(self.car_info_data.length_wheelbase*1000)
            ret = self.write_data(D_UNIT, '0000', '0001', wb_data)
            if ret==True:
                print ("write data successful!")
            else:
                print ("write data failed!")
            self.car_scanner_successful = False

            rxd = self.read_data(D_UNIT, '0000', '0003')
            print (rxd)
            

    def cleanup(self):
        self.socketHandel.close()



if __name__ == '__main__':
    time.sleep(5)
    RunService(PLC_Communication)