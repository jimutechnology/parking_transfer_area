#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import time

address = ('127.0.0.1', 31500)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # s = socket.socket()
s.bind(address)
s.listen(5)

ss, addr = s.accept()
print 'got connected from',addr
while True:
    try:
        ra = ss.recv(512)
        print ra
        if ra[22]=='1':
            ss.send('D00000FF03FF0000040000')
        elif ra[22]=='0':
            ss.send('D00000FF03FF00000800001234')
        else:
            print("receive data error")
        time.sleep(1)
    except Exception,e:
        ss.close()
        s.close()
        break
