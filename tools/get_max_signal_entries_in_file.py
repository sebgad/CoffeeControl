#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May 16 11:26:34 2021

@author: sebastian
"""

import random

f_target_size_bytes = 1000e3 #bytes
f_out_temp = r"/home/sebastian/temp.csv"

i_interval = 500
i_max_iter = 5000
i_time_offset_begin = 1e04
f_store_interval = 0.5 #s

# Key: Name, values: [min, max, precision]
dct_signals = {'pressure': [0, 8, 2],
               'temperature': [20, 140, 1],
               'pumpOn': [0, 1, 0],
               'HeatOn': [0, 1, 0]}

i = 0
i_size = 0
i_time_offset = 0
f = open(f_out_temp, "w")
while (i<i_max_iter) & (i_size < f_target_size_bytes):
    for i in range(i_interval):
        str_line = "%d," % (i_time_offset + i + i_time_offset_begin)
        for str_signal_name, lst_values in dct_signals.items():
            f_value = random.randint(lst_values[0],lst_values[1]) + random.random()
            str_line += "%s," % (round(f_value, lst_values[2]))
        str_line = str_line[:-1]
        f.writelines(str_line + "\r\n")

    i_size = f.tell()
    i_time_offset += i_interval

print("Maximum Byte Size: %s, Maximum Entries: %s" % (i_size, i_time_offset))
print("Store Interval: %ss, Maximum Measurement Length:%.2fmin" % (f_store_interval,
                                                                   i_time_offset*f_store_interval/60))
f.close()
