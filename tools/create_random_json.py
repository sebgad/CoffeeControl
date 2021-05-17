#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 15 17:45:07 2021

@author: sebastian
"""
import random

f_out = r"/home/sebastian/out.txt"
f= open(f_out, "w")

i_n_points = 1*10e2
# Key: Name, values: [min, max, precision]
dct_signals = {'pressure': [0, 8, 3],
               'temperature': [20, 140, 2]}


json_stream = "{\"time\":["
f.writelines("{\"time\":[")
i_len_dct = len(dct_signals)

for i in range(10000, 10000 + int(i_n_points)):
    f.writelines("\"%d\",\r\n" % i)

f.writelines("\"%d\"],\r\n" % i)

i_dct = 1
for str_key, lst_values in dct_signals.items():
    f.writelines("\"%s\":[" % str_key)

    for i in range(int(i_n_points)):
        f_random =round(random.randint(0, 8) + random.random(), lst_values[-1])
        f.writelines("\"%s\",\r\n" % f_random)

    f_random =round(random.randint(0, 8) + random.random(), lst_values[-1])
    if i_dct==i_len_dct:
        f.writelines("\"%s\"]\r\n" % f_random)
    else:
        f.writelines("\"%s\"],\r\n" % f_random)
    i_dct+=1

f.writelines("}")
f.close()
