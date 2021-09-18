#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 26 21:54:46 2021

@author: sebastian
"""

import pandas as pd
import numpy as np


f = open(r"/home/sebastian/Businessprojekte/EspressoMaschineErweiterung/Berechnungen/PT1000/PT_1000_tabelle.csv", "r")
lst_data = []

for i, line in enumerate(f):
    if i>0:
        lst_split = line.split(",")
        t_start = int(lst_split[0])
        for n in range(10):
            lst_data.append([t_start + n, float(lst_split[n+1])])
f.close()

df = pd.DataFrame(lst_data, columns = ["Temp", "Ohm"])
i_R1 = 1299
i_R2 = 1299
i_R3 = 1298
i_R4 = df['Ohm']
vcc = 3.3

df['Umess'] = vcc*(i_R1*i_R4-i_R3*i_R2)/ ((i_R1+i_R2)*(i_R3+i_R4))

## output
idx = (df['Temp']>=0) & (df['Temp']<=200)
df = df[idx]
p = np.polyfit(df['Umess'], df['Temp'], deg=2, full=False)
df['Temp_regression'] = np.polyval(p, df['Umess'])
df.plot(x='Umess', y=['Temp', 'Temp_regression'])

# print every 5 degree
for index, row in df.iloc[::5].iterrows():
    print("{%s, %s}," % (row['Umess'], row['Temp']))
