#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 20 20:59:49 2021

@author: sebgad
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

res_adc_bit = 16

f = open(r"PT_1000_tabelle.csv", "r")
lst_data = []

for i, line in enumerate(f):
    if i>0:
        lst_split = line.split(",")
        t_start = int(lst_split[0])
        for n in range(10):
            lst_data.append([t_start + n, float(lst_split[n+1])])
f.close()

df = pd.DataFrame(lst_data, columns = ["Temp", "Ohm"])

i_R1 = 1298
i_R2 = 1298
i_R3 = 1298
i_R4 = df['Ohm']
vcc = 5.08

df['Umess'] = vcc * (i_R1*i_R4 - i_R3*i_R2)/((i_R1 + i_R2)*(i_R3 + i_R4))

idx = (df['Temp']>=0) & (df['Temp']<=160)
df=df[idx]

pol = np.polyfit(df['Umess'], df['Temp'], deg = 2)
df['Temp_reg']= np.polyval(pol, df['Umess'])

f, ax = plt.subplots(1, 1, sharex=True, sharey=True, squeeze= True)

df.plot(x='Umess', y='Temp_reg', ax= ax)
df.plot(x='Umess', y='Temp', ax= ax)

print("const float fPt1000CoeffX2 = %f;"%pol[0])
print("const float fPt1000CoeffX1 = %f;"%pol[1])
print("const float fPt1000CoeffX0 = %f;"%pol[2])

# Create c++ array string for copy&paste
strOut = "{\r\n"
for i, row in df.iterrows():
    strOut+= "{%.17f, %.1f},\r\n" % (row['Umess'], row['Temp'])
strOut +="}"

fobj = open("array_cpp.txt", "w")
fobj.writelines(strOut)
fobj.close()
