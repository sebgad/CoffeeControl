#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 20 20:59:49 2021

@author: sebastian
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

i_R1 = 1000 # PT1000
i_R2 = 1310
i_R3 = 1310
i_R4 = 1310

res_adc_bit = 16

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
f, ax = plt.subplots(nrows=2, sharex=True, tight_layout=True)

for f_uv in [3.3]:
    df['Umess_U0_%sV' % f_uv] = (df['Ohm'] / (df['Ohm'] + i_R2) - i_R3/(i_R3+i_R4)) * f_uv
    f_volt_per_digit = f_uv / 2**res_adc_bit
    df.plot(x='Temp', y='Umess_U0_%sV' % f_uv, ax=ax[0])


    for f_u_res in [6.144, 4.096, 2.048, 1.024, 0.512, 0.256]:
        f_vol_total = (2*f_u_res)/(2**(res_adc_bit-1)) # v/bit
        df['Umess_U0_%sV_Res_%s' % (f_uv,f_u_res)] = df['Umess_U0_%sV' % f_uv].diff() / f_vol_total
        df.plot(x='Temp', y='Umess_U0_%sV_Res_%s' % (f_uv,f_u_res), ax=ax[1])


lst_factors = []

for f_temp in np.arange(0, 180, 20):
    idx = (df['Temp'] - f_temp).abs().argmin()
    lst_factors.append([df['Temp'][idx], df['Umess_U0_3.3V'][idx]])

print(lst_factors)


ax[1].set_xlim(0,150)
ax[0].grid()
ax[1].grid()
#ax[0].set_ylim(0.5,2.5)
#ax[1].set_ylim(1000,2000)
ax[0].set_ylabel('Umess (V)')
ax[1].set_ylabel('digits per °C')
ax[1].set_xlabel('Temp (°C)')
#plt.tight_layout()
