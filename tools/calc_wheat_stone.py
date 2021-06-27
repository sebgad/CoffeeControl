#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 20 20:59:49 2021

@author: sebastian
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np



res_adc_bit = 16

f = open(r"C:/Programmieren/Arduino/coffee_ctrl_main/tools/PT_1000_tabelle.csv", "r")
lst_data = []

for i, line in enumerate(f):
    if i>0:
        lst_split = line.split(",")
        t_start = int(lst_split[0])
        for n in range(10):
            lst_data.append([t_start + n, float(lst_split[n+1])])
f.close()

df = pd.DataFrame(lst_data, columns = ["Temp", "Ohm"])

i_R1 = 1295
i_R2 = 1294
i_R3 = 1294
i_R4 = df['Ohm']
vcc =3.3


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








### TODO is this still needed /correct?

# for f_uv in [3.3]:
#     df['Umess_U0_%sV' % f_uv] = (df['Ohm'] / (df['Ohm'] + i_R2) - i_R3/(i_R3+i_R4)) * f_uv
#     f_volt_per_digit = f_uv / 2**res_adc_bit
#     df.plot(x='Temp', y='Umess_U0_%sV' % f_uv, ax=ax[0])


#     for f_u_res in [6.144, 4.096, 2.048, 1.024, 0.512, 0.256]:
#         f_vol_total = (2*f_u_res)/(2**(res_adc_bit-1)) # v/bit
#         df['Umess_U0_%sV_Res_%s' % (f_uv,f_u_res)] = df['Umess_U0_%sV' % f_uv].diff() / f_vol_total
#         df.plot(x='Temp', y='Umess_U0_%sV_Res_%s' % (f_uv,f_u_res), ax=ax[1])


# lst_factors = []

# for f_temp in np.arange(0, 180, 20):
#     idx = (df['Temp'] - f_temp).abs().argmin()
#     lst_factors.append([df['Temp'][idx], df['Umess_U0_3.3V'][idx]])

# print(lst_factors)


# ax[1].set_xlim(0,150)
# ax[0].grid()
# ax[1].grid()
# #ax[0].set_ylim(0.5,2.5)
# #ax[1].set_ylim(1000,2000)
# ax[0].set_ylabel('Umess (V)')
# ax[1].set_ylabel('digits per °C')
# ax[1].set_xlabel('Temp (°C)')
# #plt.tight_layout()
