#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 19 13:39:41 2022

@author: sebgad

Curve fit for pt equitation
"""

import pandas as pd
import os
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

path = r"/home"
lst_df = []
f_start_time = 0.
f_start_temp = 0.

for i in range(9):
    fname = r"20220105_TS_step_17_pwm_%d_of_9.csv" % (i+1)
    df = pd.read_csv(os.path.join(path, fname), skiprows=2, index_col=0)
    f_delta_time = df.index[0] - f_start_time
    df.index -= f_delta_time
    
    if i>0:
        f_delta_temp = df['Temperature'].iloc[0] - f_start_temp
        df['Temperature'] -= f_delta_temp
        
    f_start_temp = df['Temperature'].iloc[-1]
    f_start_time = df.index[-1] + 0.45
    lst_df.append(df)

df = pd.concat(lst_df)

# PT1 equitation
def h_t_pt1(t, K, T):
    return K*(1-np.exp(-t/T))+df['Temperature'].iloc[0]

# PT2 equitation
def h_t_pt2(t, K, d, T):
    term1 = 1/np.sqrt(1-np.power(d,2))*np.exp(-d/T*t)
    term2 = np.cos(np.sqrt(1-np.power(d, 2))/T*t - np.pi - np.arctan(-d/np.sqrt(1-np.power(d,2))))
    return K*(1+term1*term2) + df['Temperature'].iloc[0]

popt_pt2, pcov_pt2 = curve_fit(h_t_pt2, df.index, df['Temperature'], 
                               bounds=([0.01, 0.01, 0.01], [100, 0.999, 9000]))

popt_pt1, pcov_pt1 = curve_fit(h_t_pt1, df.index, df['Temperature'], 
                               bounds=([0.01, 0.01], [100, 9000]))

plt.plot(df.index, df['Temperature'], 'b-', label='Original Data')

plt.plot(df.index, h_t_pt2(df.index, *popt_pt2), 'g--',
         label='PT2 fit: K=%5.3f, d=%5.3f, T=%5.3f' % tuple(popt_pt2))

plt.plot(df.index, h_t_pt1(df.index, *popt_pt1), 'y--',
         label='PT1 fit: K=%5.3f, T=%5.3f' % tuple(popt_pt1))

plt.legend()
plt.grid()
plt.ylabel('Temperature °C')
plt.xlabel('Time s')