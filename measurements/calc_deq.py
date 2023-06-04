#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 29 21:07:16 2023

@author: sebastian
"""

import pandas as pd
import numpy as np
import os
import scipy
import matplotlib.pyplot as plt

def fitfunction(t, T1, T2, sigma_t):
    K = 103.25
    ret_value = (K-T2) * (1 - np.exp(-t/T1) ) * sigma_t + T2
    return ret_value

lstFname = ["20220105_TS_step_17_pwm_1_of_9.csv",
            "20220105_TS_step_17_pwm_2_of_9.csv",
            "20220105_TS_step_17_pwm_3_of_9.csv",
            "20220105_TS_step_17_pwm_4_of_9.csv",
            "20220105_TS_step_17_pwm_5_of_9.csv",
            "20220105_TS_step_17_pwm_6_of_9.csv",
            "20220105_TS_step_17_pwm_7_of_9.csv",
            "20220105_TS_step_17_pwm_8_of_9.csv",
            "20220105_TS_step_17_pwm_9_of_9.csv"]

path = r"."

lst_df =[]
last_time_end = 0.
p = 0.

for i, filename in enumerate(lstFname):
    file_path = os.path.join(path, filename)
    df_temp = pd.read_csv(file_path, skiprows=2)

    if i > 0:
        xoffset = (np.poly1d(p)-df_temp['Temperature'].iloc[0]).roots
        df_temp['Time'] += xoffset[0].real

    # Calculate differential and extrapolate x coordinate of next datapoint
    p = np.polyfit(df_temp['Time'].iloc[-50::], df_temp['Temperature'].iloc[-50::], deg=2)

    lst_df.append(df_temp)

    last_time_end = df_temp["Time"].iloc[-1]

f, ax = plt.subplots()
df= pd.concat(lst_df)

popt = scipy.optimize.curve_fit(fitfunction, df['Time'].values, df['Temperature'].values)
df['Optimized'] = df['Time'].apply(fitfunction, args=tuple(popt[0]))

df.plot(x='Time', y=['Temperature', 'Optimized'], ax=ax)
ax.grid(True)
