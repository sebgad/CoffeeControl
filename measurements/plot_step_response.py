# -*- coding: utf-8 -*-
"""
Created on Wed Jan  5 14:17:56 2022

@author: Tino
"""

import pandas as pd
import scipy.signal
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import os

lstFname = ["20220105_TS_step_17_pwm_1_of_9.csv",
            "20220105_TS_step_17_pwm_2_of_9.csv",
            "20220105_TS_step_17_pwm_3_of_9.csv",
            "20220105_TS_step_17_pwm_4_of_9.csv",
            "20220105_TS_step_17_pwm_5_of_9.csv",
            "20220105_TS_step_17_pwm_6_of_9.csv",
            "20220105_TS_step_17_pwm_7_of_9.csv",
            "20220105_TS_step_17_pwm_8_of_9.csv",
            "20220105_TS_step_17_pwm_9_of_9.csv"]

obj_f, ax = plt.subplots(nrows=1, sharex=True,figsize=(100,100),dpi=200)
ax_pwm = ax.twinx() 
f_target = 90.0
##ax[1].axhline(y=0, color='black', linestyle='dashdot')

all_df = []
time_shift = 0

end_value = 103.5

for strFname in lstFname:
    strFname_wo_ext = os.path.basename(strFname).split(".")[0]

    # read in csv
    df = pd.read_csv(strFname, skiprows=2)

    # apply butter bandpass filter / LPZ filter with no phase shift

    # filter coefficients
    # N=8: Filter order / amount of coefficients
    # Wn=0.075: Cutoff Frequency as a factor of the Nyquist frequency
    f_b, f_a = scipy.signal.butter(N=10, Wn=0.1)
    # bandpass forward+backward to eleminate phase shift
    df['TempSmoothed'] = scipy.signal.filtfilt(f_b, f_a, df['Temperature'])
    
    
    df['TimeShifted'] =df['Time']+time_shift
    time_shift= df['TimeShifted'].iloc[-1]
    
    all_df.append(df)

   

df_all = pd.concat(all_df)

df_all['EndValue']=end_value
df_all['63pctEndValue']=end_value*0.63

f_b, f_a = scipy.signal.butter(N=10, Wn=0.1)
df_all['TempAllSmoothed'] = scipy.signal.filtfilt(f_b, f_a, df_all['Temperature'])

f_temp_start = df_all['Temperature'].iloc[0]
l_filt4 = [f_temp_start]*4
l_filt8 = [f_temp_start]*8

l_temp_filt4 =[]
l_temp_filt8 =[]
for f_temp in df_all['Temperature']:
    l_filt4.pop(0)
    l_filt4.append(f_temp)
    l_filt8.pop(0)
    l_filt8.append(f_temp)
    l_temp_filt4.append(np.mean(l_filt4))
    l_temp_filt8.append(np.mean(l_filt8))
    
df_all['TempMean4']=l_temp_filt4
df_all['TempMean8']=l_temp_filt8

df_all.plot(y='Temperature',x='TimeShifted', legend=False, ax=ax, label="Temp", color= 'tab:blue')
df_all.plot(y='TempMean4',x='TimeShifted', legend=False, ax=ax, label="Temp", color= 'tab:red')
df_all.plot(y='TempMean8',x='TimeShifted', legend=False, ax=ax, label="Temp", color= 'tab:purple')
df_all.plot(y='TempAllSmoothed',x='TimeShifted', legend=False, ax=ax, label="Temp", color= 'k')
df_all.plot(y='EndValue',x='TimeShifted', legend=False, ax=ax, label="EndValue", color= 'tab:green')
df_all.plot(y='63pctEndValue',x='TimeShifted', legend=False, ax=ax, label="63pctEndValue", color= 'tab:green')
df_all.plot(y='TargetPWM',x='TimeShifted', legend=False, ax=ax_pwm, label="PWM", color= 'tab:red')
ax.grid(True)
ax.set_ylim(0, 130)
#ax_pwm.set_ylim(0, 260)

ax.set_yticks(np.arange(0,135,5))
ax.yaxis.set_minor_locator(ticker.MultipleLocator(2.5))
ax.yaxis.grid(True,which='minor',linestyle='--')
#set_minor(np.arange(0,135,2.5))
ax_pwm.set_yticks(np.arange(0,255,17))



ax.set_ylabel('Temperature / °C')
ax_pwm.set_ylabel('PWM [0,255]')
ax.set_xlabel('Time / s')

plt.tight_layout()
obj_f.subplots_adjust(hspace=0.05)
obj_f.autofmt_xdate(rotation=90)
plt.xticks(np.arange(0,time_shift,250))


ax.xaxis.set_minor_locator(ticker.MultipleLocator(50))
ax.xaxis.grid(True,which='minor',linestyle='--')





