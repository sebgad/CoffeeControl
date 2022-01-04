#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 20 19:13:36 2021

@author: sebgad
"""
import pandas as pd
import scipy.signal
import matplotlib.pyplot as plt
import numpy as np

lstFname = []

obj_f, ax = plt.subplots(nrows=2, sharex=True)
ax_pwm = ax[0].twinx()
f_target = 90.0
ax[1].axhline(y=0, color='black', linestyle='dashdot')

for strFname in lstFname:
    strFname_wo_ext = os.path.basename(strFname).split(".")[0]

    # read in csv
    df = pd.read_csv(strFname, skiprows=2, index_col=0)

    # apply butter bandpass filter / LPZ filter with no phase shift

    # filter coefficients
    # N=8: Filter order / amount of coefficients
    # Wn=0.075: Cutoff Frequency as a factor of the Nyquist frequency
    f_b, f_a = scipy.signal.butter(N=10, Wn=0.1)
    # bandpass forward+backward to eleminate phase shift
    df['TempSmoothed'] = scipy.signal.filtfilt(f_b, f_a, df['Temperature'])

    # create plot objects
    df.plot(y='Temperature', legend=False, ax=ax[0], label=strFname_wo_ext+"_Temp")

    df.plot(y='TargetPWM', ax=ax_pwm, legend=False, label=strFname_wo_ext+"_TargetPWM")

    df['TemperatureDeviation'] = f_target - df['TempSmoothed']
    df.plot(y='TemperatureDeviation', label=strFname_wo_ext+"_TempDiffTgt", ax=ax[1], legend=False)

ax[0].grid(True)
ax[0].set_ylim(0, 130)
ax_pwm.set_ylim(0, 260)

ax[0].set_yticks(np.arange(0,135,5))
ax_pwm.set_yticks(np.arange(0,265,10))
ax[0].set_ylabel('Temperature')
ax_pwm.set_ylabel('TargetPWM')

ax[1].set_ylim(-10, 10)
ax[1].grid(True)
ax[1].set_yticks(range(-10,11))
ax[1].set_ylabel("Temperature Deviation to Target")


obj_f.legend(loc='lower right', ncol=4)
plt.tight_layout()
obj_f.subplots_adjust(hspace=0.05)

