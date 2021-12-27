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

strFname = r"rancilio_silvia_stock_measurement.csv"

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
obj_f, lst_ax = plt.subplots(nrows=2, sharex=True)
df.plot(y=['Temperature', 'TempSmoothed'], ax=lst_ax[0])

# differential
df['TempSmoothedDiff^1'] = df['TempSmoothed'].diff(1)
df['TempSmoothedDiff^2'] = df['TempSmoothed'].diff(2)

df.plot(y=['TempSmoothedDiff^1'], ax=lst_ax[1])
df.plot(y=['TempSmoothedDiff^2'], ax=lst_ax[1])

# calculate regression (1st order) between 120s and 170s
obj_idx = (df.index>120) & (df.index<170)

# coefficients of the regression line
lst_coeffs = np.polyfit(df[obj_idx].index, df[obj_idx]['Temperature'], deg=1)

# x points for plot regression line
xp = np.arange(0, 275, 5)
lst_ax[0].plot(xp, np.polyval(lst_coeffs, xp))
# df[idx].plot(x='Time', y='Temperature', ax=ax)

lst_ax[0].grid(True)
lst_ax[1].grid(True)
lst_ax[0].set_yticks(range(0,120,10))
lst_ax[0].set_xlim(0, 400)
lst_ax[0].set_ylabel('Temperature')
