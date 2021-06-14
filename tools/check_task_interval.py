#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 21 12:08:21 2021

@author: sebastian
"""

import matplotlib.pyplot as plt
import numpy as np


i_task_long = 100
i_length = 10000

lst_delta_min = []
lst_delta_bins = []
for i_task_short in range(85, 95):
    arr_task_long = (np.ones((i_length, )) * i_task_long).cumsum()
    arr_task_short = (np.ones((i_length, )) * i_task_short).cumsum()

    # Get maximum difference between two tasks in the end
    i_delta_max = arr_task_long[-1] - arr_task_short[-1]

    res = arr_task_long[None, :] - arr_task_short[:, None]
    abs_res = np.abs(res)
    idx_sum = (abs_res.astype(int) == 0).sum()
    lst_delta_min.append([i_task_short, idx_sum])
    lst_delta_bins.append(np.bincount(abs_res.astype(int).flatten())[:20])

arr_delta_bins = np.array(lst_delta_bins).squeeze()
arr_delta = np.array(lst_delta_min)
print(arr_delta)

f, ax = plt.subplots()
ax.bar(x=arr_delta[:,0], height=arr_delta[:,1]/i_length*100, width=0.8, align="center")
ax.set_xticks(arr_delta[:,0])
ax.set_ylabel('Maximum number of same interrupt time (in percent to maximum length)')
ax.set_xlabel('short task interval (ms)')
ax.set_title("long task interval: %sms, Period: %ss" % (i_task_long,
                                                        i_task_long*i_length/1000))
plt.show()

f, ax = plt.subplots()
for i, arr_bins in enumerate(lst_delta_bins):
    ax.plot(np.arange(20), arr_bins/i_length*100, '-o', label='%sms task' % arr_delta[i,0])

ax.set_ylabel('Maximum number of same interrupt time (in percent)')
ax.set_xlabel('difference between long and short task (ms)')
ax.set_xticks(range(21))
ax.set_title("long task interval: %sms, Period: %ss" % (i_task_long,
                                                        i_task_long*i_length/1000))
ax.legend()
plt.show()
