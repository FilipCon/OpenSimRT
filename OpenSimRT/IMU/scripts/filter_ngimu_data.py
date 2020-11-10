##
import os
import numpy as np
import pandas as pd
import opensim as osim
import matplotlib.pyplot as plt
import ahrs
import math
from matplotlib.backends.backend_pdf import PdfPages
from scipy import signal
import scipy.integrate as it
from sklearn.preprocessing import normalize
from scipy.spatial.transform import Rotation

# data
subject_dir = os.path.abspath('../../../data/gait1992/experimental_data')
csv_on_file = os.path.join(subject_dir, '../real_time/avp.csv')

# read csv
csv = pd.read_csv(csv_on_file, skiprows=4)

time = np.array(csv['time'])
pos_rt = np.array([csv['p_1'], csv['p_2'],
                   csv['p_3']]).transpose()
vel_rt = np.array([csv['v_1'], csv['v_2'],
                   csv['v_3']]).transpose()
acc_rt = np.array([csv['a_1'], csv['a_2'],
                   csv['a_3']]).transpose()

def integrate(x):
    vel = np.zeros(acc.shape)
    for i in range(1, acc.shape[0]):
        vel[i, :] = vel[i-1, :] + acc[i, :] * (1/frameRate)
acc = acc_rt

# FILTER
frameRate = 40

# filter cutoff freq
filtOrder = 1
# acc_LP_freq = 5
# acc_HP_freq = 0.001
vel_HP_freq = 0.1
pos_HP_freq = 0.1

# acceleration
# b, a = signal.butter(filtOrder, (2 * acc_LP_freq) /
#                      frameRate, 'low', analog=False)
# d, c = signal.butter(filtOrder, (2 * acc_HP_freq) /
#                      frameRate, 'high', analog=False)
# acc_f = signal.lfilter(b, a, signal.lfilter(d, c, acc))

# velocity
vel = np.zeros(acc.shape)
for i in range(1, acc.shape[0]):
    vel[i, :] = vel[i-1, :] + acc[i, :] * (1/frameRate)
b, a = signal.butter(filtOrder, (2 * vel_HP_freq) /
                     frameRate, 'high', analog=False)
# vel = signal.lfilter(b, a, vel, axis=0)

# position
pos = np.zeros(vel.shape)
for i in range(1, vel.shape[0]):
    pos[i, :] = pos[i-1, :] + vel[i, :] * (1/frameRate)
b, a = signal.butter(filtOrder, (2 * pos_HP_freq) /
                     frameRate, 'high', analog=False)
# pos = signal.lfilter(b, a, pos, axis=0)

_, ax = plt.subplots(3)
# ax[0].plot(acc)
ax[0].plot(acc_rt)
# ax[1].plot(vel)
ax[1].plot(vel_rt)
# ax[2].plot(pos)
ax[2].plot(pos_rt)
plt.show()
##
