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
csv_file = os.path.join(subject_dir, 'ngimu_data.csv')
# csv_file = os.path.join(subject_dir, 'sensors.csv')
csv_on_file = os.path.join(subject_dir, 'estimates.csv')

# read csv
csv = pd.read_csv(csv_file, skiprows=4)
csv_online = pd.read_csv(csv_on_file, skiprows=4)

pos_on = np.array([csv_online['px'], csv_online['py'],
                   csv_online['pz']]).transpose()

# csv = pd.read_csv(csv_file)

gravity = 9.81
frameRate = 60

# # column labels
imu_name = 'pelvis_imu'
time_label = 'time'
acc_suffix = ['_ax', '_ay', '_az']
vel_suffix = ['_vx', '_vy', '_vz']
pos_suffix = ['_px', '_py', '_pz']
quat_suffix = ['_q1', '_q2', '_q3', '_q4']
acc_column_labels = [imu_name + x for x in acc_suffix]
vel_column_labels = [imu_name + x for x in vel_suffix]
pos_column_labels = [imu_name + x for x in pos_suffix]
quat_column_labels = [imu_name + x for x in quat_suffix]
bar_column_labels = [imu_name + 'barometer']
acc_filt_column_labels = [imu_name + x + '_filt' for x in acc_suffix]
vel_filt_column_labels = [imu_name + x + '_filt' for x in vel_suffix]
# pos_filt_column_labels = [imu_name + x + '_filt' for x in pos_suffix]
# quat_column_labels = ['W', 'X', 'Y', 'Z']
# acc_column_labels = ['Accelerometer X (g)',
#                      'Accelerometer Y (g)',
#                      'Accelerometer Z (g)']
# vel_column_labels = ['Velocity X (m/s)',
#                      'Velocity Y (m/s)',
#                      'Velocity Z (m/s)']
# pos_column_labels = ['Position X (m)',
#                      'Position Y (m)',
#                      'Position Z (m)']
# acc_filt_column_labels = ['Filtered Accelerometer X (m/s/s)',
#                           'Filtered Accelerometer Y (m/s/s)',
#                           'Filtered Accelerometer Z (m/s/s)']
# vel_filt_column_labels = ['Filtered Velocity X (m/s)',
#                           'Filtered Velocity Y (m/s)',
#                           'Filtered Velocity Z (m/s)']
# pos_filt_column_labels = ['Filtered Position X (m)',
#                           'Filtered Position Y (m)',
#                           'Filtered Position Z (m)']


def quatern2rotMat(q):
    R = np.array(np.zeros((3, 3)))
    R[0, 0] = 2.0 * q[0]**2 - 1 + 2.0 * q[1]**2
    R[0, 1] = 2.0 * (q[1] * q[2] + q[0] * q[3])
    R[0, 2] = 2.0 * (q[1] * q[3] - q[0] * q[2])
    R[1, 0] = 2.0 * (q[1] * q[2] - q[0] * q[3])
    R[1, 1] = 2.0 * q[0]**2 - 1 + 2.0 * q[2]**2
    R[1, 2] = 2.0 * (q[2] * q[3] + q[0] * q[1])
    R[2, 0] = 2.0 * (q[1] * q[3] + q[0] * q[2])
    R[2, 1] = 2.0 * (q[2] * q[3] - q[0] * q[1])
    R[2, 2] = 2.0 * q[0]**2 - 1 + 2.0 * q[3]**2
    return R


h0 = 0
Rgas = 8.3144598
g = 9.80665
M = 0.0289644
T0 = 288.15
P0 = 101325.00

# get pelvis measured data
time = np.array(csv['time'])
acc = np.array([csv[x] for x in acc_column_labels]).transpose()
quat = np.array([csv[x] for x in quat_column_labels]).transpose()
bar = np.array([csv[x] for x in bar_column_labels]).transpose()
# altidute = h0 - Rgas * T0 / g / M * np.log(bar /100 / P0)

R = np.array([quatern2rotMat(x) for x in quat])
tAcc = np.array([R[i].dot(acc[i]) for i, _ in enumerate(acc)])

linAcc = (tAcc - np.array([0, 0, 1])) * gravity

# FILTER

# filter cutoff freq
filtOrder = 2
acc_LP_freq = 5
acc_HP_freq = 0.001
vel_HP_freq = 0.1
pos_HP_freq = 0.1

#  acceleration
b, a = signal.butter(filtOrder, (2 * acc_LP_freq) /
                     frameRate, 'low', analog=False)
d, c = signal.butter(filtOrder, (2 * acc_HP_freq) /
                     frameRate, 'high', analog=False)
acc_f = signal.lfilter(b, a, signal.lfilter(d, c, linAcc))

# velocity
vel = np.zeros(linAcc.shape)
for i in range(1, linAcc.shape[0]):
    vel[i, :] = vel[i-1, :] + linAcc[i, :] * (1/frameRate)
b, a = signal.butter(filtOrder, (2 * vel_HP_freq) /
                     frameRate, 'high', analog=False)
vel_f = signal.lfilter(b, a, vel, axis=0)

# position

pos = np.zeros(vel.shape)
for i in range(1, vel.shape[0]):
    pos[i, :] = pos[i-1, :] + vel_f[i, :] * (1/frameRate) + 1/2 * acc_f[i]* (1/frameRate)**2
# b, a = signal.butter(filtOrder, (2 * pos_HP_freq) /
#                      frameRate, 'high', analog=False)
# pos_f = signal.lfilter(b, a, pos, axis=0)


_, ax = plt.subplots()
# plt.plot(acc_f)
# plt.plot(vel_f)
# plt.plot(pos_f)
# ax.plot(pos_on)
ax.plot(pos)
ax.plot(altidute - altidute[0])

##

from pykalman import UnscentedKalmanFilter

##
# plt.legend(['x', 'y', 'z'])
# def createDataFrame(time, x, x_f, x_columns, x_f_columns):
#     return pd.DataFrame({
#         ** {'Time': time},
#         ** {label: x[:, i] for i, label in enumerate(x_columns)},
#         ** {label: x_f[:, i] for i, label in enumerate(x_f_columns)},
#     })


# # create dataframes
# df_acc = createDataFrame(csv['time'], linAcc, acc_f,
#                          acc_column_labels, acc_filt_column_labels)
# df_vel = createDataFrame(csv['time'], vel, vel_f,
#                          vel_column_labels, vel_filt_column_labels)
# df_pos = createDataFrame(csv['time'], pos, pos_f,
#                          pos_column_labels, pos_filt_column_labels)

# # plots
# with PdfPages(os.path.join(subject_dir, 'Acceleration.pdf')) as pdf:
#     _, ax = plt.subplots(nrows=3, ncols=1)
#     for i in range(3):
#         df_acc.plot(y=[acc_column_labels[i], acc_filt_column_labels[i]], x='Time',
#                     ax=ax[i])
#     pdf.savefig()
# with PdfPages(os.path.join(subject_dir, 'Velocity.pdf')) as pdf:
#     _, ax = plt.subplots(nrows=3, ncols=1)
#     for i in range(3):
#         df_vel.plot(y=[vel_column_labels[i], vel_filt_column_labels[i]], x='Time',
#                     ax=ax[i])
#     pdf.savefig()
# with PdfPages(os.path.join(subject_dir, 'Position.pdf')) as pdf:
#     _, ax = plt.subplots(nrows=3, ncols=1)
#     for i in range(3):
#         df_pos.plot(y=[pos_column_labels[i], pos_filt_column_labels[i]], x='Time',
#                     ax=ax[i])
#     pdf.savefig()
plt.show()
##
