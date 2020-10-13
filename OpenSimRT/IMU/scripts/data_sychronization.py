##
import os
import pandas as pd
import numpy as np

import opensim as osim
import matplotlib.pyplot as plt

from matplotlib.backends.backend_pdf import PdfPages

# data
subject_dir = os.path.abspath('../../../data/gait1992/experimental_data')
imu_data_file = os.path.join(subject_dir, 'ngimu_data.csv')
insole_data_file = os.path.join(subject_dir, 'moticon.csv')

imu_data = pd.read_csv(imu_data_file, skiprows=4)
insole_data = pd.read_csv(insole_data_file, skiprows=4)
time_imu = imu_data['time']
time_insole = insole_data['time']

r_imu_column_names = ['talus_r_linAcc_x',
                      'talus_r_linAcc_y', 'talus_r_linAcc_z']
l_imu_column_names = ['talus_l_linAcc_x',
                      'talus_l_linAcc_y', 'talus_l_linAcc_z']
insole_column_names = ['R.CoP_x', 'L.CoP_x']

force_labels = ['R.TotalForce', 'L.TotalForce']

start = 0
end = -1

imu_df = pd.DataFrame({
    ** {'Time': time_imu[start:end]},
    # ** {'Time': list(range(imu_data.shape[0]))[start:end]},
    ** {label: imu_data[label][start:end] for label in r_imu_column_names + l_imu_column_names}

})

insole_df = pd.DataFrame({
    ** {'Time': time_insole[start:end]},
    # ** {'Time': list(range(imu_data.shape[0]))[start:end]},
    ** {label: insole_data[label][start:end] for label in insole_column_names},
    ** {label: insole_data[label][start:end] for label in force_labels}

})

# plots

_, ax = plt.subplots(nrows=4, ncols=1)
imu_df.plot(y=r_imu_column_names, x='Time', ax=ax[0])
imu_df.plot(y=l_imu_column_names, x='Time', ax=ax[1])
insole_df.plot(y=insole_column_names, x='Time', ax=ax[2])
insole_df.plot(y=force_labels, x='Time', ax=ax[3])
plt.show()
##
