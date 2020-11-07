##
import os
import pandas as pd
# import numpy as np

# import opensim as osim
import matplotlib.pyplot as plt

# from matplotlib.backends.backend_pdf import PdfPages

# data
subject_dir = os.path.abspath('../../../data/gait1992')
original_imu_data_file = os.path.join(
    subject_dir, 'experimental_data/ngimu_data_3.csv')
original_insole_data_file = os.path.join(
    subject_dir, 'experimental_data/moticon_3.csv')
sync_imu_data_file = os.path.join(
    subject_dir, 'real_time/sync/ngimu_data.csv')
sync_insole_data_file = os.path.join(
    subject_dir, 'real_time/sync/moticon_data.csv')

original_imu_data = pd.read_csv(original_imu_data_file, skiprows=4)
original_insole_data = pd.read_csv(original_insole_data_file, skiprows=4)
original_time_imu = original_imu_data['time']
original_time_insole = original_insole_data['time']
sync_imu_data = pd.read_csv(sync_imu_data_file, skiprows=5)
sync_insole_data = pd.read_csv(sync_insole_data_file, skiprows=5)
sync_time_imu = sync_imu_data['time']
sync_time_insole = sync_insole_data['time']

r_imu_column_names = ['calcn_r_imu_linAcc_x']
l_imu_column_names = ['calcn_l_imu_linAcc_x']
insole_column_names = ['R.CoP_x']
force_labels = ['R.TotalForce']

start = 0
end = -1

imu_df = pd.DataFrame({
    ** {'Time_original': original_time_imu[start:end]},
    ** {'Time_sync': sync_time_imu[start:end]},
    ** {label + '_original': original_imu_data[label][start:end] for label in r_imu_column_names +
        l_imu_column_names},
    ** {label + '_sync': sync_imu_data[label][start:end] for label in r_imu_column_names +
        l_imu_column_names}
})

insole_df = pd.DataFrame({
    ** {'Time_original': original_time_insole[start:end]},
    ** {'Time_sync': sync_time_insole[start:end]},
    ** {label + '_original': original_insole_data[label][start:end] for label in insole_column_names},
    ** {label + '_original': original_insole_data[label][start:end] for label in force_labels},
    ** {label + '_sync': sync_insole_data[label][start:end] for label in insole_column_names},
    ** {label + '_sync': sync_insole_data[label][start:end] for label in force_labels}
})

# plots

_, ax = plt.subplots(nrows=3, ncols=1)

#
imu_df.plot(y=[x + '_original' for x in r_imu_column_names],
            x='Time_original', ax=ax[0])
imu_df.plot(y=[x + '_sync' for x in r_imu_column_names],
            x='Time_sync', ax=ax[0])

#
insole_df.plot(y=[x + '_original' for x in insole_column_names],
               x='Time_original', ax=ax[1])
insole_df.plot(y=[x + '_sync' for x in insole_column_names],
               x='Time_sync', ax=ax[1])

#
insole_df.plot(y=[x + '_original' for x in force_labels],
               x='Time_original', ax=ax[2])
insole_df.plot(y=[x + '_sync' for x in force_labels],
               x='Time_sync', ax=ax[2])
plt.show()
##
