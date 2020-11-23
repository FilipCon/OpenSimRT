##
import os
import numpy as np
import pandas as pd
from scipy import signal

from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

# data

subject_dir = os.path.abspath('../')
results_dir = os.path.join(subject_dir, 'real_time/grfm_prediction/')

ngimu_data_file = os.path.join(
    subject_dir, 'experimental_data/ngimu_data_3.csv')

right_wrench_rt_file = os.path.join(results_dir, 'wrench_right.sto')
left_wrench_rt_file = os.path.join(results_dir, 'wrench_left.sto')

if not (os.path.isfile(right_wrench_rt_file) and
        os.path.isfile(left_wrench_rt_file)):
    raise RuntimeError('required files do not exist')


# read data
right_wrench = pd.read_table(right_wrench_rt_file, skiprows=(4))
left_wrench = pd.read_table(left_wrench_rt_file, skiprows=(4))
ngimu_data = pd.read_csv(ngimu_data_file, skiprows=(4))

# convert zeros values to NaN for the Co
# right_wrench.p_x = right_wrench.p_x.replace(0.0, float('nan'))
# right_wrench.p_y = right_wrench.p_y.replace(0.0, float('nan'))
# right_wrench.p_z = right_wrench.p_z.replace(0.0, float('nan'))
# left_wrench.p_x = left_wrench.p_x.replace(0.0, float('nan'))
# left_wrench.p_y = left_wrench.p_y.replace(0.0, float('nan'))
# left_wrench.p_z = left_wrench.p_z.replace(0.0, float('nan'))

##


def plotXYZ(est_data_frame, id_est, hs_events, to_events,
            title, y_label):
    ''' Helper function for plotting forces/moments/cop
    '''
    fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(8, 8))
    for i in range(3):
        ax[i].plot(est_data_frame.time, est_data_frame.iloc[:, i + id_est],
                   '--', label='Predicted')
        ax[i].set_xlabel('time (s)')
        ax[i].set_ylabel(y_label)
        ax[i].set_title(est_data_frame.columns[id_est + i] + ' ' + title)
        ax[i].legend(loc='lower left')
        # ax[i].grid(True)
        # for hs in hs_events:
        #     ax[i].axvline(x=hs, label='HS', color='tab:red',
        #                   linestyle='--', linewidth=0.5)
        # for to in to_events:
        #     ax[i].axvline(x=to, label='TO', color='tab:cyan',
        #                   linestyle='--', linewidth=0.5)
    fig.tight_layout()
    pdf.savefig(fig)
    plt.close()


##
with PdfPages(results_dir + 'grfm_estimation.pdf') as pdf:

    # forces
    # right
    id_est = right_wrench.columns.get_loc('f_x')
    plotXYZ(right_wrench, id_est,
            [],
            [],
            'right', 'force (N)')

    # left
    id_est = left_wrench.columns.get_loc('f_x')
    plotXYZ(left_wrench, id_est,
            [],
            [],
            'left', 'force (N)')

    # # torques
    # right
    id_est = right_wrench.columns.get_loc('tau_x')
    plotXYZ(right_wrench, id_est, [], [],
            'right', 'moment (Nm)')

    # left
    id_est = left_wrench.columns.get_loc('tau_x')
    plotXYZ(left_wrench, id_est, [], [],
            'left', 'moment (Nm)')

    # # point
    # right
    id_est = right_wrench.columns.get_loc('p_x')
    plotXYZ(right_wrench, id_est, [], [],
            'right', 'coordinate (m)')

    # left
    id_est = left_wrench.columns.get_loc('p_x')
    plotXYZ(left_wrench, id_est, [], [],
            'left', 'coordinate (m)')

##
