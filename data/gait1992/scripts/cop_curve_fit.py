##
import os
import opensim
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
from utils import annotate_plot, calculate_absolute_body_transformations, \
    get_model_coordinates, plot_sto_file, \
    read_from_storage, rmse_metric
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

# ==============================================================================
# curves to fit

# double support time-period determined empirically
Tss = 0.4
d = 0.24


def sigmoid_with_bump(t, A, K, B, M, m1, m2, c):
    ''' Sigmoid with "bump" component
    '''
    return  A + K / (1.0 + np.exp(B * (t - m1 * Tss))) + \
        M * np.exp(-np.power((t - m2*Tss), 2) / (2 * np.power(c, 2))) * t


def test_curve(t, M, m2, c):
    return M * t * np.exp(-np.power((t - m2 * Tss), 2) / (2 * np.power(c, 2)))


# curves from bibliography
def jung(t):
    omega = 2.0 * np.pi / Tss
    return (-2.0 / (3 * np.pi) *
            (np.sin(omega * t) - np.sin(2 * omega * t) / 8 -
             3.0 / 4.0 * omega * t))


# ==============================================================================

# data
subject_dir = os.path.abspath("../experimental_data/")
results_dir = os.path.join(subject_dir, "../real_time/grfm_prediction/")

model_file = os.path.join(
    subject_dir, "../residual_reduction_algorithm/model_adjusted.osim")
ik_motion_file = os.path.join(
    subject_dir, "../inverse_kinematics/task_InverseKinematics.mot")

right_grf_file = os.path.join(subject_dir, "wrench_right_in_foot.sto")
left_grf_file = os.path.join(subject_dir, "wrench_left_in_foot.sto")

# read data
ik_motion = read_from_storage(ik_motion_file)
right_wrench = read_from_storage(right_grf_file)
left_wrench = read_from_storage(left_grf_file)

# %%
# ==============================================================================
# help functions


def detectEvent(v, event):
    """ Detect HS and TO events in time-series based on transition from zero to
        non-zero values, and vice-versa.
    """
    if event == "HS":
        return np.where([
            v[i - 1] != v[i] and v[i - 1] == 0
            for i, x in enumerate(np.array(v) != 0)
        ])[0].tolist()
    elif event == "TO":
        return np.where([
            v[i - 1] != v[i] and v[i] == 0
            for i, x in enumerate(np.array(v) != 0)
        ])[0].tolist()
    else:
        raise RuntimeError("Wrong event value")


# prepare data
# ==============================================================================

# find HS and TO events
gait_events = {}
gait_events["rhs"] = detectEvent(right_wrench['f_y'].values, "HS")
gait_events["lhs"] = detectEvent(left_wrench['f_y'].values, "HS")
gait_events["rto"] = detectEvent(right_wrench['f_y'].values, "TO")
gait_events["lto"] = detectEvent(left_wrench['f_y'].values, "TO")

# assert equal number of paired (hs,to) indexes
assert len(gait_events["rhs"]) == len(gait_events["lto"])
assert len(gait_events["lhs"]) == len(gait_events["rto"])

# concatenate data
to_idx = np.array([gait_events["rto"], gait_events["lto"]])
hs_idx = np.array([gait_events["rhs"], gait_events["lhs"]])

grf_px = np.array([left_wrench['p_x'].values , right_wrench['p_x'].values ])
grf_pz = np.array([left_wrench['p_z'].values , -right_wrench['p_z'].values ])

# grf_px = np.array([ right_wrench['p_x'].values , left_wrench['p_x'].values])
# grf_pz = np.array([ -right_wrench['p_z'].values , left_wrench['p_z'].values])

grf_data = [grf_px, grf_pz]
curves = [sigmoid_with_bump, sigmoid_with_bump]
titles = ['Anterior CoP Displacement', 'Lateral CoP Displacement']

# ==============================================================================
# fit curve and save results

def prepareSingleSupportData(measurements, time, hs_id_list, to_id_list):
    ydata = []
    xdata = []
    for i, (to_ids, hs_ids) in enumerate(zip(to_id_list, hs_id_list)):
        for to, hs in zip(to_ids, hs_ids):
            t0 = time[to]
            y_row = measurements[i][to:hs]
            xdata.extend([t - t0 for t in time[to:hs]])
            ydata.extend([(y - y_row[0]) / d for y in y_row])
    return ydata, xdata

def prepareStanceData(measurements, time, hs_id_list, to_id_list):
    ydata = []
    xdata = []
    for i, (to, hs) in enumerate(zip(to_id_list, hs_id_list)):
        t0 = time[hs]
        y_row = measurements[i][hs:to]
        xdata.extend([t - t0 for t in time[hs:to]])
        ydata.extend([(y - y_row[0]) / d for y in y_row])
    return ydata, xdata

with PdfPages(results_dir + "cop_transition.pdf") as pdf, open(
        results_dir + "cop_curve_parameters.txt", "w") as text_file:
    for i, (data, function) in enumerate(zip(grf_data, curves)):

        # create data vector with measured data
        ydata, xdata = prepareSingleSupportData(data, ik_motion["time"].values,
                                           hs_idx, to_idx)
        # ydata, xdata = prepareStanceData(data, ik_motion.time.values, hs_idx[:,0], to_idx[:,1])

        # fit curve to measured data
        params, cov = curve_fit(function, xdata, ydata,
                                method='trf')  # lm (default), trf, dogbox

        # write parameters to file
        text_file.write('// {} {} parameters\n'.format(titles[i],
                                                       function.__name__))
        for j, v in enumerate(params):
            text_file.write('double {} = {};\n'.format(
                function.__code__.co_varnames[1:][j], v))
        text_file.write('\n\n')

        # # plot results
        time = np.linspace(0, Tss)
        fig, ax = plt.subplots()
        ax.scatter(xdata, ydata, s=1, label="Measured Data")
        ax.plot(time,
                function(time, *params),
                color='r',
                linewidth=2,
                label='Fit Curve')
        # ax.plot(time, jung(time), color='k', linewidth=2, label='Fit Curve')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel('Normalized Units')
        ax.set_title(titles[i])
        ax.legend()
        ax.grid(True)
        plt.show()
        pdf.savefig(fig)
        ##
