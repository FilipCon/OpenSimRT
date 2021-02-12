##
import os
import numpy as np
import pandas as pd
from utils import read_from_storage, plot_sto_file, annotate_plot, to_gait_cycle
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from sklearn.metrics import mean_squared_error

params = {'legend.fontsize': 8,
          'legend.handlelength': 2}
plt.rcParams.update(params)

# data

subject_dir = os.path.abspath('../')
osim_results = os.path.join(subject_dir, 'inverse_dynamics/')
output_dir = os.path.join(subject_dir, 'real_time/grfm_prediction/')

tau_reference_file = os.path.join(osim_results, 'task_InverseDynamics_after_IK.sto')
tau_reference_rra_file = os.path.join(osim_results, 'task_InverseDynamics.sto')

tau_rt_file = os.path.join(output_dir, 'tau.sto')
tau_rt_gt_cop_file = os.path.join(output_dir, 'tau_with_gt_cop.sto')
tau_rt_gt_force_file = os.path.join(output_dir, 'tau_with_gt_force.sto')
tau_rt_gt_moment_file = os.path.join(output_dir, 'tau_with_gt_moment.sto')
tau_rt_gt_cop_moment_file = os.path.join(output_dir, 'tau_with_gt_cop_moment.sto')
tau_rt_gt_cop_force_file = os.path.join(output_dir, 'tau_with_gt_cop_force.sto')
tau_rt_gt_cop_moment_force_file = os.path.join(output_dir, 'tau_with_gt_cop_moment_force.sto')

if not (os.path.isfile(tau_reference_file) and
        os.path.isfile(tau_reference_rra_file) and
        os.path.isfile(tau_rt_gt_cop_file) and
        os.path.isfile(tau_rt_gt_force_file) and
        os.path.isfile(tau_rt_gt_moment_file) and
        os.path.isfile(tau_rt_gt_cop_moment_file) and
        os.path.isfile(tau_rt_gt_cop_force_file) and
        os.path.isfile(tau_rt_gt_cop_moment_force_file) and
        os.path.isfile(tau_rt_file)):
    raise RuntimeError('required files do not exist')

# read data

tau_reference = read_from_storage(tau_reference_file, resample=True)
tau_reference_rra = read_from_storage(tau_reference_rra_file, resample=True)
tau_rt = read_from_storage(tau_rt_file, resample=True)
tau_rt_gt_cop = read_from_storage(tau_rt_gt_cop_file, resample=True)
tau_rt_gt_force = read_from_storage(tau_rt_gt_force_file, resample=True)
tau_rt_gt_moment = read_from_storage(tau_rt_gt_moment_file, resample=True)
tau_rt_gt_cop_moment = read_from_storage(tau_rt_gt_cop_moment_file, resample=True)
tau_rt_gt_cop_force = read_from_storage(tau_rt_gt_cop_force_file, resample=True)
tau_rt_gt_cop_moment_force = read_from_storage(tau_rt_gt_cop_moment_force_file, resample=True)

##
# Dropping last n rows using drop
n = 5
simulation_loops = 2
total_time = 2.45

tau_reference = tau_reference.head(-n)
tau_reference_rra = tau_reference_rra.head(-n)

num_rows = tau_reference.shape[0]

# extend measured grfm in file
df = tau_reference
for i in range(simulation_loops - 1):
    tau_reference = tau_reference.append(df, ignore_index=True)
tau_reference.time = list(
    map(lambda x: x / 100.0, range(0, num_rows * simulation_loops, 1)))

df = tau_reference_rra
for i in range(simulation_loops - 1):
    tau_reference_rra = tau_reference_rra.append(df, ignore_index=True)
tau_reference_rra.time = list(
    map(lambda x: x / 100.0, range(0, num_rows * simulation_loops, 1)))

lower_lim = 0.6 + total_time
upper_lim = 1.83 + total_time

tau_reference = tau_reference[
    (tau_reference.time.apply(lambda x: x) >= lower_lim)
    & (tau_reference.time.apply(lambda x: x) <= upper_lim)]

tau_reference_rra = tau_reference_rra[
    (tau_reference_rra.time.apply(lambda x: x) >= lower_lim)
    & (tau_reference_rra.time.apply(lambda x: x) <= upper_lim)]

tau_rt = tau_rt[(tau_rt["time"].apply(lambda x: x) >= lower_lim)
                & (tau_rt["time"].apply(lambda x: x) <= upper_lim)]

tau_rt_gt_cop = tau_rt_gt_cop[
    (tau_rt_gt_cop["time"].apply(lambda x: x) >= lower_lim)
    & (tau_rt_gt_cop["time"].apply(lambda x: x) <= upper_lim)]

tau_rt_gt_force = tau_rt_gt_force[
    (tau_rt_gt_force["time"].apply(lambda x: x) >= lower_lim)
    & (tau_rt_gt_force["time"].apply(lambda x: x) <= upper_lim)]

tau_rt_gt_moment = tau_rt_gt_moment[
    (tau_rt_gt_moment["time"].apply(lambda x: x) >= lower_lim)
    & (tau_rt_gt_moment["time"].apply(lambda x: x) <= upper_lim)]

tau_rt_gt_cop_moment = tau_rt_gt_cop_moment[
    (tau_rt_gt_cop_moment["time"].apply(lambda x: x) >= lower_lim)
    & (tau_rt_gt_cop_moment["time"].apply(lambda x: x) <= upper_lim)]

tau_rt_gt_cop_moment_force = tau_rt_gt_cop_moment_force[
    (tau_rt_gt_cop_moment_force["time"].apply(lambda x: x) >= lower_lim)
    & (tau_rt_gt_cop_moment_force["time"].apply(lambda x: x) <= upper_lim)]

tau_rt_gt_cop_force = tau_rt_gt_cop_force[
    (tau_rt_gt_cop_force["time"].apply(lambda x: x) >= lower_lim)
    & (tau_rt_gt_cop_force["time"].apply(lambda x: x) <= upper_lim)]

##
# data

gait_cycle = True

if gait_cycle:
    t0 = 0.6 + total_time  # right heel strike
    tf = 1.83 + total_time  # next right heel strike
    tau_reference = to_gait_cycle(tau_reference, t0, tf)
    tau_reference_rra = to_gait_cycle(tau_reference_rra, t0, tf)
    tau_rt = to_gait_cycle(tau_rt, t0, tf)
    tau_rt_gt_cop = to_gait_cycle(tau_rt_gt_cop, t0, tf)
    tau_rt_gt_force = to_gait_cycle(tau_rt_gt_force, t0, tf)
    tau_rt_gt_moment = to_gait_cycle(tau_rt_gt_moment, t0, tf)
    tau_rt_gt_cop_moment = to_gait_cycle(tau_rt_gt_cop_moment, t0, tf)
    tau_rt_gt_cop_force = to_gait_cycle(tau_rt_gt_cop_force, t0, tf)
    tau_rt_gt_cop_moment_force = to_gait_cycle(tau_rt_gt_cop_moment_force, t0, tf)

##
# compare

d_tau_total = []
with PdfPages(output_dir + 'id_comparison.pdf') as pdf:
    for i in range(1, tau_reference.shape[1]):

        # find index
        key = tau_reference.columns[i].replace('_moment',
                                               '').replace('_force', '')
        j = tau_rt.columns.get_loc(key)

        d_tau = round(
                mean_squared_error(tau_reference.iloc[:, i],
                                   tau_rt.iloc[:, j], squared=False), 2)

        d_tau2 = round(
                mean_squared_error(tau_reference.iloc[:, i],
                                   tau_rt_gt_cop.iloc[:, j], squared=False), 2)

        is_deg = True
        if tau_rt.columns[i] in ['pelvis_tx', 'pelvis_ty', 'pelvis_tz']:
            is_deg = False

        units = ''
        if is_deg:
            units = ' (Nm)'
        else:
            units = ' (N)'

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        # ax.plot(tau_reference.time,
        #         tau_reference.iloc[:, i],
        #         label='OpenSim (after IK)',
        #         linestyle='-')
        ax.plot(tau_reference_rra.time,
                tau_reference_rra.iloc[:, i],
                label='OpenSim',
                linestyle='-')
        ax.plot(tau_rt.time,
                tau_rt.iloc[:, j],
                label='Predicted GRF&M',
                linestyle='--')
        ax.plot(tau_rt_gt_cop.time,
                tau_rt_gt_cop.iloc[:, j],
                label='Known CoP',
                linestyle='dotted')
        # ax.plot(tau_rt_gt_moment.time,
        #         tau_rt_gt_moment.iloc[:, j],
        #         label='Known Moment',
        #         linestyle='-.')
        # ax.plot(tau_rt_gt_force.time,
        #         tau_rt_gt_force.iloc[:, j],
        #         label='Known Force',
        #         linestyle='--')
        ax.plot(tau_rt_gt_cop_moment.time,
                tau_rt_gt_cop_moment.iloc[:, j],
                label='Known Moment, CoP',
                linestyle=(0, (5, 5)))
        ax.plot(tau_rt_gt_cop_force.time,
                tau_rt_gt_cop_force.iloc[:, j],
                label='Known Force, Cop',
                linestyle='-.')
        # ax.plot(tau_rt_gt_cop_moment_force.time,
        #         tau_rt_gt_cop_moment_force.iloc[:, j],
        #         label='Known Force, Moment, Cop',
        #         linestyle='-.')
        if gait_cycle:
            ax.set_xlabel('gait cycle (%)')
        else:
            ax.set_xlabel('time (s)')

        ax.set_ylabel('generalized forces' + units)
        ax.set_title(tau_rt.columns[j])
        # annotate_plot(ax, 'RMSE = ' + str(d_tau))
        ax.legend(loc='lower right')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

# print('d_tau: μ = ', np.round(np.mean(d_tau_total), 3), ' σ = ',
#       np.round(np.std(d_tau_total, ddof=1), 3))

# with open(output_dir + 'metrics.txt', 'w') as file_handle:
#     file_handle.write('RMSE\n')
#     file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_tau_total), 3)) +
#                       ' σ = ' + str(np.round(np.std(d_tau_total, ddof=1), 3)))

##