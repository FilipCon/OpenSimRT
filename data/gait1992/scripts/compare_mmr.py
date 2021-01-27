##

import os
import numpy as np
import matplotlib.pyplot as plt
import opensim as osim
from matplotlib.backends.backend_pdf import PdfPages
from utils import *

# data

subject_dir = os.path.abspath('../')
results_dir = os.path.join(subject_dir, 'real_time')
trc_original_file = os.path.join(subject_dir, 'experimental_data/task.trc')
trc_reconstructed_file = os.path.join(
    results_dir, 'marker_reconstruction/reconstructed_markers.sto')

if not (os.path.isfile(trc_original_file)
        and os.path.isfile(trc_reconstructed_file)):
    raise RuntimeError('required files do not exist')

original_marker_data = osim.TimeSeriesTableVec3(trc_original_file)
reconstructed_marker_data = osim.TimeSeriesTable(trc_reconstructed_file).packVec3()

column_labels = reconstructed_marker_data.getColumnLabels()
time = np.array(reconstructed_marker_data.getIndependentColumn())

##
with PdfPages(os.path.join(results_dir, 'compare_mmr.pdf')) as pdf:
    for column_name in column_labels:
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        idx = original_marker_data.getColumnIndex(column_name)
        idy = reconstructed_marker_data.getColumnIndex(column_name)

        original_marker = np.round(np.array(
            [original_marker_data.getRow(t)[idx].to_numpy() for t in time]) / 1000, 4)
        reconstructed_marker = np.round(np.array([
            reconstructed_marker_data.getRow(t)[idy].to_numpy() for t in time
        ]), 4)

        ax.plot(time,
                np.linalg.norm(original_marker - reconstructed_marker, axis=1))

        ax.set_xlabel('time (s)')
        ax.set_ylabel('Distance Error (m)')
        ax.set_title(column_labels[idy])
        ax.grid(True)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

##
