##
import os
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation
import pandas as pd

from util import readMotionFile, readCSV, index_containing_substring


# data

subject_dir = os.getcwd() + '/../data/gait1848/'
results_dir = subject_dir + 'results_rt/'
trc_original_file = results_dir + 'marker_reconstruction_original.csv'
trc_reconstructed_file = results_dir + 'marker_reconstruction_reconstructed.csv'


missing_marker_labels = ["R.ASIS", "R.Thigh.Rear", "R.Toe.Tip",
            "R.Shank.Upper", "L.Shank.Upper","L.Thigh.Rear", "L.Toe.Tip"]



if not (os.path.isfile(trc_original_file) and
        os.path.isfile(trc_reconstructed_file)):
    raise RuntimeError('required files do not exist')

(trc_original_labels, trc_original) = readCSV(trc_original_file)
(trc_reconstructed_labels, trc_reconstructed) = readCSV(trc_reconstructed_file)
trc_original = np.array(trc_original)
trc_reconstructed = np.array(trc_reconstructed)



marker_labels = trc_original_labels[1::3]
time = trc_original[:, 0]
frames = time.shape[0]
num_markers = len(marker_labels)

data = []
t = []
color = []

for label in missing_marker_labels:
    idx = index_containing_substring(trc_reconstructed_labels, label)[0]

    marker_coord = np.stack((trc_reconstructed[:, idx],
                             trc_reconstructed[:, idx+1],
                             trc_reconstructed[:, idx+2]), axis=1)
    data += list(marker_coord)
    t += list(time)
    color += [0]

for label in marker_labels:
    if label == 'time':
        continue

    idx = index_containing_substring(trc_original_labels, label)[0]

    marker_coord = np.stack((trc_original[:, idx],
                             trc_original[:, idx+1],
                             trc_original[:, idx+2]), axis=1)
    data += list(marker_coord)
    t += list(time)
    color += [-1]

data = np.array(data)
t = np.array(t)

df = {"time": t, "x": data[:, 0], "y": -data[:, 2], "z": data[:, 1]}


def update_graph(num):
    index = df['time'] == time[num]
    graph._offsets3d = (df['x'][index], df['y'][index], df['z'][index])
    # graph.set_array(np.array(color))
    title.set_text('Missing Marker Reconstruction\n Time={}'.format(time[num]))
    return title, graph


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
title = ax.set_title('Missing Marker Reconstruction\n')

index = df['time'] == 0
graph = ax.scatter(df['x'][index], df['y'][index], df['z'][index])


ax.set_xlim3d(0, 1.2)
ax.set_ylim3d(-0.5, 0.5)
ax.set_zlim3d(-0.05, 1.2)
plt.xlabel('x')
plt.ylabel('y')

ani = matplotlib.animation.FuncAnimation(fig, update_graph, frames=frames,
                                         interval=60, blit=False)

plt.show()
