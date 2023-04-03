#!/usr/bin/env python3.7

from pysdf import SDF
import matplotlib.pyplot as plt
import matplotlib.collections as mc
import numpy as np
import time

from corridor import Corridor

for worldnr in range(99, 100):
    # --------------------------------------------------------------------
    # Load world
    file_name = '../../jackal_helper/worlds/BARN/world_' + \
        str(worldnr) + '.world'
    world = SDF.from_file(file_name, remove_blank_text=True)
    world.version = "1.6"  # v1
    unit_cylinder = {}
    counter = 0
    for idx, pose in enumerate(world.iter("model/link/pose")):
        cyl_pos = [float(x) for x in pose.text.split()]
        x, y, z, a, b, c = cyl_pos
        unit_cylinder[idx] = [x, y]
        counter += 1

    xy = np.array([value for value in unit_cylinder.values()]).T
    xy = xy[:, :-2]
    counter = counter - 2
    radius = 30
    sizes = radius*np.ones(len(xy))

    xyh = np.vstack((xy, np.ones(len(xy.T))))

    # --------------------------------------------------------------------
    # Parameters
    veh_width = 0.430
    veh_length = 0.508
    room_width = 4.425-0.075
    start_pose = [-2.25, 2.7]
    sweep = 0

    # --------------------------------------------------------------------
    # Construct initial corridor
    corridor_0 = Corridor(width=4, height=4.8, center=start_pose, copy=True)
    best_corridor = [corridor_0]
    best_corridor[0].quality = 1

    # --------------------------------------------------------------------
    # Start sweeping until the end
    while max([y[1] for y in best_corridor[-1].corners]) < 9.9 and sweep < 10:
        sweep += 1
        print(str(worldnr) + ' sweep ' + str(sweep))
        best_cor = best_corridor[-1]

        n = 3
        centers = [best_cor.center + (1+i)*best_cor.height/2/(n+1) *
                   best_cor.wf[:2] for i in range(n)]
        centers = centers + centers + centers
        tilts_neg = [-np.pi/4 for i in range(n)]
        tilts_zero = [0. for i in range(n)]
        tilts_pos = [np.pi/4 for i in range(n)]
        tilts = tilts_neg + tilts_zero + tilts_pos

        best_cor.sweep_corridor(width=veh_width, height=veh_length,
                                centers=centers, tilts=tilts, copy=True)

        # --------------------------------------------------------------------
        # Grow corridors in all directions
        for corridor in best_cor.children:
            corridor.grow_all_edges(xyh)

        # --------------------------------------------------------------------
        # Select best corridors, based on furthest corner
        corridors_sorted = sorted(best_cor.children,
                                  key=lambda x: max([y[1] for y in x.corners]),
                                  reverse=True)[:3]
        best_corridor += [corridors_sorted[0]]
        best_corridor[-1].quality = 1

    # --------------------------------------------------------------------
    # Plot results
    plt.rcParams["figure.figsize"] = [5.0, 9.0]

    fig, ax = plt.subplots()
    collection = mc.CircleCollection(sizes, offsets=xy.T,
                                     transOffset=ax.transData, color='green')
    ax.add_collection(collection)
    plt.xlim([-4.7, 0.2])
    plt.ylim([-0.5, 12])
    ax.set_aspect('equal', 'box')

    # plot original corridor
    corridor_0.plot(ax, 'green', '-.')
    ax.set_title('world ' + str(worldnr) + ', sweep ' + str(0) + ' of ' +
                 str(sweep) + ' sweeps')
    fig_name = '../worlds_sweeps/world_' + str(worldnr) + '_0.png'
    # fig.savefig(fig_name)

    for i in range(sweep):
        # plot all sweeps
        sweep_plots = []
        for corridor in best_corridor[i].children:
            sweep_plots += [corridor.original.plot(ax, 'pink')]
        ax.set_title('world ' + str(worldnr) + ', sweep ' + str(i+1) +
                     ' of ' + str(sweep) + ' sweeps')
        fig_name = '../worlds_sweeps/world_' + str(
            worldnr) + '_' + str(i+1) + 'a.png'
        # fig.savefig(fig_name)
        plt.pause(1)

        for corridor in best_corridor[i].children:
            sweep_plots += [corridor.plot(ax, 'firebrick', '-.')]
        fig_name = '../worlds_sweeps/world_' + str(
            worldnr) + '_' + str(i+1) + 'b.png'
        # fig.savefig(fig_name)
        plt.pause(1)

        for corridor in sweep_plots:
            ax.lines.remove(corridor)
        best_corridor[i+1].plot(ax, 'gold')
        fig_name = '../worlds_sweeps/world_' + str(
            worldnr) + '_' + str(i+1) + 'c.png'
        # fig.savefig(fig_name)
        plt.pause(1)

plt.show()
