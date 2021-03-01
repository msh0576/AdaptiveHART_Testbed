import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from adaptive_check import get_cdf

def show_adaptivity(adaptive_intervals_set):
    '''
    output: CDF results of the input
    '''
    obj_len = len(adaptive_intervals_set)
    fig, ax = plt.subplots()


    colors = ['b', 'g', 'r', 'c']
    line_style = ['-', '--', ':', '-.']
    legends = ['T: to20', 'T: to10', 'T: to25']
    for i in range(obj_len):
        x_values, y_values = get_cdf(adaptive_intervals_set[i])
        x_values_ms = x_values * 10
        line = Line2D( x_values_ms, y_values,
                        color=colors[0],
                        drawstyle='steps',
                        ls=line_style[np.mod(i, len(line_style)-1)],
                        label=legends[i] if i <= len(legends)-1 else None)
        ax.add_line(line)

    ax.set_ylabel('ECDF')
    ax.set_xlabel('adaptive interval (ms)')
    ax.relim()
    ax.autoscale()
    plt.legend()
    plt.show()
