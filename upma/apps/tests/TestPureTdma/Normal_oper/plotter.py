import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
# from normal_oper_check import get_cdf


def show_delay_distribution(flows_delay_distribution):
    '''
        input:
            flows_delay_distribution = [HI_flow_delay_dist, LO_flow_delay_dist]
            HI_flow_delay_dist = {node1: {delay1 : prob1, delay2 : prob2, ...}, node2: ...}
    '''
    fig, ax = plt.subplots()
    colors = ['b', 'r']
    markers = ['o', 'x']
    legends = ['HI_TASK', 'LO_TASK']

    max_y_tick = 0
    max_x_tick = 0
    flow_idx = 0
    for flow_delay_distribution in flows_delay_distribution:
        x_values = []
        y_values = []
        z_values = []
        hop_idx = 1
        for node, delay_prob in flow_delay_distribution.items():
            for delay, prob in delay_prob.items():
                x_values.append(hop_idx)
                y_values.append(delay)
                z_values.append(prob)
            hop_idx += 1
        # x_values = list(range(1, len(flow_delay_distribution)+1))
        # y_values = [list(delay_dict.keys())[0] for delay_dict in list(flow_delay_distribution.values())]
        # z_values = [list(delay_dict.values())[0] for delay_dict in list(flow_delay_distribution.values())]
        max_y_tick = max(y_values) if max(y_values) > max_y_tick else max_y_tick
        max_x_tick = max(x_values) if max(x_values) > max_x_tick else max_x_tick
        print("[dbg]flow_delay_distribution.values:", list(flow_delay_distribution.values()))
        print("[dbg]x_values:", x_values)
        print("[dbg]y_values:", y_values)
        print("[dbg]z_values:", z_values)
        addtext(ax, flow_idx, x_values, y_values, z_values, colors[flow_idx])
        ax.scatter(x_values,
                    y_values,
                    color = colors[flow_idx],
                    marker = markers[flow_idx],
                    s = 100,
                    label=legends[flow_idx] if flow_idx <= len(legends)-1 else None)
        flow_idx += 1

    ax.set_ylabel('Delay (slot)')
    ax.set_xlabel('Distance (hop)')
    plt.xticks(np.arange(1, max_x_tick+1))
    plt.yticks(np.arange(0, max_y_tick+1))
    plt.axis([0.5, max_x_tick+0.5, -1, max_y_tick+1])
    plt.legend(loc='upper left')
    plt.grid(True)
    # plt.show()

def addtext(ax, flow_id, x_values, y_values, z_values, color_):
    y_add_position = -0.3 if flow_id == 0 else 0.15
    x_add_position = -0.1
    for idx in range(len(x_values)):
        ax.text(x_values[idx]+x_add_position,
                y_values[idx]+y_add_position,
                '{}%'.format(int(z_values[idx]*100)),
                color = color_)


def show_delay_distribution_with_cdf(flows_delay_distribution):
    '''
        input:
            flows_delay_distribution = [HI_flow_delay_dist, LO_flow_delay_dist]
            HI_flow_delay_dist = {node1: [(delay1, delay2, ...), CDF], node2: ...}
    '''
    fig, ax = plt.subplots()
    colors = ['b', 'r']
    line_style = ['-', '--', ':', '-.']
    markers = ['o', 'x']
    idx = 0

    for flow_delay_distribution in flows_delay_distribution:
        # legends = list(range(1, len(flow_delay_distribution)+1)+'hop')
        inner_idx = 0
        for node, delay_cdf in flow_delay_distribution.items():
            x_values = delay_cdf[0]
            y_values = delay_cdf[1]
            print("x_values:", x_values)
            print("y_values:", y_values)
            line = Line2D( x_values,
                            y_values,
                            color=colors[idx],
                            drawstyle='steps',
                            ls=line_style[np.mod(inner_idx, len(line_style)-1)]
                            )
            ax.add_line(line)
            inner_idx += 1
        idx += 1
    ax.set_ylabel('ECDF')
    ax.set_xlabel('Delay (slot)')
    ax.relim()
    ax.autoscale()
    # plt.legend(loc='lower right')


# def show_adaptivity(adaptive_intervals_set):
#     '''
#     output: CDF results of the input
#     '''
#     obj_len = len(adaptive_intervals_set)
#     fig, ax = plt.subplots()
#
#
#     colors = ['b', 'g', 'r', 'c']
#     line_style = ['-', '--', ':', '-.']
#     legends = ['T: to20', 'T: to10', 'T: to25']
#     for i in range(obj_len):
#         x_values, y_values = get_cdf(adaptive_intervals_set[i])
#         x_values_ms = x_values * 10
#         line = Line2D( x_values_ms, y_values,
#                         color=colors[0],
#                         drawstyle='steps',
#                         ls=line_style[np.mod(i, len(line_style)-1)],
#                         label=legends[i] if i <= len(legends)-1 else None)
#         ax.add_line(line)
#
#     ax.set_ylabel('ECDF')
#     ax.set_xlabel('adaptive interval (ms)')
#     ax.relim()
#     ax.autoscale()
#     plt.legend()
#     plt.show()
