import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import seaborn as sns
import pandas as pd

def show_adaptivity(adaptive_intervals_set, WH_adaptive_intervals_set):
    '''
    output: CDF results of the input
    '''
    obj_len = len(adaptive_intervals_set)
    colors = ['b', 'g', 'r', 'c']
    line_style = ['solid', 'dashed','dotted']
    AH_legends = ['AH_T: to10', 'AH_T: to20', 'AH_T: to25']
    WH_legends = ['WH_T: to10', 'WH_T: to20', 'WH_T: to25']


    df_AH = make_pandas_frame(adaptive_intervals_set, AH_legends)
    df_WH = make_pandas_frame(WH_adaptive_intervals_set, WH_legends)

    for idx in range(len(df_AH.columns)):
        fig = sns.ecdfplot(df_AH[AH_legends[idx]],  color='red', ls=line_style[idx], label=AH_legends[idx])
    for idx in range(len(df_AH.columns)):
        sns.ecdfplot(df_WH[WH_legends[idx]],  color='blue', ls=line_style[idx], label=WH_legends[idx])


    # fig, ax = plt.subplots()
    # for i in range(obj_len):
    #     x_values, y_values = get_cdf(adaptive_intervals_set[i])
    #     print("x_values:", x_values)
    #     print("y_values:", y_values)
    #     x_values_ms = x_values * 10
    #     line = Line2D( x_values_ms, y_values,
    #                     color=colors[0],
    #                     drawstyle='steps',
    #                     ls=line_style[np.mod(i, len(line_style)-1)],
    #                     label=legends[i] if i <= len(legends)-1 else None)
    #     ax.add_line(line)
    # draw_figure_in_ax(ax, adaptive_intervals_set, 'AH', 'b', line_style, legends)
    # draw_figure_in_ax(ax, WH_adaptive_intervals_set, 'WH', 'r', line_style, legends)

    # ax.set_ylabel('ECDF')
    # ax.set_xlabel('adaptive interval (ms)')
    # ax.relim()
    # ax.autoscale()
    plt.xlim(0, 2100)
    plt.ylabel('ECDF')
    plt.xlabel('adaptive interval (ms)')
    plt.legend()
    plt.grid()
    plt.show()


def draw_figure_in_ax(ax, dataset, protocol, color_, line_style, legends):
    '''
        draw a line per data in a dataset
    '''
    obj_len = len(dataset)

    for i in range(obj_len):
        x_values, y_values = get_cdf(dataset[i])
        x_values_ms = x_values * 10
        line = Line2D( x_values_ms, y_values,
                        color=color_,
                        drawstyle='steps',
                        ls=line_style[np.mod(i, len(line_style)-1)],
                        label=protocol+'_'+legends[i] if i <= len(legends)-1 else None)
        ax.add_line(line)


def get_cdf(data):
    '''
    output (array): x_values, y_values : cumulative probability
    '''
    raw_data = np.array(data)
    unique_data_list = list(set(data))
    x_values = np.sort(np.array(unique_data_list))
    data_size = raw_data.size
    y_values = []
    for x in x_values:
        tmp = raw_data[raw_data <= x]
        value = tmp.size/data_size
        y_values.append(value)
    return x_values, y_values


def make_pandas_frame(data, columns):
    '''
        <input>
        data = [[a1,a2,a3], [b1,b2,b3], [c1,c2,c3], ...]
        columns = [A, B, C, ...]
        <output>
        df =
            A  B  C
        0   a1 b1 c1
        1   a2 b2 c2
        2   a3 b3 c3
    '''
    assert len(data) == len(columns), 'Must be same with lengths of data and columns'

    # find minimum lengh of lists
    dict_ = {}
    min_idx = 1000000
    for idx, list_ in enumerate(data):
        if len(list_) <= min_idx:
            min_idx = len(list_)
        raw_data = np.array(list_)
        dict_[columns[idx]] = raw_data * 10

    # make pandas frame
    for idx, list_ in enumerate(data):
        dict_[columns[idx]] = dict_[columns[idx]][:min_idx]
    df = pd.DataFrame(dict_)
    return df
