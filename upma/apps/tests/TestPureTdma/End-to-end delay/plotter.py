import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import seaborn as sns
import pandas as pd
# from normal_oper_check import get_cdf


def show_delay_distribution(AH_retx_delay_distribution, WH_retx_delay_distribution):
    '''
        input:
            AH_retx_delay_distribution = [[HI_TASK delays], [LO_TASK delays]]
    '''
    obj_len = len(AH_retx_delay_distribution)
    colors = ['b', 'r', 'g', 'c']
    line_style = ['solid', 'dashed','dotted']
    AH_legends = ['AH_HI', 'AH_LO']
    WH_legends = ['WH_HI', 'WH_LO']


    df_AH = make_pandas_frame(AH_retx_delay_distribution, AH_legends)
    df_WH = make_pandas_frame(WH_retx_delay_distribution, WH_legends)

    for idx in range(len(df_AH.columns)):
        fig = sns.ecdfplot(df_AH[AH_legends[idx]],  color='red', ls=line_style[idx], label=AH_legends[idx])
    for idx in range(len(df_WH.columns)):
        sns.ecdfplot(df_WH[WH_legends[idx]],  color='blue', ls=line_style[idx], label=WH_legends[idx])

    plt.ylim(0, 1.1)
    plt.xlim(0, 250)
    plt.ylabel('ECDF')
    plt.xlabel('End-to-end delay (ms)')
    plt.legend()
    plt.grid()
    plt.show()


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
