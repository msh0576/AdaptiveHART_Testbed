import numpy as np
import plotter
import random

path1 = "./node2_frame200_from15to20_AH.log"
path2 = "./node2_frame200_from15to10_AH.log"
path3 = "./node2_frame200_from15to25_AH.log"
useful_idx_begin = 5
useful_idx_end = 13
HI_TASK = 1
LO_TASK = 2
random_slots = [19, 21, 22, 25, 28, 35, 39, 43, 46, 52]
random_slot = [0, 0, 43]
changed_period = [0, 20, 10, 25]    # index: changed period of file_idx
tx_opper = [0, 2, 2]
e2e_hop = 4

def check_period(tx_slot_list):
    '''
    input: a list [1, 3, 6, 8]
    output: a list [2, 3, 2] : a period between two vales in the input list
    '''
    output = []
    tmp_tx_slot = int(tx_slot_list[0])
    for tx_slot in tx_slot_list[1:]:
        period = int(tx_slot) - tmp_tx_slot
        tmp_tx_slot = int(tx_slot)
        output.append(period)
    return output

def find_adaptive_interval(rx_slots, tx_intervals, task_id, changed_slot, changed_period):
    '''
    output: an interval from a changed instant to the first instant that gets the expected period
    '''
    adaptive_interval = 0
    for idx, val in enumerate(tx_intervals):
        if val >= changed_period - tx_opper[task_id] and val <= changed_period:
            assert idx < len(tx_intervals)-1, 'Need more printing'
            adaptive_interval = int(rx_slots[idx+1]) - changed_slot
            break
    return adaptive_interval


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



def get_adaptive_intervals(path, file_idx):
    '''
    output: adaptive interval set
    '''
    adaptive_intervals = []
    with open(path, 'r') as file:
        lines = file.readlines()
        random.seed(0)
        for line in lines:
            line = line.replace(',','')
            line_list = line.split(' ')

            # check useful line
            if line_list[0] == 'random_slot:':
                period_change_slot = int(line_list[1])
                tx_slots = line_list[useful_idx_begin : useful_idx_end+1]
                rx_slots = insert_trans_delay(tx_slots)
                tx_intervals = check_period(tx_slots)
                adaptive_interval = find_adaptive_interval(rx_slots, tx_intervals, LO_TASK, period_change_slot, changed_period[file_idx])
                # print('period_change_slot: ', period_change_slot)
                # print('tx_slots',tx_slots)
                # print('rx_slots',rx_slots)
                # print('tx_intervals',tx_intervals)
                # print('adaptive_interval',adaptive_interval)
                adaptive_intervals.append(adaptive_interval)
    return adaptive_intervals

def insert_trans_delay(tx_slots):
    '''
    transmission delay is plused to the tx_slot
    '''
    loss_prob = 0.05

    rx_slots = []
    for tx_slot in tx_slots:
        rand = random.random()
        delay = get_delay(rand, loss_prob)
        rx_slot = int(tx_slot) + delay
        rx_slots.append(rx_slot)
    return rx_slots

def get_delay(rand, loss_prob):
    if rand <= loss_prob**4:
        delay = e2e_hop + 4
    elif rand > loss_prob**4 and rand <= loss_prob**3:
        delay = e2e_hop + 3
    elif rand > loss_prob**3 and rand <= loss_prob**2:
        delay = e2e_hop + 2
    elif rand > loss_prob**2 and rand <= loss_prob**1:
        delay = e2e_hop + 1
    else:
        delay = e2e_hop
    return delay


if __name__ == "__main__":
    adaptive_intervals_set = []
    adaptive_intervals_file1 = get_adaptive_intervals(path1, 1)
    adaptive_intervals_file2 = get_adaptive_intervals(path2, 2)
    adaptive_intervals_file3 = get_adaptive_intervals(path3, 3)

    adaptive_intervals_set.append(adaptive_intervals_file1)
    adaptive_intervals_set.append(adaptive_intervals_file2)
    adaptive_intervals_set.append(adaptive_intervals_file3)

    plotter.show_adaptivity(adaptive_intervals_set)
