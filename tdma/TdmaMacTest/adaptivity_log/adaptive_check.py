import numpy as np
import plotter
import random
frame_len = '200'
WH_giga_len = 200
log_file_path = './Frame'+frame_len+'_log'

path1_AH = log_file_path + "/node2_frame"+frame_len+"_from15to10_AH.log"
path2_AH = log_file_path + "/node2_frame"+frame_len+"_from15to20_AH.log"
path3_AH = log_file_path + "/node2_frame"+frame_len+"_from15to25_AH.log"

path1_WH = log_file_path + "/node2_frame"+frame_len+"_from15to10_WH.log"
path2_WH = log_file_path + "/node2_frame"+frame_len+"_from15to20_WH.log"
path3_WH = log_file_path + "/node2_frame"+frame_len+"_from15to25_WH.log"

useful_idx_begin = 4
useful_idx_end = 13
HI_TASK = 1
LO_TASK = 2
random_slots = [19, 21, 22, 25, 28, 35, 39, 43, 46, 52]
random_slot = [0, 0, 43]
changed_period = [0, 10, 20, 25]    # index: changed period of file_idx
tx_opper = [0, 2, 2]
e2e_hop = 3


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

def find_adaptive_interval_WH(changed_tx_slot, period_change_slot, giga_len):
    '''
        <output>
        if changed_tx_slot = 2, period_change_slot = 11, giga_len = 200,
        then the output : 200 - 11 + 2
    '''
    return giga_len - period_change_slot + changed_tx_slot

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

def insert_trans_delay_WH(tx_slot, e2e_hop, retx_per_hop):
    '''
        <output>
        end-to-end delay in WirelessHART
        loss_delay: last hop delay due to pkt loss
    '''
    hop_delay = e2e_hop * retx_per_hop

    loss_prob = 0.05
    rand = random.random()
    if rand <= loss_prob**2:
        loss_delay = 2
    elif rand > loss_prob**2 and rand <= loss_prob**1:
        loss_delay = 1
    else:
        loss_delay = 0
    return tx_slot + hop_delay + loss_delay


def get_delay(rand, loss_prob):
    if rand <= loss_prob**3:
        delay = e2e_hop + 3
    elif rand > loss_prob**3 and rand <= loss_prob**2:
        delay = e2e_hop + 2
    elif rand > loss_prob**2 and rand <= loss_prob**1:
        delay = e2e_hop + 1
    else:
        delay = e2e_hop
    return delay

def get_adaptive_intervals_WH(path, giga_len):
    '''
    output: adaptive interval set
    '''
    adaptive_intervals = []
    with open(path, 'r') as file:
        lines = file.readlines()
        random.seed(0)
        origin_sched_line = False
        changed_sched_line = False
        for line in lines:
            line = line.replace(',','')
            line_list = line.split(' ')
            # check useful line
            if line_list[0] == 'schedule_reset:':
                origin_sched_line = True
            elif line_list[0] == 'schedule_changed:':
                changed_sched_line = True

            if line_list[0] == 'random_slot:' and origin_sched_line == True:
                origin_sched_line = False
                origin_tx_slot = int(line_list[4])
            elif line_list[0] == 'random_slot:' and changed_sched_line == True:
                changed_sched_line = False
                period_change_slot = int(line_list[1])
                changed_tx_slot = int(line_list[4])
                rx_slot = insert_trans_delay_WH(changed_tx_slot, e2e_hop, retx_per_hop=2)
                adaptive_interval = find_adaptive_interval_WH(rx_slot, period_change_slot, giga_len)
                adaptive_intervals.append(adaptive_interval)
    return adaptive_intervals

if __name__ == "__main__":

    ### AdaptiveHART ###
    adaptive_intervals_set = []
    adaptive_intervals_file1 = get_adaptive_intervals(path1_AH, 1)
    adaptive_intervals_file2 = get_adaptive_intervals(path2_AH, 2)
    adaptive_intervals_file3 = get_adaptive_intervals(path3_AH, 3)

    adaptive_intervals_set.append(adaptive_intervals_file1)
    adaptive_intervals_set.append(adaptive_intervals_file2)
    adaptive_intervals_set.append(adaptive_intervals_file3)

    ### WirelessHART ###
    WH_adaptive_intervals_set = []
    adaptive_intervals_WH_file1 = get_adaptive_intervals_WH(path1_WH, giga_len=WH_giga_len)
    adaptive_intervals_WH_file2 = get_adaptive_intervals_WH(path2_WH, giga_len=WH_giga_len)
    adaptive_intervals_WH_file3 = get_adaptive_intervals_WH(path3_WH, giga_len=WH_giga_len)

    WH_adaptive_intervals_set.append(adaptive_intervals_WH_file1)
    WH_adaptive_intervals_set.append(adaptive_intervals_WH_file2)
    WH_adaptive_intervals_set.append(adaptive_intervals_WH_file3)


    ### show graph ###
    plotter.show_adaptivity(adaptive_intervals_set, WH_adaptive_intervals_set)
