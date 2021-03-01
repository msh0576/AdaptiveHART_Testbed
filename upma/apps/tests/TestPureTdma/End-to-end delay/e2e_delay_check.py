import numpy as np
import plotter
import random
import matplotlib.pyplot as plt

AH_node6_retx2_path = "./AH_node6_retx2.log"
AH_node6_retx3_path = "./AH_node6_retx3.log"
AH_node6_retx4_path = "./AH_node6_retx4.log"
AH_node7_retx2_path = "./AH_node7_retx2.log"
AH_node7_retx3_path = "./AH_node7_retx3.log"
AH_node7_retx4_path = "./AH_node7_retx4.log"

WH_node6_retx2_path = "./WH_node6_retx2.log"
WH_node6_retx3_path = "./WH_node6_retx3.log"
WH_node6_retx4_path = "./WH_node6_retx4.log"
WH_node7_retx2_path = "./WH_node7_retx2.log"
WH_node7_retx3_path = "./WH_node7_retx3.log"
WH_node7_retx4_path = "./WH_node7_retx4.log"

HI_TASK = 0
LO_TASK = 1


def receive_slots(path):
    '''
    Read received slots info in the file with respect to flow_id
    '''
    rcv_slots = {
                HI_TASK: [],
                LO_TASK: []
                }
    with open(path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            line = line.replace(',','')
            line_list = line.split(' ')

            if line_list[0] == 'HI_TASK:':
                rcv_slots[HI_TASK].append(int(line_list[3]))
            elif line_list[0] == 'LO_TASK:':
                rcv_slots[LO_TASK].append(int(line_list[3]))
    return rcv_slots

def from_slot_to_delay(rcv_slots, root_start):
    '''
    compute transmission delay distribution of a flow and a node;
    '''

    num_total_pkt = len(rcv_slots)
    tx_delays = [int(rcv_slot)-root_start if int(rcv_slot)-root_start >= 0 else -1 for rcv_slot in rcv_slots ]
    return tx_delays


def ratio_per_delay(tx_delays):
    '''
    For a node and a flow, counts each tx delay in the tx_delays
    '''
    total_num_pkt = len(tx_delays)
    delays_ratio = {}
    delay_types = list(set(tx_delays))
    for delay in delay_types:
        delay_cnt = tx_delays.count(delay)
        if round(delay_cnt/total_num_pkt,2) > 0.0:
            delays_ratio[delay] = round(delay_cnt/total_num_pkt,2)
    # print('flow %s delays_ratio: %s'%(flowid, delays_ratio))
    return delays_ratio

def get_flow_e2eDelay_distribution(path_list, flows_id, root_start, expect_fastest_rcvslots):
    '''
    output:
        e2e delay distribution
    '''
    node_delays_set = []
    for idx in range(len(path_list)):
        rcv_slots = receive_slots(path_list[idx])
        rcv_slots_ = remove_dummy_data(rcv_slots[flows_id[idx]], expect_fastest_rcvslots[idx])
        tx_delays = from_slot_to_delay(rcv_slots_, root_start)
        node_delays_set.append(tx_delays)
    return node_delays_set

def remove_dummy_data(rcv_slots, expect_fastest_rcvslot):
    '''
        <output>
        remove some data that received prior than the expect_fastest_rcvslot
    '''
    output = [slot for slot in rcv_slots if slot >= expect_fastest_rcvslot]
    return output


def remove_loss_data(delays_ratio):
    '''
    input:
        delays_ratio = {delay1: prob1, delay2: prob2, ...}
    output:
        get rid of -1 data (loss data) from the input
    '''
    output = {}
    for key, value in delays_ratio.items():
        if key > -1:
            output[key] = value
    return output


def extract_highest_data(flow_delay_distribution):
    '''
        input:
            flow_delay_dist = {nodeid: {delay1: prob1, delay2: prob2, ...}, ...}
        output:
            highest_delay_dist = {nodeid: {delayX: probX}, ...}
    '''
    highest_flow_delay_dist = {}
    for nodeid, delay_distributions in flow_delay_distribution.items():
        highest_delay = {}
        v_list = list(delay_distributions.values())
        k_list = list(delay_distributions.keys())
        keyMax = k_list[v_list.index(max(v_list))]
        highest_delay[keyMax] = delay_distributions[keyMax]
        highest_flow_delay_dist[nodeid] = highest_delay
    return highest_flow_delay_dist

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
    return [x_values, y_values]

if __name__ == "__main__":

    flow_list = [HI_TASK, LO_TASK]

    AH_root_start = 5

    AH_retx2_except_fastest_rcvslots = [7, 9]
    AH_retx3_except_fastest_rcvslots = [7, 9]
    AH_retx4_except_fastest_rcvslots = [7, 9]

    WH_root_start = 2

    WH_retx2_except_fastest_rcvslots = [6, 12]
    WH_retx3_except_fastest_rcvslots = [8, 17]
    WH_retx4_except_fastest_rcvslots = [10, 22]

    AH_retx2_e2eDelay_paths = [AH_node6_retx2_path, AH_node7_retx2_path]
    AH_retx3_e2eDelay_paths = [AH_node6_retx3_path, AH_node7_retx3_path]
    AH_retx4_e2eDelay_paths = [AH_node6_retx4_path, AH_node7_retx4_path]

    WH_retx2_e2eDelay_paths = [WH_node6_retx2_path, WH_node7_retx2_path]
    WH_retx3_e2eDelay_paths = [WH_node6_retx3_path, WH_node7_retx3_path]
    WH_retx4_e2eDelay_paths = [WH_node6_retx4_path, WH_node7_retx4_path]

    AH_retx2_Delay_dist = get_flow_e2eDelay_distribution(AH_retx2_e2eDelay_paths, flow_list, AH_root_start, AH_retx2_except_fastest_rcvslots)
    AH_retx3_Delay_dist = get_flow_e2eDelay_distribution(AH_retx3_e2eDelay_paths, flow_list, AH_root_start, AH_retx3_except_fastest_rcvslots)
    AH_retx4_Delay_dist = get_flow_e2eDelay_distribution(AH_retx4_e2eDelay_paths, flow_list, AH_root_start, AH_retx4_except_fastest_rcvslots)

    WH_retx2_Delay_dist = get_flow_e2eDelay_distribution(WH_retx2_e2eDelay_paths, flow_list, WH_root_start, WH_retx2_except_fastest_rcvslots)
    WH_retx3_Delay_dist = get_flow_e2eDelay_distribution(WH_retx3_e2eDelay_paths, flow_list, WH_root_start, WH_retx3_except_fastest_rcvslots)
    WH_retx4_Delay_dist = get_flow_e2eDelay_distribution(WH_retx4_e2eDelay_paths, flow_list, WH_root_start, WH_retx4_except_fastest_rcvslots)

    # AH_Delay_dist = [AH_retx2_Delay_dist, AH_retx3_Delay_dist, AH_retx4_Delay_dist]

    ### show figures ###
    plotter.show_delay_distribution(AH_retx2_Delay_dist, WH_retx2_Delay_dist)
    plotter.show_delay_distribution(AH_retx3_Delay_dist, WH_retx3_Delay_dist)
    plotter.show_delay_distribution(AH_retx4_Delay_dist, WH_retx4_Delay_dist)
