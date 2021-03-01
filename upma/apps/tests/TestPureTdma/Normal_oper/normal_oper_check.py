import numpy as np
import plotter
import random
import matplotlib.pyplot as plt

SameSlot_node3_path = "./Normal_channel_log/node3_sameSlot.log"
SameSlot_node5_path = "./Normal_channel_log/node5_sameSlot.log"
SameSlot_node6_path = "./Normal_channel_log/node6_sameSlot.log"
SameSlot_node7_path = "./Normal_channel_log/node7_sameSlot.log"

HILate1_node3_path = "./Normal_channel_log/node3_HIlate1.log"
HILate1_node5_path = "./Normal_channel_log/node5_HIlate1.log"
HILate1_node6_path = "./Normal_channel_log/node6_HIlate1.log"
HILate1_node7_path = "./Normal_channel_log/node7_HIlate1.log"

LOLate1_node3_path = "./Normal_channel_log/node3_LOlate1.log"
LOLate1_node5_path = "./Normal_channel_log/node5_LOlate1.log"
LOLate1_node6_path = "./Normal_channel_log/node6_LOlate1.log"
LOLate1_node7_path = "./Normal_channel_log/node7_LOlate1.log"

HILate2_node3_path = "./Normal_channel_log/node3_HIlate2.log"
HILate2_node5_path = "./Normal_channel_log/node5_HIlate2.log"
HILate2_node6_path = "./Normal_channel_log/node6_HIlate2.log"
HILate2_node7_path = "./Normal_channel_log/node7_HIlate2.log"

LOLate2_node3_path = "./Normal_channel_log/node3_LOlate2.log"
LOLate2_node5_path = "./Normal_channel_log/node5_LOlate2.log"
LOLate2_node6_path = "./Normal_channel_log/node6_LOlate2.log"
LOLate2_node7_path = "./Normal_channel_log/node7_LOlate2.log"

HI_TASK = 0
LO_TASK = 1


def receive_slots(path, nodeid):
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
    delays_ratio = {delay1: prob1, delay2: prob2, ...}
    '''

    num_total_pkt = len(rcv_slots)
    tx_delays = [int(rcv_slot)-root_start if int(rcv_slot)-root_start >= 0 else -1 for rcv_slot in rcv_slots ]
    delays_ratio = ratio_per_delay(tx_delays)
    return delays_ratio


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

def get_flow_delay_distribution(path_list, node_list, flow_id, root_start):
    '''
    output:
        delay distribution of the flow nodes
    '''
    node_delays_ratio = {}
    for idx in range(len(path_list)):
        rcv_slots = receive_slots(path_list[idx], node_list[idx])
        delays_ratio = from_slot_to_delay(rcv_slots[flow_id], root_start)
        node_delays_ratio[node_list[idx]] = remove_loss_data(delays_ratio)
    return node_delays_ratio

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
    HI_node_list = [3,5,6]
    LO_node_list = [3,5,7]

    ### Same slot data ###
    SameSlot_base_slot = 6
    SameSlot_HI_root_start = 6
    SameSlot_LO_root_start = 6

    SameSlot_HI_file_list = [SameSlot_node3_path, SameSlot_node5_path, SameSlot_node6_path]
    SameSlot_LO_file_list = [SameSlot_node3_path, SameSlot_node5_path, SameSlot_node7_path]

    SameSlot_HI_flow_delay_distribution = get_flow_delay_distribution(SameSlot_HI_file_list, HI_node_list, HI_TASK, SameSlot_base_slot)
    SameSlot_LO_flow_delay_distribution = get_flow_delay_distribution(SameSlot_LO_file_list, LO_node_list, LO_TASK, SameSlot_base_slot)

    # SameSlot_Highest_HI_flow_delay_distribution = extract_highest_data(SameSlot_HI_flow_delay_distribution)
    # SameSlot_Highest_LO_flow_delay_distribution = extract_highest_data(SameSlot_LO_flow_delay_distribution)

    print("SameSlot_HI_flow_delay_distribution:", SameSlot_HI_flow_delay_distribution)
    # print("[dbg]SameSlot_Highest_HI_flow_delay_distribution:", SameSlot_Highest_HI_flow_delay_distribution)
    print("SameSlot_LO_flow_delay_distribution:", SameSlot_LO_flow_delay_distribution)
    # print("[dbg]SameSlot_Highest_LO_flow_delay_distribution:", SameSlot_Highest_LO_flow_delay_distribution)



    ### HI_Task delay 1 slot data ###
    HILate1_base_slot = 5
    HILate1_HI_root_start = 6
    HILate1_LO_root_start = 5

    HILate1_HI_file_list = [HILate1_node3_path, HILate1_node5_path, HILate1_node6_path]
    HILate1_LO_file_list = [HILate1_node3_path, HILate1_node5_path, HILate1_node7_path]

    HILate1_HI_flow_delay_distribution = get_flow_delay_distribution(HILate1_HI_file_list, HI_node_list, HI_TASK, HILate1_base_slot)
    HILate1_LO_flow_delay_distribution = get_flow_delay_distribution(HILate1_LO_file_list, LO_node_list, LO_TASK, HILate1_base_slot)

    # HILate1_Highest_HI_flow_delay_distribution = extract_highest_data(HILate1_HI_flow_delay_distribution)
    # HILate1_Highest_LO_flow_delay_distribution = extract_highest_data(HILate1_LO_flow_delay_distribution)

    print("HILate1_HI_flow_delay_distribution:", HILate1_HI_flow_delay_distribution)
    # print("[dbg]HILate1_Highest_HI_flow_delay_distribution:", HILate1_Highest_HI_flow_delay_distribution)
    print("HILate1_LO_flow_delay_distribution:", HILate1_LO_flow_delay_distribution)
    # print("[dbg]HILate1_Highest_LO_flow_delay_distribution:", HILate1_Highest_LO_flow_delay_distribution)


    ### LO_Task delay 1 slot data ###
    LOLate1_base_slot = 5
    LOLate1_HI_root_start = 5
    LOLate1_LO_root_start = 6

    LOLate1_HI_file_list = [LOLate1_node3_path, LOLate1_node5_path, LOLate1_node6_path]
    LOLate1_LO_file_list = [LOLate1_node3_path, LOLate1_node5_path, LOLate1_node7_path]

    LOLate1_HI_flow_delay_distribution = get_flow_delay_distribution(LOLate1_HI_file_list, HI_node_list, HI_TASK, LOLate1_base_slot)
    LOLate1_LO_flow_delay_distribution = get_flow_delay_distribution(LOLate1_LO_file_list, LO_node_list, LO_TASK, LOLate1_base_slot)

    # LOLate1_Highest_HI_flow_delay_distribution = extract_highest_data(LOLate1_HI_flow_delay_distribution)
    # LOLate1_Highest_LO_flow_delay_distribution = extract_highest_data(LOLate1_LO_flow_delay_distribution)

    print("LOLate1_HI_flow_delay_distribution:", LOLate1_HI_flow_delay_distribution)
    # print("[dbg]LOLate1_Highest_HI_flow_delay_distribution:", LOLate1_Highest_HI_flow_delay_distribution)
    print("LOLate1_LO_flow_delay_distribution:", LOLate1_LO_flow_delay_distribution)
    # print("[dbg]LOLate1_Highest_LO_flow_delay_distribution:", LOLate1_Highest_LO_flow_delay_distribution)
    # plotter.show_adaptivity(adaptive_intervals_set)


    ### HI_Task delay 1 slot data ###
    HILate2_base_slot = 5
    HILate2_HI_root_start = 7
    HILate2_LO_root_start = 5

    HILate2_HI_file_list = [HILate2_node3_path, HILate2_node5_path, HILate2_node6_path]
    HILate2_LO_file_list = [HILate2_node3_path, HILate2_node5_path, HILate2_node7_path]

    HILate2_HI_flow_delay_distribution = get_flow_delay_distribution(HILate2_HI_file_list, HI_node_list, HI_TASK, HILate2_base_slot)
    HILate2_LO_flow_delay_distribution = get_flow_delay_distribution(HILate2_LO_file_list, LO_node_list, LO_TASK, HILate2_base_slot)


    ### LO_Task delay 2 slot data ###
    LOLate2_base_slot = 5
    LOLate2_HI_root_start = 5
    LOLate2_LO_root_start = 7

    LOLate2_HI_file_list = [LOLate2_node3_path, LOLate2_node5_path, LOLate2_node6_path]
    LOLate2_LO_file_list = [LOLate2_node3_path, LOLate2_node5_path, LOLate2_node7_path]

    LOLate2_HI_flow_delay_distribution = get_flow_delay_distribution(LOLate2_HI_file_list, HI_node_list, HI_TASK, LOLate2_base_slot)
    LOLate2_LO_flow_delay_distribution = get_flow_delay_distribution(LOLate2_LO_file_list, LO_node_list, LO_TASK, LOLate2_base_slot)

    ### show figures ###
    plotter.show_delay_distribution([SameSlot_HI_flow_delay_distribution, SameSlot_LO_flow_delay_distribution])
    plotter.show_delay_distribution([HILate1_HI_flow_delay_distribution, HILate1_LO_flow_delay_distribution])
    plotter.show_delay_distribution([LOLate1_HI_flow_delay_distribution, LOLate1_LO_flow_delay_distribution])
    plotter.show_delay_distribution([HILate2_HI_flow_delay_distribution, HILate2_LO_flow_delay_distribution])
    plotter.show_delay_distribution([LOLate2_HI_flow_delay_distribution, LOLate2_LO_flow_delay_distribution])
    plt.show()
