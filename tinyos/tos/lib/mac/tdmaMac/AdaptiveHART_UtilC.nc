
#include "TdmaMac.h"

module AdaptiveHART_UtilC {
  provides interface AdaptiveHART_Util;
}
implementation {

  /* flow release info */
  // 0: flow id
  // 1: flow root node
  // 2: release offset time
  // 3: flow destination
  uint8_t flow_info[NETWORK_FLOW][4] = {
    {0, 0, 0, 0},
    {1, 1, 3, 6},
    {2, 2, 2, 7}
  };

  uint8_t HI_routing_map[MAX_NODEID] = {0, 3, 0, 5, 0, 6, 0, 0};
  uint8_t LO_routing_map[MAX_NODEID] = {0, 0, 3, 5, 0, 7, 0, 0};

  uint8_t Offset_1 = 0; //ms
  uint8_t Offset_2 = 3;
  uint8_t Offset_3 = 7;

  /* Functions */
  bool has_LS_type(uint8_t nodeid);

  async command uint8_t AdaptiveHART_Util.get_release_offset(uint8_t nodeid){
    uint8_t i;
    uint8_t release_offset = 0;

    for(i=1; i<NETWORK_FLOW; i++){
      if(flow_info[i][1] == nodeid){
        release_offset = flow_info[i][2];
        break;
      }
    }
    return release_offset;
  }

  async command uint8_t AdaptiveHART_Util.get_flow_id(uint8_t nodeid){
    uint8_t i;
    uint8_t flow_id = 0;

    for(i=1; i<NETWORK_FLOW; i++){
      if(flow_info[i][1] == nodeid){
        flow_id = flow_info[i][0];
        break;
      }
    }
    return flow_id;
  }


  async command bool AdaptiveHART_Util.is_root(uint8_t nodeid){
    uint8_t i;
    bool result = FALSE;

    for(i=1; i<NETWORK_FLOW; i++){
      if(flow_info[i][1] == nodeid){
        result = TRUE;
        break;
      }
    }
    return result;
  }

  async command bool AdaptiveHART_Util.is_dest(uint8_t flowid, uint8_t nodeid){
    bool result = FALSE;
    if(flow_info[flowid][3] == nodeid){
      result = TRUE;
    }
    return result;
  }

  async command uint8_t AdaptiveHART_Util.set_destination(uint8_t flowid, uint8_t nodeid){
    uint8_t dest;
    if(flowid == HI_TASK){
      dest = HI_routing_map[nodeid];
    }else if(flowid == LO_TASK){
      dest = LO_routing_map[nodeid];
    }else{
      dest = 100;
    }
    return dest;
  }

  /* return a TxOffset of a flow node, considering its possible conflit types */
  async command uint8_t AdaptiveHART_Util.set_TxOffset(uint8_t flowid, uint8_t nodeid){


    if(flowid == HI_TASK){
        return Offset_1;
    }else if(flowid == LO_TASK){
      if(has_LS_type(nodeid) == TRUE){
        return Offset_3;
      }else{
        return Offset_2;
      }
    }else{
      return 0;
    }
  }

  /* return true or false, when the input node has LS (Low-flow sender) conflict type */
  bool has_LS_type(uint8_t nodeid){
    uint8_t i, hi_flow_root, lo_flow_root, dest, sender;
    uint8_t hi_receiver = 0;
    bool result = FALSE;

    hi_flow_root = flow_info[HI_TASK][1];
    lo_flow_root = flow_info[LO_TASK][1];

    // check nodeid is a receiver of high-flow
    for(i=hi_flow_root; i<MAX_NODEID; i++){
      dest = HI_routing_map[i];
      if(dest != 0 && dest == nodeid){
        hi_receiver = nodeid;
        break;
      }
    }

    // check nodeid is a sender of low-flow
    for(sender=lo_flow_root; sender<MAX_NODEID; sender++){
      dest = HI_routing_map[sender];
      if(dest != 0 && sender == nodeid && sender == hi_receiver){
        result = TRUE;
        break;
      }
    }

    return result;
  }


}
