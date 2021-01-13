
#include "TdmaMac.h"

module AdaptiveHART_UtilC {
  provides interface AdaptiveHART_Util;
}
implementation {

  /* flow release info */
  // 0: flow id
  // 1: flow root node
  // 2: release offset time
  uint8_t flow_info[NETWORK_FLOW][3] = {
    {0, 0, 0},
    {1, 1, 1},
    {2, 2, 2}
  };

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


}
