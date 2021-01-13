

interface AdaptiveHART_Util {

  async command uint8_t get_release_offset(uint8_t nodeid);
  async command uint8_t get_flow_id(uint8_t nodeid);
  async command bool is_root(uint8_t nodeid);
}
