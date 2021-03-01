#ifndef TEST_PURE_TDMA_H
#define TEST_PURE_TDMA_H




typedef nx_struct TdmaMsg
{
  nx_uint16_t ParentID;
	nx_uint16_t NodeID;
  nx_uint16_t SequenceNo;
	nx_uint8_t Data;

} TdmaMsg_t;

typedef nx_struct test_ftsp_msg
{
  nx_uint16_t    src_addr;
  nx_uint16_t    counter;
  nx_uint32_t    local_rx_timestamp;
  nx_uint32_t    global_rx_timestamp;
  nx_int32_t     skew_times_1000000;
  nx_uint8_t     is_synced;
  nx_uint16_t    ftsp_root_addr;
  nx_uint8_t     ftsp_seq;
  nx_uint8_t     ftsp_table_entries;
} test_ftsp_msg_t;


#endif /* TEST_PURE_TDMA_H */
