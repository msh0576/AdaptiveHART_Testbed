#ifndef TEST_PURE_TDMA_H
#define TEST_PURE_TDMA_H


#define MAX_CHILDREN 3
#define MAX_NODES 10
#define MAX_NEI 10
#define NETWORK_FLOW 3		// If there are 2 flows in the network, it should be 2+1
#define MAX_NODEID 8			// Same as the above
#define HI_TASK 1
#define LO_TASK 2

typedef nx_struct TdmaMsg
{
  //nx_uint16_t ParentID;
	//nx_uint16_t NodeID;
  nx_uint16_t SequenceNo;
	nx_uint16_t Data;
  nx_uint8_t Schedule[20];    //***
  nx_uint8_t Flowid;
  //nx_uint8_t Schedule1[11];   //***
  //nx_uint8_t Schedule2[11];   //***
} TdmaMsg_t;

typedef nx_struct adaptivehart_msg
{
	nx_uint8_t flowid;
	nx_uint8_t priority;
	nx_uint8_t maxtx;
	nx_uint16_t deadline;
	nx_uint16_t seq;
	nx_uint8_t is_changed;
} adaptivehart_msg_t;

#endif /* TEST_PURE_TDMA_H */
