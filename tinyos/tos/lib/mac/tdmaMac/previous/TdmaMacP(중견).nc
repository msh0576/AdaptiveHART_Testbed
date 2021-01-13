//**************************************************************************
// * file:        TdmaMac implementation module
// *
// * author:      A. Ajith Kumar S.
// * copyright:   (c) A. Ajith Kumar S.
// * homepage:    www.hib.no/ansatte/aaks
// * email:       aji3003 @ gmail.com
// **************************************************************************
// * part of:     TinyOS MAC tutorial.
// * Refined on:  26-June-2015
// **************************************************************************
// *This file is part of TinyOS MAC tutorial.
// *
// *TinyOS MAC tutorial is free software: you can redistribute it and/or modify
// *it under the terms of the GNU General Public License as published by
// *the Free Software Foundation, either version 3 of the License, or
// *(at your option) any later version.
// *
// *TinyOS MAC tutorial is distributed in the hope that it will be useful,
// *but WITHOUT ANY WARRANTY; without even the implied warranty of
// *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// *GNU General Public License for more details.
// *
// *You should have received a copy of the GNU General Public License
// *along with TinyOS MAC tutorial.  If not, see <http://www.gnu.org/licenses/>./
// **************************************************************************

//#define NEW_PRINTF_SEMANTICS
#include "printf.h"
#include "TdmaMac.h"
#include "Timer.h"

#define PACKET_TIME_32HZ 		37

module TdmaMacP {

	provides { //General purpose
		interface Init;
		interface FrameConfiguration as Frame;
	}

	provides { // Radio Control
		interface Send as MacSend;
		interface Receive as MacReceive;
		interface SplitControl as MacPowerControl;
	}

	uses { //General purpose
		interface Boot;
	}

	uses { //MAC Stuff
		interface AMPacket;
		interface Packet;
		//interface CC2420Packet as linkIndicator;
		interface AsyncStdControl as GenericSlotterPowerControl;
		interface Slotter;
		interface SlotterControl;
		interface FrameConfiguration;
		interface Queue<RefData_t*> as forwardQueue;
		interface Queue<notification_t*> as forwardNotifyQueue;
		interface PacketAcknowledgements as ACK;
	}

	uses { //Radio Control
		interface SplitControl as RadioPowerControl;
		interface AMSend as phyNotificationSend;
		interface Receive as phyNotificationReceive;
		interface AMSend as phyDataSend;
		interface Receive as phyDataReceive;
	}

	uses {//Time sync stuff from Ftps library
		interface GlobalTime<TMilli> as GlobalTime;
		interface TimeSyncInfo;
		interface TimeSyncMode;
	}

	uses interface Alarm<T32khz, uint32_t> as SyncAlarm;
	//uses interface Alarm<T32khz, uint32_t> as ReferenceAlarm;
	uses interface Timer<TMilli> as ReferenceAlarm;
	uses interface Leds;

}

implementation {

	/** @brief Radio operation variables*/
	bool init;
	bool requestStop;
	bool radioOff;
	bool phyLock = FALSE; 			// Radio lock to prevent double send

	/** @brief Time sync info */
	uint32_t rxTimestamp ;
	/** @brief Time sync info */
	uint32_t myLocalTime,myGlobalTime,myOffset,mySkew;
	uint32_t myGlobalTime2;

	uint16_t rcv_cunt = 0;
	uint16_t send_cunt = 0;
	uint16_t enQ_cunt = 0;
	uint16_t TxseqNum = 0;
	uint32_t flowSeq = 0;

	uint8_t value;
	uint8_t inSync; 		// Is the node synchronized on time with sink

	/** @brief Message to transmit From TestAcks */
	message_t forwardPktBuffer[MAX_CHILDREN];
	message_t forwardNotifyPktBuffer[MAX_CHILDREN];
	message_t selfDataPkt;
	message_t selfNotificationPkt;
	message_t selfSyncPkt;
	message_t referencePkt;
	message_t rootPayload;

	/** @brief Payload parts for packets */
	notification_t* notificationPacket;
	dmamac_data_t* dataPacket;
	RefData_t* forwardPayload;

	/** @brief Slot mechanism and superframe */
	bool slotterInit ;		// Slotter initialized after time-synchronized
	uint32_t slotSize;
	uint8_t currentSlot;

	uint8_t toSlot;
	uint32_t tempGlobal;

	uint8_t superFrameLength,currFrameSize;


	uint16_t sequenceCount;
	uint8_t queueSize;
	uint16_t nbReceivedPkts[MAX_FLOWS][MAX_NODES];
	uint16_t nbSentPkts;

	am_addr_t src;			// Sender source
	am_addr_t realSrc;		// Base source (origin)
	uint8_t i;
	uint8_t outputPower;	// TX power
	int16_t rssi;			// Received signal strength

	uint8_t Time_stemp_src_node = 1;
	uint8_t Num_ReTx = 1;	// minimum is 1
	uint32_t first_Tx_slot = 0;
	uint8_t my_network_id = 1;
	uint8_t curr_network_id = 0;
	// for reference change
	uint8_t per_frame_flow_idx = 0;
	uint8_t per_frame_num_flow = 1;

	uint8_t currRef1 = 2;
	uint8_t currRef2 = 2;
	uint8_t currRef3 = 2;
	uint8_t currRef4 = 2;
	uint8_t currRef5 = 2;

	uint8_t dbg_currRef1 = 0;
	uint8_t dbg_currRef1_idx = 0;
	uint8_t dbg_currRef2 = 0;
	uint8_t dbg_currRef2_idx = 0;
	uint8_t dbg_currRef3 = 0;
	uint8_t dbg_currRef3_idx = 0;
	uint8_t dbg_currRef4 = 0;
	uint8_t dbg_currRef4_idx = 0;
	uint8_t dbg_currRef5 = 0;
	uint8_t dbg_currRef5_idx = 0;

	/* Prototypes */
	void sendPkt(RefData_t * packet, uint8_t destination);
	void generatePayload(RefData_t * packet);
	void loadPayload(RefData_t * forwardPacket, RefData_t * queuePacket);
	uint8_t get_multiFlow_mapping(uint8_t slot);
	uint8_t get_singleFlow_mapping(uint8_t slot);

	/** @briedf scheduling matrix for all nodes
	flownum , currentid, beforeID, nextID, slot
	scheduling_matrix*/

	uint8_t broadcastMatrix[20] = {0, 0, 4, 4, 0, 4, 4, 4, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 18};
	//uint8_t broadcastMatrix[20] = {0, 0, 4, 4, 0, 4, 4, 4, 4, 8, 8, 12, 10, 12, 12, 12, 12, 16, 16, 16};

	//0 Slot
//1 Sender
//2 Receiver
//3 Channel
//4 Access Type:    0: dedicated,  1: shared, 2: steal, 3: ack
//5 Flow Type:      0: emergency, 1: regular
//6 Flow ID:        1, 2
//7 Flow root: root of the flow, i.e., sensor that launch the communcation
//8 Send status in sendDone: 0: no-ack, 1: acked
//9 Last Hop Status:
//10 Hop count in the flow
//11 Retransmission:

uint8_t schedule_flow1[50][11]={//Source Routing, 16 sensor topology, 2 prime trans, retrans twice, baseline. //***   12 == # of total hopcount
	{1, 4, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{2, 6, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{3, 8, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{4, 10, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{5, 12, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{6, 14, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{7, 16, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{8, 18, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	/* Reference Tx Schedule */
/*
	{9, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{10, 96, 98, 22, 0, 1, 1, 2, 0, 0, 3},
	{11, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{12, 98, 90, 22, 0, 1, 1, 2, 0, 0, 4},
	{13, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{14, 90, 92, 22, 0, 1, 1, 2, 0, 0, 5},
	{15, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{16, 92, 95, 22, 0, 1, 1, 2, 0, 0, 6},
	{17, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{18, 95, 96, 22, 0, 1, 1, 2, 0, 0, 7},
	{19, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8}
	*/
/*
	{9, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{10, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{11, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{12, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{13, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{14, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{15, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{16, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{17, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{18, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{19, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8},
	{20, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8}
	*/

	{9, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{10, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{11, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{12, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{13, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{14, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{15, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{16, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{17, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{18, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{19, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{20, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{21, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{22, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{23, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{24, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8},
	{25, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8},
	{26, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8}

/*
	{9, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{10, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{11, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{12, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
	{13, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{14, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{15, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{16, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
	{17, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{18, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{19, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{20, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
	{21, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{22, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{23, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{24, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
	{25, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{26, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{27, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{28, 15, 16, 22, 0, 1, 1, 2, 0, 0, 7},
	{29, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8},
	{30, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8},
	{31, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8},
	{32, 16, 19, 22, 0, 1, 1, 2, 0, 0, 8}
*/
	};


	uint8_t schedule_only_data[40][11]={//Source Routing, 16 sensor topology, 2 prime trans, retrans twice, baseline. //***   12 == # of total hopcount
		//flow 11, flow sensor
/*
		{1, 1, 5, 22, 0, 1, 1, 1, 0, 0, 1},         //P2C root node trans
		{2, 100, 100, 22, 0, 1, 1, 1, 0, 0, 1},     //P2C root node guard time
		{3, 5, 6, 22, 0, 1, 1, 1, 0, 0, 2},
		{4, 6, 4, 22, 0, 1, 1, 1, 0, 0, 3},
		{5, 100, 100, 22, 0, 1, 1, 1, 0, 0, 1},     //controller execution time
		{6, 3, 7, 22, 0, 1, 1, 1, 0, 0, 1},     //C2P root node trans
		{7, 100, 100, 22, 0, 1, 1, 1, 0, 0, 1},     //C2P root node guard time
		{8, 7, 8, 22, 0, 1, 1, 1, 0, 0, 2},
		{9, 8, 2, 22, 0, 1, 1, 1, 0, 0, 3},
		{10, 100, 100, 22, 0, 1, 1, 1, 0, 0, 1},    //end node guard time
		{11, 100, 100, 22, 0, 1, 1, 1, 0, 0, 1}
		*/
		/* Reference Tx Schedule */
		{1, 2, 6, 22, 0, 1, 1, 2, 0, 0, 2},
		{2, 2, 6, 22, 0, 1, 1, 2, 0, 0, 2},
		{3, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
		{4, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
		{5, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
		{6, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
		{7, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
		{8, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
		{9, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
		{10, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
		{11, 15, 17, 22, 0, 1, 1, 2, 0, 0, 7},
		{12, 15, 17, 22, 0, 1, 1, 2, 0, 0, 7},
		{13, 17, 19, 22, 0, 1, 1, 2, 0, 0, 8},
		{14, 17, 19, 22, 0, 1, 1, 2, 0, 0, 8},
		{15, 0, 0, 22, 0, 1, 1, 1, 0, 0, 9},
		{16, 0, 0, 22, 0, 1, 1, 1, 0, 0, 10},
		{17, 0, 0, 22, 0, 1, 1, 1, 0, 0, 11},
		{18, 0, 0, 22, 0, 1, 1, 1, 0, 0, 12},
		{19, 0, 0, 22, 0, 1, 1, 1, 0, 0, 13},
		{20, 0, 0, 22, 0, 1, 1, 1, 0, 0, 14},
		{21, 0, 0, 22, 0, 1, 1, 1, 0, 0, 15},
		{22, 0, 0, 22, 0, 1, 1, 1, 0, 0, 16},
		{23, 0, 0, 22, 0, 1, 1, 1, 0, 0, 17},
		{24, 0, 0, 22, 0, 1, 1, 1, 0, 0, 18},
		{25, 0, 0, 22, 0, 1, 1, 1, 0, 0, 19},
		{26, 0, 0, 22, 0, 1, 1, 1, 0, 0, 20},
		{27, 0, 0, 22, 0, 1, 1, 1, 0, 0, 21}

		};

		uint8_t schedule_flow2[50][11]={//Source Routing, 16 sensor topology, 2 prime trans, retrans twice, baseline. //***   12 == # of total hopcount
			{1, 4, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{2, 6, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{3, 8, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{4, 10, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{5, 12, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{6, 14, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{7, 16, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{8, 18, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			/* Reference Tx Schedule */
			{9, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
			{10, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
			{11, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
			{12, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
			{13, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
			{14, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
			{15, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
			{16, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
			{17, 10, 12, 22, 0, 1, 1, 2, 0, 0, 5},
			{18, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
			{19, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
			{20, 12, 15, 22, 0, 1, 1, 2, 0, 0, 6},
			{21, 15, 14, 22, 0, 1, 1, 2, 0, 0, 7},
			{22, 15, 14, 22, 0, 1, 1, 2, 0, 0, 7},
			{23, 15, 14, 22, 0, 1, 1, 2, 0, 0, 7}

		};

		uint8_t schedule_flow3[50][11]={//Source Routing, 16 sensor topology, 2 prime trans, retrans twice, baseline. //***   12 == # of total hopcount
			{1, 4, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{2, 6, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{3, 8, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{4, 10, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{5, 12, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{6, 14, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{7, 16, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			{8, 18, 100, 22, 0, 1, 1, 1, 0, 0, 1},
			/* Reference Tx Schedule */
			{9, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
			{10, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
			{11, 6, 8, 22, 0, 1, 1, 2, 0, 0, 3},
			{12, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
			{13, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
			{14, 8, 10, 22, 0, 1, 1, 2, 0, 0, 4},
			{15, 10, 11, 22, 0, 1, 1, 2, 0, 0, 5},
			{16, 10, 11, 22, 0, 1, 1, 2, 0, 0, 5},
			{17, 10, 11, 22, 0, 1, 1, 2, 0, 0, 5}

		};

	uint8_t count_recv_slot[85] = {0,};

	uint8_t (*schedule)[11] = schedule_flow1;

	uint8_t number_of_UD_len;

	uint8_t schedule_len=50;   //***    schedule_len == # of total hop
  uint32_t gigaframe_length = 300; //5Hz at most     //***  10 == NodeNumber(schedule_len + 1) + controlschedule_len
	uint8_t max_schedule_len = 50;
	uint8_t last_level;
  uint8_t cycle_len;
	uint8_t broad_len;
	uint16_t b_last_index;
	uint16_t s_last_index;
	uint16_t UD_len;
	uint8_t my_level;
	uint8_t bi, sd;
	uint8_t Data_trans_start_slot;
	uint8_t flow1_sched_len;
	uint8_t flow2_sched_len;
	uint8_t flow3_sched_len;
	uint8_t flow2_sched_last_idx;
	uint8_t flow3_sched_last_idx;
	uint8_t flow_root = 6;

	uint16_t count;
	bool ref_flag;
	uint32_t prev_seq = 0;
	bool ReTx = FALSE;

	//나중에 flow 수가 늘어나면 첫번째 index를 수정해야함

	/** @brief Boot Events
	 * @source TestAcks in cc2420 tests
	 */
	event void Boot.booted() {
		call RadioPowerControl.start();
	}

	/** @brief
	 * @source From PureTDMASchedulerP (UPMA)
	 * When the module is initialized
	 */
	command error_t Init.init() {

		/** @brief Initialization of necessary variables */
		currentSlot = 0xff;
		slotSize = 10 * 32; // 10ms standard
		inSync = 0; 		// Set to zero initially since nodes are not synchronized
		sequenceCount = 0;

		/** @brief Packets */
		notificationPacket = NULL;
		dataPacket = NULL;
		forwardPayload = NULL;

		/** @brief Boolean */
		init = FALSE;
		requestStop = FALSE;
		radioOff = TRUE;
		phyLock = FALSE;
		slotterInit = FALSE;

		superFrameLength = 100;
		currFrameSize = 100; //Same as above used by Frame Configuration interface
		ref_flag = 0;

		// ** Caution: Problem of variable initilization
		//Time_stemp_src_node = 1;

		return SUCCESS;
	}

	/** @brief
	 * @source From PureTDMASchedulerP (UPMA)
	 * Interface to startup the MAC module
	 *  */
 	command error_t MacPowerControl.start() {

 		error_t err;
		//int i,j;

		/** @brief Statistics counter and parent declaration **/
		/*for(j=0;j<MAX_FLOWS;j++)
			for(i=0;i<MAX_NODES;i++)
			{
				nbReceivedPkts[j][i] = 0;
			}*/

			number_of_UD_len = 1; //The number of UD schedule in a gigaframe (common variable)

			schedule = schedule_flow1;  // added by M.S.

	    //determine last_level, cycle_len, broad_len and my_level
	    for(i=0; i<max_schedule_len; i++)
	    {
	      //if receiver is not 255, then the time slot is the end of broadcast schedule
	      if(schedule[i][2] != 100)
	      {
	          last_level = schedule[i-1][0];
	          b_last_index = i-1;
	          Data_trans_start_slot = schedule[i][0];
	          break;

	      }

	      //if sender is same with my node id, at that time slot, slot number is my network level
	      if(schedule[i][1] == TOS_NODE_ID)
	      {
	        my_level = schedule[i][0];
	      }
	    }

	    for(i=b_last_index + 1; i<max_schedule_len; i++)
	    {
	      if(schedule[i][1] == 0)             //set s_last_index by checking sender in schedule[][]
	      {
	        s_last_index = i-1;
	        cycle_len = s_last_index - b_last_index;
	        broad_len = (last_level - 1) * cycle_len;
	        /*remove broad_len************/
	        broad_len = b_last_index + 1;


	        UD_len = s_last_index - b_last_index;   //set Up/Down link schedule length

	        //Test gigaframe_length
	        //UD_len = 10;

	        gigaframe_length = broad_len + (UD_len * number_of_UD_len);
	        bi = gigaframe_length;
	        sd = gigaframe_length;
	        break;
	      }
	    }

			// multi flow schedule
			flow1_sched_len = (broad_len + UD_len);
			for(i=0; i<max_schedule_len; i++){
				if(schedule_flow2[i][1] == 0){
					flow2_sched_last_idx = i-1;
					flow2_sched_len = i;
					break;
				}
			}
			for(i=0; i<max_schedule_len; i++){
				if(schedule_flow3[i][1] == 0){
					flow3_sched_last_idx = i-1;
					flow3_sched_len = i;
					break;
				}
			}
			//gigaframe_length = per_frame_num_flow * (flow1_sched_len + flow2_sched_len + flow3_sched_len); // for multi flow
			gigaframe_length = per_frame_num_flow * (flow1_sched_len); // for single flow
			bi = gigaframe_length;
			sd = gigaframe_length;
			//printf("flow1,2,3_sched_len:%d, %d, %d\r\n", flow1_sched_len, flow2_sched_len, flow3_sched_len);
			//printf("gigaframe_length:%d\r\n", gigaframe_length);
 		if (init == FALSE) {
	 		/** Configuring frame @aaks */
 		}
 		err = call RadioPowerControl.start();
		printf("<Mac>: We are up and running \n");
 		return err;
 	}

 	/** @brief
 	 * @source From PureTDMASchedulerP (UPMA)
 	 * Interface to stop the MAC module
 	 */
 	command error_t MacPowerControl.stop() {
 		requestStop = TRUE;
 		call GenericSlotterPowerControl.stop();
 		call RadioPowerControl.stop();
 		return SUCCESS;
 	}

 	/** @brief
	 * @source From PureTDMASchedulerP (UPMA)
	 * Powering up the radio
	 */
 	event void RadioPowerControl.startDone(error_t error) {

 		radioOff = FALSE;
 		//printf("<PHY>: Radio startup done");
 		/** @brief Rest is 0 thus for 3 we define 1 */

 		/** @brief Sink starts right away, rest of the nodes synchronize to the sink */
 		atomic{
	 		if (init == FALSE) {
				if (TOS_NODE_ID  == 0)
	 			{
					call GenericSlotterPowerControl.start();
		 			call SlotterControl.synchronize(1);
					call FrameConfiguration.setFrameLength(bi+1);
	 			}
				init = TRUE;
				call TimeSyncMode.setMode(1);
				//call ReferenceAlarm.start(0);	// 10 min
				if(TOS_NODE_ID == flow_root) call ReferenceAlarm.startPeriodic(10000); // 1000 = 1 s?
				signal MacPowerControl.startDone(error);
			}
 		}
	}

	/** @brief
 	 * @source PureTDMASchedulerP (UPMA)
 	 * Stopping radio services
	 */
 	event void RadioPowerControl.stopDone(error_t error)  {
		if (requestStop)  {
			requestStop = FALSE;
		 //	printf("<MAC>: Radio stopped \n");
			radioOff = TRUE;
		}
	}

 	/** @brief
 	 * @source From PureTDMASchedulerP (UPMA)
 	 * Goes through each slot performing the desired function
 	 * for the respective slot
 	 */
  event void Slotter.slot(uint8_t slot) {
 		dmamac_data_t *forwardDataPacket;
		RefData_t *queuePacket;
		RefData_t *rootPayload;
		uint8_t temp_Q_size;
		uint8_t i;
		uint8_t mapping = 0;
		uint32_t remaining;

		Timeinsec_t *dataBuffer_time;

		atomic currentSlot = slot;

		/* Reference Change */
		if(slot == 0 && TOS_NODE_ID != 0) {

			printf(" ------- new gigaframe ------ \r\n");
			if(TOS_NODE_ID == 19){
				printf("ID : %d  send : %d, receive : %d TxseqNum : %d\r\n",TOS_NODE_ID, send_cunt, rcv_cunt, TxseqNum);
			}else if (TOS_NODE_ID == flow_root){
				printf("ID : %d  send : %d, receive : %d flowSeq : %d\r\n",TOS_NODE_ID, send_cunt, rcv_cunt, flowSeq);
			}
			/*
			if(TOS_NODE_ID != flow_root){
				printf("ref1 - idx:%d\r\n", dbg_currRef1_idx);
				printf("ref1 - value:%d\r\n", dbg_currRef1);
				printf("ref5 - idx:%d\r\n", dbg_currRef5_idx);
				printf("ref5 - value:%d\r\n", dbg_currRef5);
				printf("rcv prev seq:%d\r\n", prev_seq);
			}*/

			if(!call forwardQueue.empty())
			{
				temp_Q_size = call forwardQueue.size();
				for(i=0;i<temp_Q_size;i++)
				{
					call forwardQueue.dequeue();
				}
			}

			atomic phyLock = FALSE;
			atomic ReTx = FALSE;
			atomic flowSeq++;
			/*
			printf("ID : %d  send : %d, receive : %d seqNum : %d\r\n",TOS_NODE_ID, send_cunt, rcv_cunt, TxseqNum);
			if(TOS_NODE_ID == 6){
				printf("lastest payload -- [ref1=%d]\r\n", currRef1);
				printf("lastest payload -- [ref5=%d]\r\n", currRef5);
			}else{
				printf("lastest payload -- [ref1,idx=%d,%d]\r\n", dbg_currRef1, dbg_currRef1_idx);
				printf("lastest payload -- [ref5,idx=%d,%d]\r\n", dbg_currRef5, dbg_currRef5_idx);
			}*/

			return;
		}
		/* for single flow
		if(slot != 0) mapping = slot - 1;
		if(slot >= broad_len + UD_len + 1) {
			mapping = (slot - broad_len - UD_len - 1) % UD_len;
		}
		if(mapping == 0 && slot <= 1) schedule = schedule_flow1;
		else if(mapping == 0 && slot > 1) schedule = schedule_only_data;
		*/

		//mapping = get_singleFlow_mapping(slot);		// single flow
		mapping = get_multiFlow_mapping(slot);	// multi flows




		//printf("slot = %d,  mapping = %d,  sender = %d, per_frame_flow_idx:%d, flow seq: %d\r\n",slot,mapping,schedule[mapping][1], per_frame_flow_idx, flowSeq);

		if(TOS_NODE_ID == 0 && slot == 0)
		{
			notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
		//	call AMPacket.setSource(&selfNotificationPkt, TOS_NODE_ID);

			notificationPacket->nextSlot = currentSlot+1;
			notificationPacket->nextAlarm = call SlotterControl.getNow();
			//  schedule_idx will be increase in phyNotificationSend.sendDone function
			if(!phyLock)
			{
				if(call phyNotificationSend.send
				(AM_BROADCAST_ADDR, &selfNotificationPkt, sizeof(notification_t)) == SUCCESS)  //broadcast_route[TOS_NODE_ID % 10][notificationPacket->curId]
				{
					atomic phyLock = TRUE;
				//	printf("<Mac,notification>: Pkt -> Radio \n");
				}
				else
					printf("<Mac>:  notifPkt <FAIL>  \r\n");
			}
			else
					printf("<Mac-nofiPkt-Fail>: Radio locked  \r\n");
		}



		/** @brief Notification slot for sink */
		if(schedule[mapping][1] == TOS_NODE_ID && slot != 0) {	//sender check
			if (schedule[mapping][2] == 100 )
			 {

				 myGlobalTime = call GlobalTime.getLocalTime();
				 inSync = (uint32_t) call GlobalTime.local2Global(&myGlobalTime);
				 remaining = call SlotterControl.getRemaining();

				 notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
				 notificationPacket->nextSlot = slot + 1;
				 notificationPacket->nextAlarm = myGlobalTime + (remaining / 32);

				 if(!phyLock)
				 {
					 if(call phyNotificationSend.send
					 (AM_BROADCAST_ADDR, &selfNotificationPkt, sizeof(notification_t)) == SUCCESS)  //broadcast_route[TOS_NODE_ID % 10][notificationPacket->curId]
					 {
						 atomic phyLock = TRUE;
					 }
					 else
						 printf("<Mac>:  notifPkt <FAIL>  \r\n");
				 }
				 else
						 printf("<Mac-nofiPkt-Fail>: Radio locked  \r\n");

			}
			else
			{

				if(!call forwardQueue.empty())	// relay transmission
				{
						first_Tx_slot = schedule[mapping][0];	// for Retransmission
						queuePacket = call forwardQueue.head();


					  forwardPayload = (RefData_t*)call Packet.getPayload(&selfDataPkt, sizeof(RefData_t));
						loadPayload(forwardPayload, queuePacket);

						if (phyLock == FALSE && ReTx == FALSE) {
							call ACK.requestAck(&selfDataPkt);
							//printf("slot:%d, ID:%d, dest:%d\r\n", schedule[mapping][0], TOS_NODE_ID, schedule[mapping][2]);
							if(call phyDataSend.send(schedule[mapping][2], &selfDataPkt, sizeof(RefData_t)) == SUCCESS)
								{
									//printf("[Transmission] at cur_slot:%d\r\n", currentSlot);
									atomic phyLock = TRUE;
									atomic send_cunt++;
								}
								else
									printf("MAC: Pkt fail  \r\n");
						}
				}
				//Test for link quality
				else{
					//if(flowSeq<1000){
					if(TOS_NODE_ID == flow_root && flowSeq < 1000){	//flow root Tx
						if (phyLock == FALSE && ReTx == FALSE) {
							rootPayload = (RefData_t*)call Packet.getPayload(&selfDataPkt, sizeof(RefData_t));
							generatePayload(rootPayload);

							call ACK.requestAck(&selfDataPkt);
							if(call phyDataSend.send(schedule[mapping][2], &selfDataPkt, sizeof(RefData_t)) == SUCCESS)
								{
									atomic phyLock = TRUE;
									atomic send_cunt++;
								}
								else
									printf("MAC: Pkt fail  \r\n");
						}
					}
				}
			}

		}
	}

	/** @brief Interfaces provided by MAC */
	/** @brief Send interface provided by MAC */
	command error_t MacSend.send(message_t * msg, uint8_t len) {

		/** @brief APP packet is dummy, will be created here and sent by MAC
		 * if proper App is written its payload can be fit into the MAC packet */
		atomic {
		//		printf("MAC: Prepare APP packet \n");
		    	return SUCCESS;
				}
	}

	/** @brief Cancel possibilities provided by the MAC interface */
 	command error_t MacSend.cancel(message_t *msg) {

  		/** @brief if using the toSend variable for Application packet
  		atomic {
 			if (toSend == NULL) return SUCCESS;
 			atomic toSend = NULL;
 		}
 		 * */
 		/** @brief No current plans to use this */
 		return call phyDataSend.cancel(msg);
 	}

 	/** @brief
	 * Auto generated missing declarations for interfaces used
	 */
	command uint8_t MacSend.maxPayloadLength(){
		// Unused for now (not using APP packets)
		return 0;
	}

	command void * MacSend.getPayload(message_t *msg, uint8_t len){
		// Unused for now (not using APP packets)
		return msg;
	}
	/**}*/


	/** @brief parts handling radio commands and events part */
	/**{*/
	/** @brief
	 * Event handler for sendDone from the radio module
	 * When packet is sent successfully, we free up the local buffer
	 * <toSend> variable is a local packet buffer
	 */

	/** @brief Event handler for Radio receiving packets */

	event void phyNotificationSend.sendDone(message_t *msg, error_t error){
		//printf("SinkMAC: Notification sent : %u \n",error);
		/** @brief Clearing send lock once successfully sent on channel */
		atomic phyLock = FALSE;
		//printf(" notify send at %d\r\n",currentSlot);
		signal MacSend.sendDone(msg, error);

	}

	event message_t * phyNotificationReceive.receive(message_t *msg, void *payload, uint8_t len){
		/* prepare for forwarding for notification_t*/
		notification_t* notifyPacketTemp =(notification_t *)payload;
		uint32_t gap;
		uint32_t alarmTime;

		uint8_t nextSlot;
		uint32_t nextAlarm;
		uint32_t myNextAlarm;

		uint32_t remaining;

		src = (am_addr_t )call AMPacket.source(msg);
		//printf("src = %d || slot:%d\r\n",src, currentSlot);
		if(src == broadcastMatrix[TOS_NODE_ID]) {

			nextSlot = notifyPacketTemp->nextSlot;
			nextAlarm = notifyPacketTemp->nextAlarm;

		//	printf(" nextSlot = %d   nextAlarm = %d\r\n",nextSlot, nextAlarm);

		// printf("Global time Rx : %d\r\n",myGlobalTime);
		//	printf("  myGlobalTime = %d  ",myGlobalTime);
		//	printf("myNextAlarm = %d\r\n", myNextAlarm);

	/*		printf("Notifi receive at %d   ",currentSlot);
			printf("node = %d   ",TOS_NODE_ID);
			printf("to slot = %d\r\n",notifyPacketTemp->currentSlot);*/
	/*		toSlot = notifyPacketTemp->currentSlot + 1;

			if(slotterInit == FALSE){
				call FrameConfiguration.setFrameLength(bi+1);
				call SlotterControl.synchronize(toSlot);
				slotterInit = TRUE;
			}
			else {*/
		/*		if(gap <= slotSize && gap >= 0)	{
					call SlotterControl.stop();
					call SyncAlarm.start(gap);
				}
				else if(gap < 0 && gap >= slotSize * (-1)) {
					call SlotterControl.stop();
					call SyncAlarm.start(notifyPacketTemp->nextAlarm + slotSize - tempGlobal);
				}
				else{
					call SlotterControl.stop();*/
				/*	gap = call SlotterControl.getNow(); */
				/*	printf(" now = %d   ",gap);
					printf("   src now = %d   ",notifyPacketTemp->nextAlarm);
					printf(" gap = %d \r\n", gap - notifyPacketTemp->nextAlarm);*/
					//call SlotterControl.stop();
			/*		call SlotterControl.synchronize(toSlot);*/
					//call SyncAlarm.start(call SlotterControl.getRemaining());
			//	}
			//}
			//call GlobalTime.local2Global(&tempGlobal);

			/** @brief part used to synchronize rest of the nodes
			 * Makes sure nodes are time synced through FTSP with minimum 8 table entries
			 * */
			/**{*/
		//printf(" notify receive at %d\r\n",currentSlot);

			myGlobalTime = call GlobalTime.getLocalTime();
			inSync = (uint32_t) call GlobalTime.local2Global(&myGlobalTime);
			myNextAlarm = myGlobalTime + (call SlotterControl.getRemaining() / 32);

			toSlot = notifyPacketTemp->nextSlot;
			gap = (notifyPacketTemp->nextAlarm - myGlobalTime) * 32;
		/*	remaining = call SlotterControl.getRemaining();
			printf("slot = %d  gap = %d  ",currentSlot,myNextAlarm - notifyPacketTemp->nextAlarm);
			printf("remaining = %d\r\n",remaining);*/

			if(slotterInit == FALSE)
			{
				call TimeSyncMode.setMode(0);
				call FrameConfiguration.setFrameLength(bi+1);
				/** @brief Synchronizing to the send slot of the sink that is 1 currently (MOVE planned) */
				if(TOS_NODE_ID != 4)
				{
					call SlotterControl.stop();
					call SyncAlarm.start(gap);
				}
				else
					call SlotterControl.synchronize(toSlot);
				slotterInit = TRUE;
			}
			if(gap >= 320) {
				gap = gap % 320;
				call SlotterControl.stop();
				call SyncAlarm.start(gap);
			}
			else {
				call SlotterControl.stop();
				call SyncAlarm.start(gap);
			}
		}
		return msg;
	}


	event void phyDataSend.sendDone(message_t *msg, error_t error){
		uint8_t temp_Q_size;
		uint8_t i;

		if(call ACK.wasAcked(msg))
		{
			atomic ReTx = TRUE;
			if(!call forwardQueue.empty())
			{
				temp_Q_size = call forwardQueue.size();
				for(i=0;i<temp_Q_size;i++)
					call forwardQueue.dequeue();
			}
		}


		/** @brief Clearing send lock once successfully sent on channel */
		atomic phyLock = FALSE;
		signal MacSend.sendDone(msg, error);
	}



	event message_t * phyDataReceive.receive(message_t *msg, void *payload, uint8_t len){
		/*Reference Receive*/
		RefData_t* dataPacketTemp = (RefData_t *)payload;
		RefData_t* dataBuffer =  (RefData_t *)NULL;
		RefData_t* queueHead;	// Used to verify the contents saved in the queue
		/*Data Receive*/
		//dmamac_data_t* dataPacketTemp = (dmamac_data_t *)payload;
		//dmamac_data_t* dataBuffer =  (dmamac_data_t *)NULL;
		//dmamac_data_t* queueHead;	// Used to verify the contents saved in the queue
		uint8_t src;

		//if(prev_seq != dataPacketTemp->seq && my_network_id == dataPacketTemp->network_id)	// for multi flow
		//printf("network id:%d\r\n", dataPacketTemp->network_id);
		if(prev_seq != dataPacketTemp->seq)
		{
			src = (am_addr_t )call AMPacket.source(msg);
			count_recv_slot[currentSlot]++;

			queueSize = call forwardQueue.size();
			rcv_cunt++;
			call Leds.led0Toggle();
			dataBuffer = (RefData_t*)call Packet.getPayload(&forwardPktBuffer[queueSize], sizeof(RefData_t));
			dataBuffer->refidx1 = dataPacketTemp->refidx1;
			dataBuffer->ref1 = dataPacketTemp->ref1;
			dataBuffer->refidx2 = dataPacketTemp->refidx2;
			dataBuffer->ref2 = dataPacketTemp->ref2;
			dataBuffer->refidx3 = dataPacketTemp->refidx3;
			dataBuffer->ref3 = dataPacketTemp->ref3;
			dataBuffer->refidx4 = dataPacketTemp->refidx4;
			dataBuffer->ref4 = dataPacketTemp->ref4;
			dataBuffer->refidx5 = dataPacketTemp->refidx5;
			dataBuffer->ref5 = dataPacketTemp->ref5;
			dataBuffer->seq = dataPacketTemp->seq;
			dataBuffer->network_id = dataPacketTemp->network_id;
			prev_seq = dataPacketTemp->seq;

			// for debug
			/*
			if(TOS_NODE_ID != flow_root && TOS_NODE_ID == 19){
				dbg_currRef1 = dataPacketTemp->ref1;
				dbg_currRef1_idx = dataPacketTemp->refidx1;
				dbg_currRef2 = dataPacketTemp->ref2;
				dbg_currRef2_idx = dataPacketTemp->refidx2;
				dbg_currRef3 = dataPacketTemp->ref3;
				dbg_currRef3_idx = dataPacketTemp->refidx3;
				dbg_currRef4 = dataPacketTemp->ref4;
				dbg_currRef4_idx = dataPacketTemp->refidx4;
				dbg_currRef5 = dataPacketTemp->ref5;
				dbg_currRef5_idx = dataPacketTemp->refidx5;

				printf("<Received from %d>\r\n", src);
				printf("ref1 - idx:%d\r\n", dataBuffer->refidx1);
				printf("ref1 - value:%d\r\n", dataBuffer->ref1);
				printf("ref5 - idx:%d\r\n", dataBuffer->refidx5);
				printf("ref5 - value:%d\r\n", dataBuffer->ref5);

			}*/


			TxseqNum = dataPacketTemp->seq;
			//printf("[Receive] Reference data: %d, %d, %d\r\n", dataPacketTemp->ref1, dataPacketTemp->ref2, dataPacketTemp->ref3);
			call forwardQueue.enqueue(dataBuffer);
			if(!call forwardQueue.empty())
			{
					queueHead = call forwardQueue.head();
			}

		}

		return msg;

	}

	async event void SyncAlarm.fired() {
		//inSync = (uint8_t) call GlobalTime.local2Global(&tempGlobal);
		call SlotterControl.synchronize(toSlot);
	}

	event void ReferenceAlarm.fired() {
		printf("referenceAlarm fired\r\n");
		// after 10 min, reference change
		if(ref_flag == 1){
			currRef1 = 2;
			currRef2 = 2;
			currRef3 = 2;
			currRef4 = 2;
			currRef5 = 2;

			ref_flag = 0;
		}else{
			currRef1 = 6;
			currRef2 = 6;
			currRef3 = 6;
			currRef4 = 6;
			currRef5 = 6;

			ref_flag = 1;
		}
		//call ReferenceAlarm.start(10*32 * 100 * 600);	// 10 min

	}

	/**}*/

	/** @brief
	 * Frame configuration part, linking Frame from our MAC to
	 * Frame Configuration interface provided by Generic Slotter
	 */
	/**{*/

  command void Frame.setSlotLength(uint32_t slotTimeBms) {
		atomic slotSize = slotTimeBms;
		call FrameConfiguration.setSlotLength(slotSize);
		return;
 	}
 	command void Frame.setFrameLength(uint8_t numSlots) {
 		atomic currFrameSize = numSlots;
 		call FrameConfiguration.setFrameLength(currFrameSize);
 		return;
 	}
 	command uint32_t Frame.getSlotLength() {
		atomic slotSize = call FrameConfiguration.getSlotLength();
 		return slotSize;
 	}
 	command uint8_t Frame.getFrameLength() {
 		uint8_t frameLength;
 		atomic frameLength = call FrameConfiguration.getFrameLength();
 		return frameLength;
 	}
	/**}*/

	void sendPkt(RefData_t * packet, uint8_t destination){

		if (phyLock == FALSE) {
			call ACK.requestAck(packet);
			if(call phyDataSend.send(destination, packet, sizeof(RefData_t)) == SUCCESS)
				{
					atomic phyLock = TRUE;
					send_cunt++;
					flowSeq++;
				}
				else{
					printf("MAC: Pkt fail  \r\n");
				}
		}
		else{
			printf("MAC: Radio Locked \r\n");
		}
	}

	void generatePayload(RefData_t * packet){
		// set reference
		packet->refidx1 = (5*per_frame_flow_idx) + 0;
		packet->ref1 = currRef1;
		packet->refidx2 = (5*per_frame_flow_idx) + 1;
		packet->ref2 = currRef2;
		packet->refidx3 = (5*per_frame_flow_idx) + 2;
		packet->ref3 = currRef3;
		packet->refidx4 = (5*per_frame_flow_idx) + 3;
		packet->ref4 = currRef4;
		packet->refidx5 = (5*per_frame_flow_idx) + 4;
		packet->ref5 = currRef5;
		packet->seq = flowSeq;
		packet->network_id = curr_network_id;
	}

	void loadPayload(RefData_t * forwardPacket, RefData_t * queuePacket){
		forwardPacket->refidx1 = queuePacket->refidx1;
		forwardPacket->ref1 = queuePacket->ref1;
		forwardPacket->refidx2 = queuePacket->refidx2;
		forwardPacket->ref2 = queuePacket->ref2;
		forwardPacket->refidx3 = queuePacket->refidx3;
		forwardPacket->ref3 = queuePacket->ref3;
		forwardPacket->refidx4 = queuePacket->refidx4;
		forwardPacket->ref4 = queuePacket->ref4;
		forwardPacket->refidx5 = queuePacket->refidx5;
		forwardPacket->ref5 = queuePacket->ref5;
		forwardPacket->seq = queuePacket->seq;
		forwardPacket->network_id = queuePacket->network_id;
	}

	uint8_t get_multiFlow_mapping(uint8_t slot){
		uint8_t mapping = 0;
		uint8_t temp_Q_size;

		if(slot != 0 && slot < (flow1_sched_len + 1)){
			mapping = slot - 1;
		}else if(slot >= (flow1_sched_len + 1) && slot < (per_frame_num_flow * flow1_sched_len + 1)){
			mapping = (slot - 1) % flow1_sched_len;
			if(mapping == 0){
				// flow index change
				per_frame_flow_idx++;
				per_frame_flow_idx = (per_frame_flow_idx % per_frame_num_flow);


				if(!call forwardQueue.empty())
				{
					temp_Q_size = call forwardQueue.size();
					for(i=0;i<temp_Q_size;i++)
					{
						call forwardQueue.dequeue();
					}
				}
				atomic phyLock = FALSE;
				atomic ReTx = FALSE;
				atomic flowSeq++;
			}
		}else if(slot >= (per_frame_num_flow * flow1_sched_len + 1) && slot < (per_frame_num_flow * (flow1_sched_len + flow2_sched_len) + 1)){
			mapping = (slot - (per_frame_num_flow * flow1_sched_len) - 1) % flow2_sched_len;
			if(mapping == 0){
				schedule = schedule_flow2;
				curr_network_id = 2;
				// flow index change
				per_frame_flow_idx++;
				per_frame_flow_idx = (per_frame_flow_idx % per_frame_num_flow);

				if(!call forwardQueue.empty())
				{
					temp_Q_size = call forwardQueue.size();
					for(i=0;i<temp_Q_size;i++)
					{
						call forwardQueue.dequeue();
					}
				}
				atomic phyLock = FALSE;
				atomic ReTx = FALSE;
				atomic flowSeq++;
			}
		}else if(slot >= (per_frame_num_flow * (flow1_sched_len + flow2_sched_len) + 1)){
			mapping = (slot - (per_frame_num_flow * (flow1_sched_len + flow2_sched_len)) - 1) % flow3_sched_len;
			if(mapping == 0){
				schedule = schedule_flow3;
				curr_network_id = 3;
				// flow index change
				per_frame_flow_idx++;
				per_frame_flow_idx = (per_frame_flow_idx % per_frame_num_flow);

				if(!call forwardQueue.empty())
				{
					temp_Q_size = call forwardQueue.size();
					for(i=0;i<temp_Q_size;i++)
					{
						call forwardQueue.dequeue();
					}
				}
				atomic phyLock = FALSE;
				atomic ReTx = FALSE;
				atomic flowSeq++;
			}
		}
		if(mapping == 0 && slot <= 1){
			schedule = schedule_flow1;
			curr_network_id = 1;
			// flow index change
			per_frame_flow_idx++;
			per_frame_flow_idx = (per_frame_flow_idx % per_frame_num_flow);
		}
		return mapping;
	}


	uint8_t get_singleFlow_mapping(uint8_t slot){
		uint8_t mapping = 0;
		uint8_t temp_Q_size;

		if(slot != 0 && slot < (flow1_sched_len + 1)){
			mapping = slot - 1;
		}else if(slot >= (flow1_sched_len + 1) && slot < (per_frame_num_flow * flow1_sched_len + 1)){
			mapping = (slot - 1) % flow1_sched_len;
			if(mapping == 0){
				// flow index change
				per_frame_flow_idx++;
				per_frame_flow_idx = (per_frame_flow_idx % per_frame_num_flow);


				if(!call forwardQueue.empty())
				{
					temp_Q_size = call forwardQueue.size();
					for(i=0;i<temp_Q_size;i++)
					{
						call forwardQueue.dequeue();
					}
				}
				atomic phyLock = FALSE;
				atomic ReTx = FALSE;
				atomic flowSeq++;
			}
		}

		if(mapping == 0 && slot <= 1){
			schedule = schedule_flow1;
			curr_network_id = 1;
			// flow index change
			per_frame_flow_idx++;
			per_frame_flow_idx = (per_frame_flow_idx % per_frame_num_flow);
		}

		return mapping;
	}

}
