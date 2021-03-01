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
		interface Queue<notification_t*> as forwardNotifyQueue;
		interface PacketAcknowledgements as ACK;

		//Added by Sihoon
		interface Queue<adaptivehart_msg_t *> as HIforwardQ;
		interface Queue<adaptivehart_msg_t *> as LOforwardQ;
		interface Queue<adaptivehart_msg_t *> as RootQ;
		interface AdaptiveHART_Util as AH_Util;
		interface ReceiveIndicator;
		interface Alarm<T32khz, uint16_t> as TxOffset_relay;
		interface Alarm<T32khz, uint16_t> as TxOffset_root;
		interface LocalTime<T32khz>as dbg_timer;
		interface Timer<TMilli> as dbg_timer2;
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
	uses interface Leds;

}

implementation {
	enum {
		MAC_Q = 1,
		FLOW_SEQ = 20,
	};

	/** @brief Radio operation variables*/
	bool init;
	bool requestStop;
	bool radioOff;
	bool phyLock = FALSE; 			// Radio lock to prevent double send

	/** @brief Time sync info */
	uint32_t myLocalTime,myGlobalTime,myOffset,mySkew;
	uint8_t inSync; 		// Is the node synchronized on time with sink

	/** @brief Message to transmit From TestAcks */
	message_t forwardNotifyPktBuffer[MAX_CHILDREN];
	message_t selfNotificationPkt;

	/** @brief Payload parts for packets */
	notification_t* notificationPacket;

	/** @brief Slot mechanism and superframe */
	bool slotterInit ;		// Slotter initialized after time-synchronized
	uint32_t slotSize;
	uint16_t currentSlot;
	uint8_t toSlot;
	uint32_t tempGlobal;
	uint16_t superFrameLength,currFrameSize;
	uint16_t bi, sd;

	/* AdaptiveHART variables */
	uint8_t i;
	uint8_t flows_start_slot = 0;
	bool is_root = FALSE;
	bool is_dest[NETWORK_FLOW];
	uint8_t my_release_offset = 0;
	uint8_t my_flow_id = 0;

	uint8_t broadcastMatrix[20] = {0, 4, 4, 4, 0, 4, 4, 4, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 18};

	/* For message */
	message_t AH_pkt;
	adaptivehart_msg_t *AH_payload;
	bool is_AH_pkt_null = TRUE;

	/* For Queue */
	message_t HIforwardPktBuffer[MAC_Q];
	message_t LOforwardPktBuffer[MAC_Q];
	message_t RootPktBuffer[MAC_Q];
	uint8_t HIqueueSize;
	uint8_t LOqueueSize;
	uint8_t RootqueueSize;

	/* For schedule and relay */
	int8_t tx_opper[NETWORK_FLOW];
	uint8_t txing_flowid = 0;
	uint16_t rcv_slot[NETWORK_FLOW];
	uint16_t flow_deadline[NETWORK_FLOW];
	uint16_t root_seq = 0;
	uint16_t prev_seq[NETWORK_FLOW];
	bool rcv_indicator = FALSE;
	uint8_t my_txoffset[NETWORK_FLOW];

	/* Routing */
	uint8_t my_dest[NETWORK_FLOW];

	/* For debug */
	uint32_t timer1 = 0;
	uint32_t timer2 = 0;
	uint32_t diff = 0;
	uint16_t last_rcv_slot = 0;
	uint8_t last_src = 0;
	uint16_t rcv_slots[NETWORK_FLOW][FLOW_SEQ];
	uint16_t rcv_srcs[NETWORK_FLOW][FLOW_SEQ];
	uint16_t tx_slots[FLOW_SEQ];
	uint8_t rx_flow_seq[NETWORK_FLOW];
	uint8_t tx_flow_seq = 0;

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

uint8_t sync_schedule[50][11]={//Source Routing, 16 sensor topology, 2 prime trans, retrans twice, baseline. //***   12 == # of total hopcount
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{1, 4, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{2, 6, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{3, 8, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{4, 10, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{5, 12, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{6, 14, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{7, 16, 100, 22, 0, 1, 1, 1, 0, 0, 1},
	{8, 18, 100, 22, 0, 1, 1, 1, 0, 0, 1}
	};

	/* schedule variables */
	uint8_t (*schedule)[11] = sync_schedule;
  uint32_t gigaframe_length = 50; //5Hz at most     //***  10 == NodeNumber(schedule_len + 1) + controlschedule_len
	uint8_t max_schedule_len = 20;

	/* release schedule */
	// only flow root has their flow release time
	uint8_t my_release_schedule[2];

	/* Task character */
	// index: flow id
	// 0: Task period
	// 1: Task deadline
	// 2: Task maximum Tx oppertunity
	uint8_t Task_character[NETWORK_FLOW][3] = {
		{0, 0, 0},
		{0, 0, 2},
		{0, 0, 2}
	};
	uint8_t TASK_PERIOD = 0;
	uint8_t TASK_DEAD = 1;
	uint8_t TASK_MAXTX = 2;
	/* Adaptivity */
	uint16_t random_slots[10] = {19, 21, 22, 25, 28, 35, 39, 43, 46, 52};
	uint8_t random_slot_idx = 0;
	uint8_t random_slot_seq = 0;
	uint16_t random_slot = 0;
	uint8_t original_period[NETWORK_FLOW] = {0, 250, 250};
	uint8_t original_deadline[NETWORK_FLOW] = {0, 250, 250};
	uint8_t changed_period[NETWORK_FLOW] = {0, 20, 25};
	uint8_t changed_deadline[NETWORK_FLOW] = {0, 20, 25};
	/* Routing path */

	/* Functions */
	void send_AH_Pkt(message_t *packet, am_addr_t dest, uint8_t flowid);
	void set_payload(adaptivehart_msg_t *payload, uint8_t flowid, uint8_t priority, uint8_t maxtx, uint16_t deadline, uint16_t seq);
	void Q_reset(uint8_t flowid);
	void reset_var(uint8_t flowid);
	void offset_algorithm(uint8_t tmp_txing_flowid);

	event void Boot.booted() {
		call RadioPowerControl.start();
	}

	command error_t Init.init() {

		/** @brief Initialization of necessary variables */
		currentSlot = 0xffff;
		slotSize = 10 * 32; // 10ms standard
		inSync = 0; 		// Set to zero initially since nodes are not synchronized

		/** @brief Boolean */
		init = FALSE;
		requestStop = FALSE;
		radioOff = TRUE;
		atomic phyLock = FALSE;
		slotterInit = FALSE;

		superFrameLength = 100;
		currFrameSize = 100; //Same as above used by Frame Configuration interface

		/* Notification pkt */
		notificationPacket = NULL;
		// for notifying a last slot of synchronization



		return SUCCESS;
	}

 	command error_t MacPowerControl.start() {
		error_t err;
		uint8_t j;
		schedule = sync_schedule;  // added by M.S.

		/* set last slot of sync schedule */
		for(i=1; i<max_schedule_len; i++){
			if(sync_schedule[i][2] != 100){
				flows_start_slot = sync_schedule[i-1][0] + 1;
				break;
			}
		}
		// gigaframe_length =
		bi = gigaframe_length;
		sd = gigaframe_length;

		/* initialize essential variables */
		for(i=0; i<NETWORK_FLOW; i++){
			atomic tx_opper[i] = 0;
			atomic rcv_slot[i] = 0;
			Task_character[i][TASK_PERIOD] = original_period[i];
			Task_character[i][TASK_DEAD] = original_deadline[i];
		}

		/* set flow root, release offset and its period */
		is_root = call AH_Util.is_root(TOS_NODE_ID);
		if(is_root == TRUE){
			my_release_offset = call AH_Util.get_release_offset(TOS_NODE_ID);
			atomic my_flow_id = call AH_Util.get_flow_id(TOS_NODE_ID);
		}

		/* set destination and txoffset*/
		for(i=1; i<NETWORK_FLOW; i++){
			atomic my_dest[i] = call AH_Util.set_destination(i, TOS_NODE_ID);
			atomic my_txoffset[i] = call AH_Util.set_TxOffset(i, TOS_NODE_ID);
			is_dest[i] = call AH_Util.is_dest(i, TOS_NODE_ID);

			/* for debug, every frame a receive information is printed at a 0 slot */
			for(j=0; j++; j<FLOW_SEQ){
				rcv_slots[i][j] = 0;
				rcv_srcs[i][j] = 0;
			}
			rx_flow_seq[i] = 0;
		}

		/* adaptability parameter init */
		random_slot = random_slots[random_slot_idx];


		printf("dbg: HI_txoffset:%d, and LO_txoffset:%d\r\n", my_txoffset[1], my_txoffset[2]);

 		err = call RadioPowerControl.start();
		printf("<Mac>: We are up and running \r\n");
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

 	event void RadioPowerControl.startDone(error_t error) {
 		radioOff = FALSE;

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
				signal MacPowerControl.startDone(error);
			}
 		}
	}

 	event void RadioPowerControl.stopDone(error_t error)  {
		if (requestStop)  {
			requestStop = FALSE;
		 //	printf("<MAC>: Radio stopped \n");
			radioOff = TRUE;
		}
	}

  event void Slotter.slot(uint16_t slot) {
		uint16_t mapping;
		uint32_t remaining;
		int16_t flow_start_slot = -1;
		uint8_t tmp_flowid = 0;
		adaptivehart_msg_t *Q_payload = (adaptivehart_msg_t *)NULL;
		bool tmp_phyLock;
		uint8_t tmp_tx_opper;
		bool tmp_is_dest;

		atomic currentSlot = slot;
		mapping = slot;

		/* Reset Q, variables */
		if(slot == 0 && TOS_NODE_ID != 0) {
			/* for debug, every frame a receive information is printed at a 0 slot */

			printf("HI_TASK: Rx: [ %d, %d, %d, %d, %d ]\r\n", rcv_slots[HI_TASK][0], rcv_slots[HI_TASK][1], rcv_slots[HI_TASK][2], rcv_slots[HI_TASK][3], rcv_slots[HI_TASK][4]);
			printf("LO_TASK: Rx: [ %d, %d, %d, %d, %d ]\r\n", rcv_slots[LO_TASK][0], rcv_slots[LO_TASK][1], rcv_slots[LO_TASK][2], rcv_slots[LO_TASK][3], rcv_slots[LO_TASK][4]);

			printf("random_slot: %d, Tx: [ %d, %d, %d, %d, %d, %d, %d, %d, %d, %d ]\r\n", random_slot, tx_slots[0], tx_slots[1], tx_slots[2], tx_slots[3], tx_slots[4], tx_slots[5], tx_slots[6], tx_slots[7], tx_slots[8], tx_slots[9]);
			tx_slots[0]=0; tx_slots[1]=0; tx_slots[2]=0; tx_slots[3]=0; tx_slots[4]=0; tx_slots[5]=0; tx_slots[6]=0; tx_slots[7]=0; tx_slots[8]=0; tx_slots[9]=0;
			tx_flow_seq=0;

			rcv_slots[HI_TASK][0]=0; rcv_slots[HI_TASK][1]=0; rcv_slots[HI_TASK][2]=0; rcv_slots[HI_TASK][3]=0; rcv_slots[HI_TASK][4]=0;
			rcv_slots[LO_TASK][0]=0; rcv_slots[LO_TASK][1]=0; rcv_slots[LO_TASK][2]=0; rcv_slots[LO_TASK][3]=0; rcv_slots[LO_TASK][4]=0;

			rx_flow_seq[HI_TASK] = 0;
			rx_flow_seq[LO_TASK] = 0;

			/* else{
				printf("random_slot: %d, Tx: [ %d, %d, %d, %d, %d, %d, %d, %d, %d, %d ]\r\n", random_slot, tx_slots[0], tx_slots[1], tx_slots[2], tx_slots[3], tx_slots[4], tx_slots[5], tx_slots[6], tx_slots[7], tx_slots[8], tx_slots[9]);
				tx_slots[0]=0; tx_slots[1]=0; tx_slots[2]=0; tx_slots[3]=0; tx_slots[4]=0; tx_slots[5]=0; tx_slots[6]=0; tx_slots[7]=0; tx_slots[8]=0; tx_slots[9]=0;
				tx_flow_seq=0;
			} */


			printf(" ------- new Frame ------ \r\n");
			/* Task character reset */
			for(i=1; i<NETWORK_FLOW; i++){
				Task_character[i][TASK_PERIOD] = original_period[i];
				Task_character[i][TASK_DEAD] = original_deadline[i];
			}
			/* adaptability parameter */
			/* random_slot_seq += 1;
			if(random_slot_seq == 5){
				random_slot_seq = 0;
				random_slot_idx += 1;
				if(random_slot_idx == 10){
					random_slot_idx = 0;
				}
				random_slot = random_slots[random_slot_idx];
			} */



			atomic phyLock = FALSE;
			return;
		}

		/* adaptability check : Task period changes */
		/* if(TOS_NODE_ID == 2 && currentSlot == random_slot){
			Task_character[my_flow_id][TASK_PERIOD] = changed_period[my_flow_id];
			Task_character[my_flow_id][TASK_DEAD] = changed_deadline[my_flow_id];
		} */

		if(TOS_NODE_ID == 0 && slot == 0){
			notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
			notificationPacket->nextSlot = currentSlot+1;
			notificationPacket->nextAlarm = call SlotterControl.getNow();
			//  schedule_idx will be increase in phyNotificationSend.sendDone function
			atomic tmp_phyLock = phyLock;
			if(!tmp_phyLock) {
				if(call phyNotificationSend.send(AM_BROADCAST_ADDR, &selfNotificationPkt, sizeof(notification_t)) == SUCCESS)  //broadcast_route[TOS_NODE_ID % 10][notificationPacket->curId]
					atomic phyLock = TRUE;
				else
					printf("<Mac>:  notifPkt <FAIL>  \r\n");
			}
			else
					printf("<Mac-nofiPkt-Fail>: Radio locked  \r\n");
		}

		/* Synchronization */
		if(slot >0 && slot < flows_start_slot){
			if(schedule[mapping][1] == TOS_NODE_ID && schedule[mapping][2] == 100) {	//sender and receiver check
				myGlobalTime = call GlobalTime.getLocalTime();
				inSync = (uint32_t) call GlobalTime.local2Global(&myGlobalTime);
				remaining = call SlotterControl.getRemaining();

				notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
				notificationPacket->nextSlot = slot + 1;
				notificationPacket->nextAlarm = myGlobalTime + (remaining / 32);
				atomic tmp_phyLock = phyLock;
				if(!tmp_phyLock){
				 if(call phyNotificationSend.send(AM_BROADCAST_ADDR, &selfNotificationPkt, sizeof(notification_t)) == SUCCESS){  //broadcast_route[TOS_NODE_ID % 10][notificationPacket->curId]
					 atomic phyLock = TRUE;
				 }else{
					 printf("<Mac>:  notifPkt <FAIL>  \r\n");
				 }
				}else{
				 printf("<Mac-nofiPkt-Fail>: Radio locked  \r\n");
				}
			}
		}
		/* flow transmission */
		else{
			/* Flow root transmission */
			flow_start_slot = slot - flows_start_slot - my_release_offset;
			if(is_root == TRUE){
				// Reset variables every deadline
				if(slot == flow_deadline[my_flow_id])
					reset_var(my_flow_id);	// reset is_AH_pkt_null, tx_opper

				// generate a new packet
				if(flow_start_slot >= 0 && (flow_start_slot % Task_character[my_flow_id][TASK_PERIOD]) == 0){
					//printf("new packet generates: slot %d | task_period %d | flow_start_slot %d\r\n", currentSlot, Task_character[my_flow_id][TASK_PERIOD], flow_start_slot);
					atomic is_AH_pkt_null = FALSE;
					atomic tx_opper[my_flow_id] = Task_character[my_flow_id][TASK_MAXTX];
					flow_deadline[my_flow_id] = slot + Task_character[my_flow_id][TASK_DEAD];
					root_seq += 1;
					AH_payload = (adaptivehart_msg_t *)call Packet.getPayload(&AH_pkt, sizeof(adaptivehart_msg_t));
					set_payload(AH_payload, my_flow_id, HI_TASK, tx_opper[my_flow_id], flow_deadline[my_flow_id], root_seq);
					return;
				}
				//(re)transmission condition
				atomic tmp_tx_opper = tx_opper[my_flow_id];
				if(flow_start_slot >= 0 && tmp_tx_opper > 0 && is_AH_pkt_null == FALSE){
					offset_algorithm(my_flow_id);
				}
			}
			/* Relay node transmission */
			else{
				if(!call HIforwardQ.empty() || !call LOforwardQ.empty()){	// whether there is a pakcet to send
					if(!call HIforwardQ.empty()){
						Q_payload = (adaptivehart_msg_t *)call HIforwardQ.head();
					}else if(!call LOforwardQ.empty()){
						Q_payload = (adaptivehart_msg_t *)call LOforwardQ.head();
					}
					tmp_flowid = Q_payload -> flowid;

					// Reset variables every deadline
					if(Q_payload->deadline < slot){
						reset_var(tmp_flowid);
					}



					// (re)transmission condition
					atomic tmp_tx_opper = tx_opper[tmp_flowid];
					//atomic tmp_is_dest = is_dest[tmp_flowid];
					if(slot >= rcv_slot[tmp_flowid] + 1 && tmp_tx_opper > 0 && is_dest[tmp_flowid] == FALSE){
						atomic txing_flowid = tmp_flowid;
						AH_payload = (adaptivehart_msg_t *)call Packet.getPayload(&AH_pkt, sizeof(adaptivehart_msg_t));
						set_payload(AH_payload, Q_payload->flowid, Q_payload->priority, Q_payload->maxtx, Q_payload->deadline, Q_payload->seq);
						offset_algorithm(txing_flowid);
						//send_AH_Pkt(&AH_pkt, my_dest[txing_flowid], txing_flowid);
					}
				}
			}

			/* For only node debug */
			/* if(flow_start_slot >= 0 && (flow_start_slot % Task_character[2][TASK_PERIOD]) == 0 && is_root == FALSE){
				printf("dbg: last received pkt at %d from %d --- inverval %ld\r\n", last_rcv_slot, last_src, diff);
			} */
		}
	}



	event void phyNotificationSend.sendDone(message_t *msg, error_t error){
		atomic phyLock = FALSE;
		signal MacSend.sendDone(msg, error);
	}

	event message_t * phyNotificationReceive.receive(message_t *msg, void *payload, uint8_t len){
		/* prepare for forwarding for notification_t*/
		notification_t* notifyPacketTemp =(notification_t *)payload;
		am_addr_t src;
		uint32_t gap;
		//uint8_t nextSlot;
		//uint32_t nextAlarm, myNextAlarm;

		src = (am_addr_t )call AMPacket.source(msg);
		//printf("src = %d || slot:%d\r\n",src, currentSlot);
		if(src == broadcastMatrix[TOS_NODE_ID]) {

			//nextSlot = notifyPacketTemp->nextSlot;
			//nextAlarm = notifyPacketTemp->nextAlarm;

			myGlobalTime = call GlobalTime.getLocalTime();
			//inSync = (uint32_t) call GlobalTime.local2Global(&myGlobalTime);
			//myNextAlarm = myGlobalTime + (call SlotterControl.getRemaining() / 32);

			atomic toSlot = notifyPacketTemp->nextSlot;
			gap = (notifyPacketTemp->nextAlarm - myGlobalTime) * 32;

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
		uint8_t tmp_tx_opper, tmp_txing_flowid;

		if(is_root == TRUE){
			atomic tmp_txing_flowid = my_flow_id;
		}else{
			atomic tmp_txing_flowid = txing_flowid;
		}

		if(call ACK.wasAcked(msg)) {
			/* Reset schedule variables : tx_opper, Q, rcv_slot */
			reset_var(tmp_txing_flowid);
		}else{
			/* if maximum tx failure occurs, reset schedule variables */
			atomic tmp_tx_opper = tx_opper[tmp_txing_flowid];
			if(tmp_tx_opper <= 0){
				reset_var(tmp_txing_flowid);
				//printf("dbg: Final Retx fail\r\n");
			}
		}

		atomic txing_flowid = 0;
		atomic phyLock = FALSE;
		signal MacSend.sendDone(msg, error);
	}



	event message_t * phyDataReceive.receive(message_t *msg, void *payload, uint8_t len){
		uint8_t rcv_flow_id;
		uint16_t rcv_flow_seq, tmp_rcv_slot;
		am_addr_t src;
		adaptivehart_msg_t *rcv_AH_payload = (adaptivehart_msg_t *)payload;
		adaptivehart_msg_t *tmp_Q_payload = (adaptivehart_msg_t *)NULL;
		bool tmp_is_dest;


		rcv_flow_id = rcv_AH_payload -> flowid;
		rcv_flow_seq = rcv_AH_payload->seq;
		/* check duplicate packet receive */
		if(prev_seq[rcv_flow_id] != rcv_flow_seq){
			prev_seq[rcv_flow_id] = rcv_flow_seq;
			tmp_is_dest = is_dest[rcv_flow_id];

			src = (am_addr_t )call AMPacket.source(msg);
			tmp_rcv_slot = call SlotterControl.getSlot();
			//printf("Rx: flow %d at %d from %d\r\n", rcv_flow_id, tmp_rcv_slot, src);

			/* for debug, receiving info is printed later */
			if(rx_flow_seq[rcv_flow_id]<=FLOW_SEQ){
				rcv_slots[rcv_flow_id][rx_flow_seq[rcv_flow_id]] = tmp_rcv_slot;
				rcv_srcs[rcv_flow_id][rx_flow_seq[rcv_flow_id]] = src;
				rx_flow_seq[rcv_flow_id] += 1;
			}

			/* Relay Q */
			if(tmp_is_dest == FALSE){
				/* check packet priority : High or Low */
				if(rcv_flow_id == HI_TASK){
					HIqueueSize = call HIforwardQ.size();
					tmp_Q_payload = (adaptivehart_msg_t *) call Packet.getPayload(&HIforwardPktBuffer[HIqueueSize], sizeof(adaptivehart_msg_t));
					set_payload(tmp_Q_payload, rcv_flow_id, rcv_AH_payload->priority, rcv_AH_payload->maxtx, rcv_AH_payload->deadline, rcv_flow_seq);
					atomic tx_opper[rcv_flow_id] 		= rcv_AH_payload -> maxtx;
					atomic rcv_slot[rcv_flow_id] 		= tmp_rcv_slot;
					call HIforwardQ.enqueue(tmp_Q_payload);
				}else if(rcv_flow_id == LO_TASK){
					LOqueueSize = call LOforwardQ.size();
					tmp_Q_payload = (adaptivehart_msg_t *) call Packet.getPayload(&LOforwardPktBuffer[LOqueueSize], sizeof(adaptivehart_msg_t));
					set_payload(tmp_Q_payload, rcv_flow_id, rcv_AH_payload->priority, rcv_AH_payload->maxtx, rcv_AH_payload->deadline, rcv_flow_seq);
					atomic tx_opper[rcv_flow_id] 		= rcv_AH_payload -> maxtx;
					atomic rcv_slot[rcv_flow_id] 		= tmp_rcv_slot;
					call LOforwardQ.enqueue(tmp_Q_payload);
				}
			}

		}


		return msg;
	}

	/* after txoffset interval, it sends a prepared packet */
	// if it does not receive any packet during TxOffset, then do transmission
	async event void TxOffset_relay.fired(){
		if(rcv_indicator == FALSE){
			send_AH_Pkt(&AH_pkt, my_dest[txing_flowid], txing_flowid);
		}
		//else{
			//printf("MAC: preemption at %d\r\n", call SlotterControl.getSlot());
		//}
	}

	async event void TxOffset_root.fired(){
		send_AH_Pkt(&AH_pkt, my_dest[my_flow_id], my_flow_id);
	}

	event void dbg_timer2.fired(){}


	/* PHY layer packet receive indicator */
	event void ReceiveIndicator.anypktreceive(){
		if(rcv_indicator == FALSE && my_txoffset[LO_TASK] >= 5){
			atomic rcv_indicator = TRUE;
		}
	}

	command error_t MacSend.send(message_t * msg, uint8_t len) {atomic{return SUCCESS;}}
 	command error_t MacSend.cancel(message_t *msg) {return call phyDataSend.cancel(msg);}
	command uint8_t MacSend.maxPayloadLength(){return 0;}
	command void * MacSend.getPayload(message_t *msg, uint8_t len){return msg;}
	async event void SyncAlarm.fired() {call SlotterControl.synchronize(toSlot);}
  command void Frame.setSlotLength(uint32_t slotTimeBms) {
		atomic slotSize = slotTimeBms;
		call FrameConfiguration.setSlotLength(slotSize);
		return;
 	}
 	command void Frame.setFrameLength(uint16_t numSlots) {
 		atomic currFrameSize = numSlots;
 		call FrameConfiguration.setFrameLength(currFrameSize);
 		return;
 	}
 	command uint32_t Frame.getSlotLength() {
		atomic slotSize = call FrameConfiguration.getSlotLength();
 		return slotSize;
 	}
 	command uint16_t Frame.getFrameLength() {
 		uint16_t frameLength;
 		atomic frameLength = call FrameConfiguration.getFrameLength();
 		return frameLength;
 	}

	/* Functions */
	void send_AH_Pkt(message_t *packet, am_addr_t dest, uint8_t flowid){
		bool tmp_phyLock;

		atomic tmp_phyLock = phyLock;
		if (tmp_phyLock == FALSE) {
			call ACK.requestAck(packet);
			if(call phyDataSend.send(dest, packet, sizeof(adaptivehart_msg_t)) == SUCCESS){
				atomic phyLock = TRUE;
				atomic tx_opper[flowid] = tx_opper[flowid] - 1;
				if(is_root == TRUE && tx_flow_seq <= FLOW_SEQ){
					tx_slots[tx_flow_seq] = call SlotterControl.getSlot();
					tx_flow_seq += 1;
				}else if(is_root == FALSE && tx_flow_seq <= FLOW_SEQ){
					tx_slots[tx_flow_seq] = call SlotterControl.getSlot();
					tx_flow_seq += 1;
				}

				/* if(is_root == TRUE){
					printf("Tx: at %d and tx_opper:%d\r\n", currentSlot, tx_opper[flowid]);
				} */
			}else{
				printf("MAC: Pkt fail  \r\n");
			}
		}else{
			printf("MAC: Radio Locked \r\n");
		}
	}

	void set_payload(adaptivehart_msg_t *payload, uint8_t flowid, uint8_t priority, uint8_t maxtx, uint16_t deadline, uint16_t seq){
		payload -> flowid = flowid;
		payload -> priority = priority;
		payload -> maxtx = maxtx;
		payload -> deadline = deadline;
		payload -> seq = seq;
	}

	void Q_reset(uint8_t flowid){
		uint8_t queueSize;
		if(flowid == HI_TASK){
			queueSize = call HIforwardQ.size();
			for(i=0; i<queueSize; i++){
				call HIforwardQ.dequeue();
			}
			//printf("HIQ reset\r\n");
		}else if(flowid == LO_TASK){
			queueSize = call LOforwardQ.size();
			for(i=0; i<queueSize; i++){
				call LOforwardQ.dequeue();
			}
			//printf("LOQ reset\r\n");
		}
	}
	/* Reset rcv_slot, tx_opper, Q buffer */
	void reset_var(uint8_t flowid){
		if(is_root == TRUE){
			atomic is_AH_pkt_null = TRUE;
		}else{
			atomic rcv_slot[flowid] = 0;
			Q_reset(flowid);
		}
		atomic tx_opper[flowid] = 0;
	}

	/* TxOffset algorithm: delay the offset interval before tx */
	void offset_algorithm(uint8_t tmp_txing_flowid){
		if(is_root == TRUE){	// root nodes
			atomic rcv_indicator = FALSE;
			if(tmp_txing_flowid == LO_TASK){
				call TxOffset_root.start(32 * my_txoffset[LO_TASK]);
			}else{
				send_AH_Pkt(&AH_pkt, my_dest[tmp_txing_flowid], tmp_txing_flowid);
			}
		}else{	// relay nodes
			atomic rcv_indicator = FALSE;
			if(tmp_txing_flowid == LO_TASK){
				call TxOffset_relay.start(32 * my_txoffset[LO_TASK]);
			}else{
				send_AH_Pkt(&AH_pkt, my_dest[tmp_txing_flowid], tmp_txing_flowid);
			}
		}
	}

}
