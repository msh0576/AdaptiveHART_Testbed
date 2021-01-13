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
		interface AdaptiveHART_Util as AH_Util;
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
		MAC_Q = 2,
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
	uint8_t currentSlot;
	uint8_t toSlot;
	uint32_t tempGlobal;
	uint8_t superFrameLength,currFrameSize;
	uint8_t bi, sd;

	/* AdaptiveHART variables */
	uint8_t i;
	uint32_t prev_seq = 0;
	bool ReTx = FALSE;
	uint8_t flows_start_slot = 0;
	bool is_root = FALSE;
	uint8_t my_release_offset = 0;
	uint8_t my_flow_id = 0;

	uint8_t broadcastMatrix[20] = {0, 0, 4, 4, 0, 4, 4, 4, 6, 8, 8, 10, 10, 12, 12, 14, 14, 16, 16, 18};

	/* For message */
	message_t AH_pkt;
	adaptivehart_msg_t *AH_payload;

	/* For Queue */
	message_t HIforwardPktBuffer[MAC_Q];
	message_t LOforwardPktBuffer[MAC_Q];
	uint8_t HIqueueSize;
	uint8_t LOqueueSize;

	/* For schedule and relay */
	uint16_t rcv_slot[NETWORK_FLOW];
	uint8_t tx_opper[NETWORK_FLOW];
	uint8_t txing_flowid = 0;


	/* For debug */
	uint16_t Rcv_count[NETWORK_FLOW];
	uint16_t Tx_count[NETWORK_FLOW];
	uint16_t Total_tx_count[NETWORK_FLOW];

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
  uint32_t gigaframe_length = 100; //5Hz at most     //***  10 == NodeNumber(schedule_len + 1) + controlschedule_len
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
		{25, 25, 2},
		{25, 25, 2}
	};
	uint8_t HI_TASK = 1;
	uint8_t LO_TASK = 2;
	uint8_t TASK_PERIOD = 0;
	uint8_t TASK_DEAD = 1;
	uint8_t TASK_MAXTX = 2;

	/* Routing path */

	/* Functions */
	void send_AH_Pkt(message_t *packet, am_addr_t dest);
	void set_payload(message_t *packet, uint8_t flowid, uint8_t priority, uint8_t maxtx);
	void Q_reset(uint8_t flowid);

	event void Boot.booted() {
		call RadioPowerControl.start();
	}

	command error_t Init.init() {

		/** @brief Initialization of necessary variables */
		currentSlot = 0xff;
		slotSize = 10 * 32; // 10ms standard
		inSync = 0; 		// Set to zero initially since nodes are not synchronized

		/** @brief Boolean */
		init = FALSE;
		requestStop = FALSE;
		radioOff = TRUE;
		phyLock = FALSE;
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

		/* initialize schedule variables */
		for(i=0; i<NETWORK_FLOW; i++){
			tx_opper[i] = 0;
			rcv_slot[i] = 0;
		}

		/* set flow root, release offset and its period */
		is_root = call AH_Util.is_root(TOS_NODE_ID);
		if(is_root == TRUE){
			my_release_offset = call AH_Util.get_release_offset(TOS_NODE_ID);
			my_flow_id = call AH_Util.get_flow_id(TOS_NODE_ID);
			tx_opper[my_flow_id] = Task_character[my_flow_id][TASK_MAXTX];
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
		uint8_t mapping;
		uint32_t remaining;
		uint16_t flow_slot = -1;
		adaptivehart_msg_t *Q_payload = (adaptivehart_msg_t *)NULL;

		atomic currentSlot = slot;
		mapping = slot;

		/* Reset Q, variables */
		if(slot == 0 && TOS_NODE_ID != 0) {
			printf(" ------- new Frame ------ \r\n");
			atomic phyLock = FALSE;
			atomic ReTx = FALSE;
			return;
		}

		if(TOS_NODE_ID == 0 && slot == 0){
			notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
			notificationPacket->nextSlot = currentSlot+1;
			notificationPacket->nextAlarm = call SlotterControl.getNow();
			//  schedule_idx will be increase in phyNotificationSend.sendDone function
			if(!phyLock) {
				if(call phyNotificationSend.send(AM_BROADCAST_ADDR, &selfNotificationPkt, sizeof(notification_t)) == SUCCESS)  //broadcast_route[TOS_NODE_ID % 10][notificationPacket->curId]
					atomic phyLock = TRUE;
				else
					printf("<Mac>:  notifPkt <FAIL>  \r\n");
			}
			else
					printf("<Mac-nofiPkt-Fail>: Radio locked  \r\n");
		}

		/** @brief Notification slot for sink */
		//printf("slot:%d, mapping:%d, schedule[mapping][1]:%d\r\n", slot, mapping, schedule[mapping][1]);
		if(slot >0 && slot < flows_start_slot){
			if(schedule[mapping][1] == TOS_NODE_ID && schedule[mapping][2] == 100) {	//sender and receiver check

				myGlobalTime = call GlobalTime.getLocalTime();
				inSync = (uint32_t) call GlobalTime.local2Global(&myGlobalTime);
				remaining = call SlotterControl.getRemaining();

				notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
				notificationPacket->nextSlot = slot + 1;
				notificationPacket->nextAlarm = myGlobalTime + (remaining / 32);

				if(!phyLock){
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

			if(is_root == TRUE){	// flow root check
				flow_slot = slot - flows_start_slot - my_release_offset;
				if(flow_slot >= 0 && (flow_slot % Task_character[my_flow_id][TASK_PERIOD]) == 0){	// their tx slot check
					// here, need to add retransmission
					atomic txing_flowid = my_flow_id;
					set_payload(&AH_pkt, my_flow_id, 1, Task_character[my_flow_id][TASK_MAXTX]);
					send_AH_Pkt(&AH_pkt, 3);
				}
			}else{	// Relay nodes
				if(!call HIforwardQ.empty() || !call LOforwardQ.empty()){	// whether there is a pakcet to be sent
					if(!call HIforwardQ.empty()){
						Q_payload = (adaptivehart_msg_t *)call HIforwardQ.head();
					}else if(!call LOforwardQ.empty()){
						Q_payload = (adaptivehart_msg_t *)call LOforwardQ.head();
					}
					atomic txing_flowid = Q_payload -> flowid;
					if(rcv_slot[txing_flowid] + 1 >= slot && tx_opper[txing_flowid] > 0){	// check tx slot condition
						set_payload(&AH_pkt, txing_flowid, Q_payload->priority, Q_payload->maxtx);
						send_AH_Pkt(&AH_pkt, 5);
					}
				}
			}
			

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

			myGlobalTime = call GlobalTime.getLocalTime();
			inSync = (uint32_t) call GlobalTime.local2Global(&myGlobalTime);
			myNextAlarm = myGlobalTime + (call SlotterControl.getRemaining() / 32);

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

		if(call ACK.wasAcked(msg)) {
			atomic ReTx = TRUE;
			/* Reset schedule variables */
			// tx_opper -> 0, Q dequeue, rcv_slot -> 0
			rcv_slot[txing_flowid] = 0;
			tx_opper[txing_flowid] = 0;
			Q_reset(txing_flowid);

		}else{
			// if maximum tx failure occurs, reset schedule variables
			// else tx_opper decreases,
		}
		atomic txing_flowid = 0;

		/** @brief Clearing send lock once successfully sent on channel */
		atomic phyLock = FALSE;
		signal MacSend.sendDone(msg, error);
	}



	event message_t * phyDataReceive.receive(message_t *msg, void *payload, uint8_t len){
		uint8_t rcv_flow_id, rcv_priority, rcv_maxtx;
		adaptivehart_msg_t *rcv_AH_payload = (adaptivehart_msg_t *)payload;
		adaptivehart_msg_t *tmp_Q_payload = (adaptivehart_msg_t *)NULL;

		// check duplicate packet
		// check packet priority : High or Low
		rcv_flow_id = rcv_AH_payload -> flowid;
		rcv_priority = rcv_AH_payload -> priority;
		rcv_maxtx = rcv_AH_payload -> maxtx;
		rcv_slot[rcv_flow_id] = call SlotterControl.getSlot();
		printf("Rx OK\r\n");
		if(rcv_priority == 1){

			HIqueueSize = call HIforwardQ.size();
			tmp_Q_payload = (adaptivehart_msg_t *) call Packet.getPayload(&HIforwardPktBuffer[HIqueueSize], sizeof(adaptivehart_msg_t));
			tmp_Q_payload -> flowid = rcv_flow_id;
			tmp_Q_payload -> priority = rcv_priority;
			tmp_Q_payload -> maxtx = rcv_maxtx;
			tx_opper[rcv_flow_id] = rcv_maxtx;
			// ?? set txopper[link], txdelay[link]
			call HIforwardQ.enqueue(tmp_Q_payload);

		}


		return msg;
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

	/* Functions */
	void send_AH_Pkt(message_t *packet, am_addr_t dest){
		if (phyLock == FALSE) {
			call ACK.requestAck(packet);
			if(call phyDataSend.send(dest, packet, sizeof(adaptivehart_msg_t)) == SUCCESS){
				atomic phyLock = TRUE;
			}else{
				printf("MAC: Pkt fail  \r\n");
			}
		}else{
			printf("MAC: Radio Locked \r\n");
		}
	}

	void set_payload(message_t *packet, uint8_t flowid, uint8_t priority, uint8_t maxtx){
		AH_payload = (adaptivehart_msg_t *)call Packet.getPayload(packet, sizeof(adaptivehart_msg_t));
		AH_payload -> flowid = flowid;
		AH_payload -> priority = priority;
		AH_payload -> maxtx = maxtx;
		tx_opper[flowid] = tx_opper[flowid] - 1;
	}

	void Q_reset(uint8_t flowid){
		uint8_t queueSize;
		if(flowid == HI_TASK){
			queueSize = call HIforwardQ.size();
			for(i=0; i<queueSize; i++){
				call HIforwardQ.dequeue();
			}
		}else if(flowid == LO_TASK){
			queueSize = call LOforwardQ.size();
			for(i=0; i<queueSize; i++){
				call LOforwardQ.dequeue();
			}
		}

	}

}
