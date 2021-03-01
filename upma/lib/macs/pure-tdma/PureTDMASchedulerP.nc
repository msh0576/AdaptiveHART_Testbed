/*
 * "Copyright (c) 2007 Washington University in St. Louis.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL WASHINGTON UNIVERSITY IN ST. LOUIS BE LIABLE TO ANY PARTY
 * FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING
 * OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF WASHINGTON
 * UNIVERSITY IN ST. LOUIS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * WASHINGTON UNIVERSITY IN ST. LOUIS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND WASHINGTON UNIVERSITY IN ST. LOUIS HAS NO
 * OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
 * MODIFICATIONS."
 */

/**
 *
 * @author Octav Chipara
 * @version $Revision$
 * @date $Date$
 */

#include "printf.h"
#include "TestPureTdma.h"

module PureTDMASchedulerP {
	provides {
		interface Init;
		interface SplitControl;
		interface AsyncSend as Send;
		interface AsyncReceive as Receive;
		interface CcaControl[am_id_t amId];
		interface FrameConfiguration as Frame;
	}
	uses{
		interface AsyncStdControl as GenericSlotter;
		interface RadioPowerControl;
		interface Slotter;
		interface SlotterControl;
		interface FrameConfiguration;
		interface AsyncSend as SubSend;
		interface AsyncSend as BeaconSend;
		interface AsyncReceive as SubReceive;
    ///
    //interface AsyncSend as CommandSend;

		interface AMPacket;
		interface Resend;
		interface PacketAcknowledgements as ACK;

		interface Boot;
		interface Leds;

		//Added by Sihoon
		interface Queue<adaptivehart_msg_t *> as HIforwardQ;
		interface Queue<adaptivehart_msg_t *> as LOforwardQ;
		interface Queue<adaptivehart_msg_t *> as RootQ;
		interface AdaptiveHART_Util as AH_Util;
		interface Alarm<T32khz, uint16_t> as TxOffset_alarm;
		interface ReceiveIndicator;

	}
}
implementation {
	enum {
		SIMPLE_TDMA_SYNC = 123,
		MAC_Q = 1,
		FLOW_SEQ = 20,
		SCHEDULE_LEN = 20,
		SCHEDULE_CHAR_LEN = 6,
	};

	bool init;
	uint32_t slotSize;
	uint32_t bi, sd, cap;
	uint8_t coordinatorId;
	message_t *toSend;
	uint8_t toSendLen;
	uint8_t currentSlot;
	bool sync;
	bool requestStop;
	uint32_t *alarmTime;

  message_t packet;


	/* AdaptiveHART variables */
	bool phyLock = FALSE; 			// Radio lock to prevent double send
	uint8_t i;
	uint8_t flows_start_slot = 2;
	bool is_root = FALSE;
	bool is_dest[NETWORK_FLOW];
	uint8_t my_release_offset = 0;
	uint8_t my_flow_id = 0;

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
	uint32_t giga_seq = 0;

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
		{0, 0, 4},
		{0, 0, 4}
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

	/* WirelessHART */
	uint32_t gigaframe_length = 30; //5Hz at most     //***  10 == NodeNumber(schedule_len + 1) + controlschedule_len

	// 0: slot
	// 1: sender
	// 2: receiver
	// 3: flowid
	// 4: flow root
	// 5: tx_oppertunity
	// 6: send status 0: no-ack, 1: ack
	uint8_t schedule[30][6] = {
	};
	uint8_t origin_schedule[30][6] = {
		/* Retx2 */
		/* {2, 1, 3, 1, 1, 2},
		{3, 1, 3, 1, 1, 1},
		{4, 3, 5, 1, 1, 2},
		{5, 3, 5, 1, 1, 1},
		{6, 5, 6, 1, 1, 2},
		{7, 5, 6, 1, 1, 1},
		{8, 2, 3, 2, 2, 2},
		{9, 2, 3, 2, 2, 1},
		{10, 3, 5, 2, 2, 2},
		{11, 3, 5, 2, 2, 1},
		{12, 5, 7, 2, 2, 2},
		{13, 5, 7, 2, 2, 1} */
		/* Retx3 */
		/* {2, 1, 3, 1, 1, 3},
		{3, 1, 3, 1, 1, 2},
		{4, 1, 3, 1, 1, 1},
		{5, 3, 5, 1, 1, 3},
		{6, 3, 5, 1, 1, 2},
		{7, 3, 5, 1, 1, 1},
		{8, 5, 6, 1, 1, 3},
		{9, 5, 6, 1, 1, 2},
		{10, 5, 6, 1, 1, 1},
		{11, 2, 3, 2, 2, 3},
		{12, 2, 3, 2, 2, 2},
		{13, 2, 3, 2, 2, 1},
		{14, 3, 5, 2, 2, 3},
		{15, 3, 5, 2, 2, 2},
		{16, 3, 5, 2, 2, 1},
		{17, 5, 7, 2, 2, 3},
		{18, 5, 7, 2, 2, 2},
		{19, 5, 7, 2, 2, 1} */
		/* Retx4 */
		{2, 1, 3, 1, 1, 4},
		{3, 1, 3, 1, 1, 3},
		{4, 1, 3, 1, 1, 2},
		{5, 1, 3, 1, 1, 1},
		{6, 3, 5, 1, 1, 4},
		{7, 3, 5, 1, 1, 3},
		{8, 3, 5, 1, 1, 2},
		{9, 3, 5, 1, 1, 1},
		{10, 5, 6, 1, 1, 4},
		{11, 5, 6, 1, 1, 3},
		{12, 5, 6, 1, 1, 2},
		{13, 5, 6, 1, 1, 1},
		{14, 2, 3, 2, 2, 4},
		{15, 2, 3, 2, 2, 3},
		{16, 2, 3, 2, 2, 2},
		{17, 2, 3, 2, 2, 1},
		{18, 3, 5, 2, 2, 4},
		{19, 3, 5, 2, 2, 3},
		{20, 3, 5, 2, 2, 2},
		{21, 3, 5, 2, 2, 1},
		{22, 5, 7, 2, 2, 4},
		{23, 5, 7, 2, 2, 3},
		{24, 5, 7, 2, 2, 2},
		{25, 5, 7, 2, 2, 1}
	};
	uint8_t changed_schedule[30][6] = {
		/* Retx2 */
		{2, 1, 3, 1, 1, 2},
		{3, 1, 3, 1, 1, 1},
		{4, 3, 5, 1, 1, 2},
		{5, 3, 5, 1, 1, 1},
		{6, 5, 6, 1, 1, 2},
		{7, 5, 6, 1, 1, 1},
		{8, 2, 3, 2, 2, 2},
		{9, 2, 3, 2, 2, 1},
		{10, 3, 5, 2, 2, 2},
		{11, 3, 5, 2, 2, 1},
		{12, 5, 7, 2, 2, 2},
		{13, 5, 7, 2, 2, 1}
	};
	uint8_t changed_schedule_LO_late[30][6] = {
		{2, 1, 3, 1, 1, 2},
		{3, 1, 3, 1, 1, 1},
		{4, 3, 5, 1, 1, 2},
		{5, 3, 5, 1, 1, 1},
		{6, 5, 6, 1, 1, 2},
		{7, 5, 6, 1, 1, 1},
		{8, 2, 3, 2, 2, 2},
		{9, 2, 3, 2, 2, 1},
		{10, 3, 5, 2, 2, 2},
		{11, 3, 5, 2, 2, 1},
		{12, 5, 7, 2, 2, 2},
		{13, 5, 7, 2, 2, 1}
	};
	uint8_t schedule_len = 30;
	uint8_t schedule_char_len = 6;
	uint8_t SLOT = 0; SENDER = 1; RECEIVER = 2; FLOWID = 3; FLOWROOT = 4; TX_OPPER = 5; SEND_STATUS = 6;
	bool schedule_reset_flag = FALSE;
	bool schedule_change_flag = FALSE;

	/* Functions */
	void send_AH_Pkt(message_t *packet, am_addr_t dest, uint8_t flowid);
	void set_payload(adaptivehart_msg_t *payload, uint8_t flowid, uint8_t priority, uint8_t maxtx, uint16_t deadline, uint16_t seq);
	void Q_reset(uint8_t flowid);
	void reset_var(uint8_t flowid);
	void offset_algorithm(uint8_t tmp_txing_flowid);
	void reset_dbg_var();
	void WH_send(uint8_t slot);
	void schedule_reset_WH(uint8_t target_schedule[][6], uint8_t col, uint8_t row);
//////////////////////////////////////////////////////////////////////////////////////////////////

	event void Boot.booted()
	{
	}

	command error_t Init.init() {
		uint8_t j;
		currentSlot = 0xff;
		slotSize = 100 * 32;     //10ms
		//bi = 30; //# of slots
		//sd = 28; //last active slot
    bi = 4000; //gigaframe_length; //# of slots
		sd = 4000; //gigaframe_length; //last active slot
		cap = 0;
		coordinatorId = 0;
		init = FALSE;
		toSend = NULL;
		toSendLen = 0;
		sync = FALSE;

		atomic phyLock = FALSE;

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

		/* adaptability parameter init - AH*/
		random_slot = random_slots[random_slot_idx];

		/* adaptability parameter init - WH*/
		schedule_reset_WH(origin_schedule, schedule_len, schedule_char_len);

		printf("dbg: HI_txoffset:%d, and LO_txoffset:%d\r\n", my_txoffset[1], my_txoffset[2]);

		return SUCCESS;
	}

 	command error_t SplitControl.start() {
 		error_t err;
 		if (init == FALSE) {
 			call FrameConfiguration.setSlotLength(slotSize);
 			call FrameConfiguration.setFrameLength(bi+1);
 		}

 		err = call RadioPowerControl.start();
 		return err;
 	}


 	command error_t SplitControl.stop() {
 		requestStop = TRUE;
 		call GenericSlotter.stop();
 		call RadioPowerControl.stop();
 		return SUCCESS;
 	}

 	event void RadioPowerControl.startDone(error_t error) {
 		//call Leds.led2On();
 		if (coordinatorId == TOS_NODE_ID) {
 			if (init == FALSE) {
 				signal SplitControl.startDone(error);
 				call GenericSlotter.start();
 				call SlotterControl.synchronize(0);
 				init = TRUE;
 			}
 		} else {
 			if (init == FALSE) {
 				signal SplitControl.startDone(error);
 				init = TRUE;
 			}
 		}


	}

 	event void RadioPowerControl.stopDone(error_t error)  {
		if (requestStop)  {
			requestStop = FALSE;
			signal SplitControl.stopDone(error);
		}
	}


 	async event void Slotter.slot(uint8_t slot) {
    uint8_t tmp_slot;
		int16_t flow_start_slot = -1;
		adaptivehart_msg_t *Q_payload = (adaptivehart_msg_t *)NULL;
		uint8_t tmp_tx_opper;
		uint8_t tmp_flowid = 0;

    tmp_slot = slot;
 		//printf("slot,%d\r\n",tmp_slot);
    atomic currentSlot = tmp_slot;

		if(tmp_slot == gigaframe_length-1){
			if(is_root == TRUE){
				printf("random_slot: %d, Tx: [ %d, %d, %d, %d, %d, %d, %d, %d, %d, %d ]\r\n", random_slot, tx_slots[0], tx_slots[1], tx_slots[2], tx_slots[3], tx_slots[4], tx_slots[5], tx_slots[6], tx_slots[7], tx_slots[8], tx_slots[9]);
				tx_slots[0]=0; tx_slots[1]=0; tx_slots[2]=0; tx_slots[3]=0; tx_slots[4]=0; tx_slots[5]=0; tx_slots[6]=0; tx_slots[7]=0; tx_slots[8]=0; tx_slots[9]=0;
				tx_flow_seq=0;
			}else{
				printf("HI_TASK: Rx: [ %d, %d, %d, %d, %d ]\r\n", rcv_slots[HI_TASK][0], rcv_slots[HI_TASK][1], rcv_slots[HI_TASK][2], rcv_slots[HI_TASK][3], rcv_slots[HI_TASK][4]);
				printf("LO_TASK: Rx: [ %d, %d, %d, %d, %d ]\r\n", rcv_slots[LO_TASK][0], rcv_slots[LO_TASK][1], rcv_slots[LO_TASK][2], rcv_slots[LO_TASK][3], rcv_slots[LO_TASK][4]);
				rcv_slots[HI_TASK][0]=0; rcv_slots[HI_TASK][1]=0; rcv_slots[HI_TASK][2]=0; rcv_slots[HI_TASK][3]=0; rcv_slots[HI_TASK][4]=0;
				rcv_slots[LO_TASK][0]=0; rcv_slots[LO_TASK][1]=0; rcv_slots[LO_TASK][2]=0; rcv_slots[LO_TASK][3]=0; rcv_slots[LO_TASK][4]=0;
				rx_flow_seq[HI_TASK] = 0;
				rx_flow_seq[LO_TASK] = 0;
			}
			printfflush();
			return;
		}

 		if (tmp_slot == 0) {
 			//beacon slot
 			if (coordinatorId == TOS_NODE_ID) {
 				call BeaconSend.send(NULL, 0);
 			}
 			return;
 		}else if(tmp_slot == 1){
			//printf("slot:%d, tmp_slot:%d\r\n", slot, tmp_slot); printfflush();
			if(TOS_NODE_ID != coordinatorId){

				/* for debug, every frame a receive information is printed at a 0 slot */
				printf("<New Frame:%ld>\r\n", giga_seq);
				giga_seq += 1;
				reset_var(HI_TASK);
				reset_var(LO_TASK);


				/* Task character reset - AdaptiveHART*/
				/* for(i=1; i<NETWORK_FLOW; i++){
					Task_character[i][TASK_PERIOD] = original_period[i];
					Task_character[i][TASK_DEAD] = original_deadline[i];
				} */

				/* adaptability check : schedule changes - WH */
				// after schedule reset frame, the changed schedule is applied during a frame
				/* if(schedule_reset_flag == TRUE){
					schedule_reset_flag = FALSE;
					schedule_reset_WH(changed_schedule, schedule_len, schedule_char_len);
					printf("schedule_changed: %d\r\n", schedule_reset_flag);
				} */

				/* adaptability parameter */
				/* random_slot_seq += 1;
				if(random_slot_seq == 5){
					// Schedule reset - WirelessHART
					schedule_reset_WH(origin_schedule, schedule_len, schedule_char_len);
					schedule_reset_flag = TRUE;
					printf("schedule_reset: %d\r\n", schedule_reset_flag);

					random_slot_seq = 0;
					random_slot_idx += 1;
					if(random_slot_idx == 10){
						random_slot_idx = 0;
					}
					random_slot = random_slots[random_slot_idx];
				} */




				atomic phyLock = FALSE;
			}
			return;
		}
		/* adaptability check : Task period changes - AH */
		/* if(TOS_NODE_ID == 2 && currentSlot == random_slot){
			Task_character[my_flow_id][TASK_PERIOD] = changed_period[my_flow_id];
			Task_character[my_flow_id][TASK_DEAD] = changed_deadline[my_flow_id];
		} */



		if(tmp_slot >= 1){
			WH_send(tmp_slot);
			/* AdaptiveHART  */
			/* Flow root transmission */
			/* flow_start_slot = tmp_slot - flows_start_slot - my_release_offset;
			if(is_root == TRUE){

				// Reset variables every deadline
				if(slot == flow_deadline[my_flow_id])
					reset_var(my_flow_id);	// reset is_AH_pkt_null, tx_opper

				// generate a new packet
				if(flow_start_slot >= 0 && (flow_start_slot % Task_character[my_flow_id][TASK_PERIOD]) == 0){
					//printf("new packet generates: slot %d | task_period %d | flow_start_slot %d\r\n", currentSlot, Task_character[my_flow_id][TASK_PERIOD], flow_start_slot);
					atomic is_AH_pkt_null = FALSE;
					atomic tx_opper[my_flow_id] = Task_character[my_flow_id][TASK_MAXTX];
					flow_deadline[my_flow_id] = tmp_slot + Task_character[my_flow_id][TASK_DEAD];
					root_seq += 1;
					AH_payload = (adaptivehart_msg_t *)call SubSend.getPayload(&AH_pkt, sizeof(adaptivehart_msg_t));
					set_payload(AH_payload, my_flow_id, HI_TASK, tx_opper[my_flow_id], flow_deadline[my_flow_id], root_seq);
					return;
				}
				//(re)transmission condition
				atomic tmp_tx_opper = tx_opper[my_flow_id];
				if(flow_start_slot >= 0 && tmp_tx_opper > 0 && is_AH_pkt_null == FALSE){
					offset_algorithm(my_flow_id);
				}
			} */
			/* Relay node transmission */
			/*
			else{
				if(!call HIforwardQ.empty() || !call LOforwardQ.empty()){	// whether there is a pakcet to send
					if(!call HIforwardQ.empty()){
						Q_payload = (adaptivehart_msg_t *)call HIforwardQ.head();
					}else if(!call LOforwardQ.empty()){
						Q_payload = (adaptivehart_msg_t *)call LOforwardQ.head();
					}
					tmp_flowid = Q_payload -> flowid;

					// Reset variables every deadline
					if(Q_payload->deadline < tmp_slot){
						reset_var(tmp_flowid);
					}



					// (re)transmission condition
					atomic tmp_tx_opper = tx_opper[tmp_flowid];
					//atomic tmp_is_dest = is_dest[tmp_flowid];
					if(tmp_slot >= rcv_slot[tmp_flowid] + 1 && tmp_tx_opper > 0 && is_dest[tmp_flowid] == FALSE){
						atomic txing_flowid = tmp_flowid;
						AH_payload = (adaptivehart_msg_t *)call SubSend.getPayload(&AH_pkt, sizeof(adaptivehart_msg_t));
						set_payload(AH_payload, Q_payload->flowid, Q_payload->priority, Q_payload->maxtx, Q_payload->deadline, Q_payload->seq);
						offset_algorithm(txing_flowid);
						//send_AH_Pkt(&AH_pkt, my_dest[txing_flowid], txing_flowid);
					}
				}
			} */
		}
 	}



 	async command error_t Send.send(message_t * msg, uint8_t len) {
    atomic {
 			if (toSend == NULL) {
 				toSend = msg;
 				toSendLen = len;
 				return SUCCESS;
 			}
 		}
 		return FAIL;
 	}


	async event void BeaconSend.sendDone(message_t * msg, error_t error) {
	}


	async event void SubSend.sendDone(message_t * msg, error_t error) {
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
	}

 	//provide the send interface
 	async command error_t Send.cancel(message_t *msg) {
  		atomic {
 			if (toSend == NULL) return SUCCESS;
 			atomic toSend = NULL;
 		}
 		return call SubSend.cancel(msg);
 	}

	/**
	 * Receive
	 */
	async event void SubReceive.receive(message_t *msg, void *payload, uint8_t len) {
		am_addr_t src = call AMPacket.source(msg);
		uint8_t rcv_flow_id;
		uint16_t rcv_flow_seq, tmp_rcv_slot;
		adaptivehart_msg_t *rcv_AH_payload = (adaptivehart_msg_t *)payload;
		adaptivehart_msg_t *tmp_Q_payload = (adaptivehart_msg_t *)NULL;
		bool tmp_is_dest;


		rcv_flow_id = rcv_AH_payload -> flowid;
		rcv_flow_seq = rcv_AH_payload->seq;
		/* check duplicate packet receive */
		if(prev_seq[rcv_flow_id] != rcv_flow_seq){
			//printf("<MAC>:Received Pkt flow: %d, at src:%u , slot: %ld\r\n", rcv_flow_id, src, call SlotterControl.getSlot());

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
					tmp_Q_payload = (adaptivehart_msg_t *) call SubSend.getPayload(&HIforwardPktBuffer[HIqueueSize], sizeof(adaptivehart_msg_t));
					set_payload(tmp_Q_payload, rcv_flow_id, rcv_AH_payload->priority, rcv_AH_payload->maxtx, rcv_AH_payload->deadline, rcv_flow_seq);
					atomic tx_opper[rcv_flow_id] 		= rcv_AH_payload -> maxtx;
					atomic rcv_slot[rcv_flow_id] 		= tmp_rcv_slot;
					call HIforwardQ.enqueue(tmp_Q_payload);
				}else if(rcv_flow_id == LO_TASK){
					LOqueueSize = call LOforwardQ.size();
					tmp_Q_payload = (adaptivehart_msg_t *) call SubSend.getPayload(&LOforwardPktBuffer[LOqueueSize], sizeof(adaptivehart_msg_t));
					set_payload(tmp_Q_payload, rcv_flow_id, rcv_AH_payload->priority, rcv_AH_payload->maxtx, rcv_AH_payload->deadline, rcv_flow_seq);
					atomic tx_opper[rcv_flow_id] 		= rcv_AH_payload -> maxtx;
					atomic rcv_slot[rcv_flow_id] 		= tmp_rcv_slot;
					call LOforwardQ.enqueue(tmp_Q_payload);
				}
			}

		}

		signal Receive.receive(msg, payload, len);
	}

	/**
	 * Frame configuration
	 */
  	command void Frame.setSlotLength(uint32_t slotTimeBms) {
		atomic slotSize = slotTimeBms;
		call FrameConfiguration.setSlotLength(slotSize);
 	}
 	command void Frame.setFrameLength(uint8_t numSlots) {
 		atomic bi = numSlots;
		call FrameConfiguration.setFrameLength(bi+1);
 	}
 	command uint32_t Frame.getSlotLength() {
 		return slotSize;
 	}
 	command uint8_t Frame.getFrameLength() {
 		return bi+1;
 	}


	/**
	 * MISC functions
	 */
	async command void *Send.getPayload(message_t * msg, uint8_t len) {
		return call SubSend.getPayload(msg, len);
	}

	async command uint8_t Send.maxPayloadLength() {
		return call SubSend.maxPayloadLength();
	}

	//provide the receive interface
	command void Receive.updateBuffer(message_t * msg) { return call SubReceive.updateBuffer(msg); }

	default async event uint16_t CcaControl.getInitialBackoff[am_id_t id](message_t * msg, uint16_t defaultbackoff) {
		return 0;
	}

	default async event uint16_t CcaControl.getCongestionBackoff[am_id_t id](message_t * msg, uint16_t defaultBackoff) {
		return 0;
	}

	default async event bool CcaControl.getCca[am_id_t id](message_t * msg, bool defaultCca) {
		return FALSE;
	}

	/* after txoffset interval, it sends a prepared packet */
	// if it does not receive any packet during TxOffset, then do transmission
	async event void TxOffset_alarm.fired(){
		if(is_root == FALSE){
			if(rcv_indicator == FALSE){
				send_AH_Pkt(&AH_pkt, my_dest[txing_flowid], txing_flowid);
			}
		}else{
			send_AH_Pkt(&AH_pkt, my_dest[my_flow_id], my_flow_id);
		}
		/* if(rcv_indicator == FALSE){
			if(is_root == FALSE){
				send_AH_Pkt(&AH_pkt, my_dest[txing_flowid], txing_flowid);
			}else{
				send_AH_Pkt(&AH_pkt, my_dest[my_flow_id], my_flow_id);
			}
		} */

	}

	/* PHY layer packet receive indicator */
	event void ReceiveIndicator.anypktreceive(){
		if(rcv_indicator == FALSE && my_txoffset[LO_TASK] >= 5){
			atomic rcv_indicator = TRUE;
		}
	}

	/* Functions */
	void send_AH_Pkt(message_t *packet, am_addr_t dest, uint8_t flowid){
		bool tmp_phyLock;

		atomic tmp_phyLock = phyLock;
		if (tmp_phyLock == FALSE) {
			call ACK.requestAck(packet);
			call AMPacket.setSource(packet, TOS_NODE_ID);
			call AMPacket.setDestination(packet, dest);
			if(call SubSend.send(packet, sizeof(adaptivehart_msg_t)) == SUCCESS){
				atomic phyLock = TRUE;
				atomic tx_opper[flowid] = tx_opper[flowid] - 1;
				/* for debuging */
				if(is_root == TRUE && tx_flow_seq <= FLOW_SEQ){
					tx_slots[tx_flow_seq] = call SlotterControl.getSlot();
					tx_flow_seq += 1;
				}else if(is_root == FALSE && tx_flow_seq <= FLOW_SEQ){
					//tx_slots[tx_flow_seq] = call SlotterControl.getSlot();
					//tx_flow_seq += 1;
				}

				/* if(is_root == TRUE){
					printf("Tx: at %d and tx_opper:%d, flow:%d\r\n", currentSlot, tx_opper[flowid], txing_flowid);
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
				//call TxOffset_root.startOneShot(my_txoffset[LO_TASK]);
				call TxOffset_alarm.start(32*my_txoffset[LO_TASK]);
			}else{
				send_AH_Pkt(&AH_pkt, my_dest[tmp_txing_flowid], tmp_txing_flowid);
			}
		}else{	// relay nodes
			atomic rcv_indicator = FALSE;
			if(tmp_txing_flowid == LO_TASK){
				//call TxOffset_relay.startOneShot(my_txoffset[LO_TASK]);
				call TxOffset_alarm.start(32*my_txoffset[LO_TASK]);
			}else{
				send_AH_Pkt(&AH_pkt, my_dest[tmp_txing_flowid], tmp_txing_flowid);
			}
		}
	}

	/* debug variables reset */
	void reset_dbg_var(){
		tx_slots[0]=0; tx_slots[1]=0; tx_slots[2]=0; tx_slots[3]=0; tx_slots[4]=0; tx_slots[5]=0; tx_slots[6]=0; tx_slots[7]=0; tx_slots[8]=0; tx_slots[9]=0;
		tx_flow_seq=0;

		rcv_slots[HI_TASK][0]=0; rcv_slots[HI_TASK][1]=0; rcv_slots[HI_TASK][2]=0; rcv_slots[HI_TASK][3]=0; rcv_slots[HI_TASK][4]=0;
		rcv_slots[LO_TASK][0]=0; rcv_slots[LO_TASK][1]=0; rcv_slots[LO_TASK][2]=0; rcv_slots[LO_TASK][3]=0; rcv_slots[LO_TASK][4]=0;

		rx_flow_seq[HI_TASK] = 0;
		rx_flow_seq[LO_TASK] = 0;
	}


	void WH_send(uint8_t slot){
		adaptivehart_msg_t *Q_payload = (adaptivehart_msg_t *)NULL;

		for(i=0; i<schedule_len; i++){
			if(schedule[i][SLOT] == slot){
				if(schedule[i][SENDER] == TOS_NODE_ID){
					if(is_root == TRUE){
						AH_payload = (adaptivehart_msg_t *)call SubSend.getPayload(&AH_pkt, sizeof(adaptivehart_msg_t));
						set_payload(AH_payload, schedule[i][FLOWID], schedule[i][FLOWID], 0, 0, giga_seq);
						send_AH_Pkt(&AH_pkt, schedule[i][RECEIVER], schedule[i][FLOWID]);
					}else{
						if(schedule[i][FLOWID] == HI_TASK){
							if(!call HIforwardQ.empty()){
								Q_payload = (adaptivehart_msg_t *)call HIforwardQ.head();
							}else{
								return;
							}
						}else if(schedule[i][FLOWID] == LO_TASK){
							if(!call LOforwardQ.empty()){
								Q_payload = (adaptivehart_msg_t *)call LOforwardQ.head();
							}else{
								return;
							}
						}
						AH_payload = (adaptivehart_msg_t *)call SubSend.getPayload(&AH_pkt, sizeof(adaptivehart_msg_t));
						set_payload(AH_payload, Q_payload->flowid, Q_payload->priority, Q_payload->maxtx, Q_payload->deadline, Q_payload->seq);
						send_AH_Pkt(&AH_pkt, schedule[i][RECEIVER], schedule[i][FLOWID]);
					}
				}//end sender check
			}//end slot check
		}
	}

	void schedule_reset_WH(uint8_t target_schedule[][6], uint8_t col, uint8_t row){
		uint8_t i;
		uint8_t j;
		for(i=0; i<col; i++){
			for(j=0; j<row; j++){
				schedule[i][j] = target_schedule[i][j];
			}
		}
	}

}
