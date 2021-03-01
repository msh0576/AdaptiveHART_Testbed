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
		interface Alarm<T32khz, uint32_t> as TxOffset_relay;
		interface Alarm<T32khz, uint32_t> as TxOffset_root;
		//interface Timer<TMilli> as TxOffset_relay;
		//interface Timer<TMilli> as TxOffset_root;
		interface ReceiveIndicator;

	}
}
implementation {
	enum {
		SIMPLE_TDMA_SYNC = 123,
		MAC_Q = 1,
		FLOW_SEQ = 20,
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

	uint32_t gigaframe_length = 20; //5Hz at most     //***  10 == NodeNumber(schedule_len + 1) + controlschedule_len

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


//////////////////////////////////////////////////////////////////////////////////////////////////

	event void Boot.booted()
	{
	}

	command error_t Init.init() {
		uint8_t j;
		currentSlot = 0xff;
		slotSize = 10 * 32;     //10ms
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

		/* adaptability parameter init */
		random_slot = random_slots[random_slot_idx];

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
	uint32_t tmp_recog = 0;
 	async event void Slotter.slot(uint8_t slot) {
    uint8_t tmp_slot;
		int16_t flow_start_slot = -1;
		adaptivehart_msg_t *Q_payload = (adaptivehart_msg_t *)NULL;
		uint8_t tmp_tx_opper;
		uint8_t tmp_flowid = 0;

    tmp_slot = slot;
		timer1 = call TxOffset_relay.getNow();

 		//printf("slot,%d\r\n",tmp_slot);
    atomic currentSlot = tmp_slot;

		if(tmp_slot == 5){
			printf("rcv recog:%ld\r\n", tmp_recog);
		}

 		if (tmp_slot == 0) {
 			//beacon slot
 			if (coordinatorId == TOS_NODE_ID) {
 				call BeaconSend.send(NULL, 0);
 			}
 			return;
 		}

		if(tmp_slot == 1 && TOS_NODE_ID == 1){
			call ACK.requestAck(&AH_pkt);
			call AMPacket.setSource(&AH_pkt, TOS_NODE_ID);
			call AMPacket.setDestination(&AH_pkt, 3);
			if(call SubSend.send(&AH_pkt, sizeof(adaptivehart_msg_t)) == SUCCESS){
				atomic phyLock = TRUE;
			}
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

		/* if(TOS_NODE_ID == 3){
			timer2 = call TxOffset_relay.getNow();
			printf("time diff:%ld at %d\r\n", timer2-timer1, currentSlot);
		} */
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
	async event void TxOffset_relay.fired(){
	}

	async event void TxOffset_root.fired(){
	}

	/* PHY layer packet receive indicator */

	event void ReceiveIndicator.anypktreceive(){
		if(TOS_NODE_ID == 3){
			timer2 = call TxOffset_relay.getNow();
			tmp_recog = timer2 - timer1;
		}

	}


}
