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
		interface PacketAcknowledgements;

		interface Boot;
		interface Leds;

    //interface Queue<TdmaMsg_t*> as forwardQueue;
    //interface AMSend;

    interface MySlot;
    interface Routing;

		//interface HplMsp430GeneralIO as Pin;

    //interface GlobalTime<TMilli> as GlobalTime;
		//interface TimeSyncInfo;

    ///

    interface DynamicSchedule;
    interface Useful;
    interface IntegrateSchedule;
    interface Queue<TdmaMsg_t*> as forwardQueue;

	}
}
implementation {
	enum {
		SIMPLE_TDMA_SYNC = 123,
	};

	bool init;
	uint32_t slotSize;
	uint32_t bi, sd, cap;
	uint8_t coordinatorId;
  uint8_t actuatorId1;
  uint8_t actuatorId2;
	message_t *toSend;
	uint8_t toSendLen;
	uint8_t currentSlot;
	bool sync;
	bool requestStop;
	bool finishedSlot;
	uint32_t *alarmTime;
  bool dummyFlag;

  //TdmaMsg_t *TdmaMsg;
  message_t packet;

///am_addr_t == uint16_t
  am_addr_t parentId[MAX_NODES];
  uint8_t queueSize;
  message_t forwardPktBuffer[MAX_CHILDREN];
  //message_t selfDataPkt;

////////////////////////
	uint8_t get_last_hop_status(uint8_t flow_id_t, uint8_t access_type_t, uint8_t hop_count_t);
	void set_current_hop_status(uint32_t slot_t, uint8_t sender, uint8_t receiver);
	void set_send_status(uint32_t slot_at_send_done, uint8_t ack_t);
	void set_send (uint32_t slot_t);
	uint8_t get_flow_id(uint32_t slot_t, uint8_t sender, uint8_t receiver);
  void broadcast_controlmsg(uint8_t slot_t, uint8_t Nodeoffset);
  void Find_flow(uint8_t flowid, TdmaMsg_t* payload);
  void Receive_controlmsg(TdmaMsg_t* payload);
  void Store_in_buffer(TdmaMsg_t* payload, TdmaMsg_t* buffer);
///////////////////////

/**************************************************************************************
*   When you change # of task or Node or topology , find //*** and revise
*     in "IntegrateSchedule.nc", "IntegrateScheduleP", "SchedulerP.nc", "usefulP.nc", "PureTDMASchedulerP"
*         ,"Useful.nc", "RoutingTableP"->MakeParent,
*
*
*
*
*/
//************************************************************************************

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

    //***  "6" == # of total hopcount or efficient large number can be set.
    uint8_t schedule[6][11]={//Source Routing, 16 sensor topology, 2 prime trans, retrans twice, baseline. //***
    	//flow 11, flow sensor
    	};

      //[0]: Taskid,  [1]: Pkt period,  [2]: Pkt Deadline,  [3]: remain hop
      uint8_t Task_charater[2][4] = {   //*** 2==# of task
      {1, 10, 4, 4},                   //***remainhop:4 -> {1,2,0,4,3}
      {2, 10, 8, 2}
      //{3, 10, 9, 4}
      };

  uint8_t NodeNumber = 7;   //***
	uint8_t schedule_len=6;   //***    schedule_len == # of total hop  == superframe length
  uint8_t controlschedule_len = 6;    //*** 5 == NodeNumber + 1(calculate period)
  uint8_t controlschedule_start = 7;    //*** 7 == NodeNumber(schedule_len + 1)
	uint32_t gigaframe_length = 13; //5Hz at most     //***  10 == NodeNumber(schedule_len + 1) + controlschedule_len
  uint8_t TaskNumber = 2;   //***
  uint8_t hopcount_len[3]={0,4,2};  //***hopcount_len["3"]== TaskNumber+1, [1]: task1-hopcount=4, [2]: task2-hopcount=2
  uint8_t flowid_root[2]={1,5};    //***  "2" == TaskNumber
  uint8_t flowid_root_len = 2;     //***2==TaskNumber
  uint8_t total_nodeid[7] = {0,1,2,3,4,5,6};        //*** 7 == NodeNumber

//////////////////////////////////////////////////////////////////////////////////////////////////

	event void Boot.booted()
	{
	}

	command error_t Init.init() {
    uint8_t i;
    uint8_t j;
	  //call Pin.makeOutput();
		currentSlot = 0xff;
		slotSize = 50 * 32;     //10ms
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
		requestStop = FALSE;
		finishedSlot = TRUE;
    actuatorId1 = 3; //***
    actuatorId2 = 6; //***
    dummyFlag = FALSE;
    //dataPacket=NULL;

    for(i=0;i<MAX_NODES;i++)
		{
			//nbReceivedPkts[i] = 0;
			parentId[i] = call Routing.MakeParent((am_addr_t)i,NULL);
		}

    if(TOS_NODE_ID == coordinatorId)      //*****
    {
      call IntegrateSchedule.SetSchedule(schedule, Task_charater,schedule_len, flowid_root ,TaskNumber,hopcount_len);
    }



    /*
    printf("schedule:\n");
    for(i=0; i<schedule_len; i++)
    {
      printf("[%u]: {%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u}\r\n",i, schedule[i][0],schedule[i][1],schedule[i][2],schedule[i][3],schedule[i][4],schedule[i][5],schedule[i][6],schedule[i][7],schedule[i][8],schedule[i][9],schedule[i][10]);
    }
    printfflush();
    */  //^^

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
		//call Leds.led2Off();
	}

 	/****************************
 	 *   Implements the schedule
 	 */
 	async event void Slotter.slot(uint8_t slot) {
 		message_t *tmpToSend;
 		uint8_t tmpToSendLen;
    TdmaMsg_t *forwardDataPacket;
    TdmaMsg_t *dataPacket;
    uint8_t Nodeoffset = 0;    //***  Doesn't need to change
    uint8_t i;
    uint8_t tmp_slot;

    tmp_slot = slot % gigaframe_length;

 		printf("slot,%d\r\n",slot % gigaframe_length);

 		//atomic currentSlot = slot;
    atomic currentSlot = tmp_slot;

 		if (tmp_slot == 0) {
 			//beacon slot
 			if (coordinatorId == TOS_NODE_ID) {
 				call BeaconSend.send(NULL, 0);
 			}else{
        for(i=0; i<queueSize; i++)
          call forwardQueue.dequeue();  //Q clean
      }
 			return;
 		}

 		if (tmp_slot >= sd+1) {
 			//sleep
 			if (slot == sd+1) {
 				call RadioPowerControl.stop();
 				//call Pin.clr();
 				call Leds.led0Off();
				printf("sleep\r\n");
 			}

 			//wakeup
 			if (tmp_slot == bi) {
 				call RadioPowerControl.start();
 				//call Pin.set();
 				call Leds.led0On();
				printf("wakeup\r\n");
 			}
 			return;
 		}

 		if (tmp_slot < cap) {
 			//signal CSMASlotSend.send(slot);
 		} else {
      if((tmp_slot) <= schedule_len + 1)  //DATA MESSAGE PERIOD
      {
        //printf("id:%u current slot: %u\r\n", TOS_NODE_ID, tmp_slot );
        set_send (tmp_slot); //heart beat control

      }else //CONTROL MESSAGE PERIOD
      {
        broadcast_controlmsg(tmp_slot, Nodeoffset);
      }

 		}

 	}



 	async command error_t Send.send(message_t * msg, uint8_t len) {
//////////////
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
  /*
      printf("<MAC>: BeaconSend Done at slot: %u\r\n",currentSlot);
      printf("<MAC>: error=%u %u\r\n",error);
      printfflush();
*/
	}


	async event void SubSend.sendDone(message_t * msg, error_t error) {
		if (msg == toSend) {
      printf("<MAC>:Data Sent: %u \r\n",error);
			if (call AMPacket.type(msg) != SIMPLE_TDMA_SYNC) {
			printf("PureIDMASchedulerP,senddone\r\n");
				signal Send.sendDone(msg, error);
			} else {
				//call Slotter.stop();
				//call SyncAlarm.start(32 * slotSize - PACKET_TIME_32HZ);
			}
			atomic toSend = NULL;
		}
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
    TdmaMsg_t* TdmaMsg;
    TdmaMsg_t* databuffer2;
    TdmaMsg_t* ScheduleMsg;
    TdmaMsg_t* databuffer;
    TdmaMsg_t* queueHead;
    uint8_t flow_id_rcv;
    uint8_t i;
    uint8_t controlflag;

    controlflag = FALSE;

    set_current_hop_status(call SlotterControl.getSlot() % gigaframe_length, src, TOS_NODE_ID);
		flow_id_rcv=get_flow_id(call SlotterControl.getSlot() % gigaframe_length , src, TOS_NODE_ID);

    printf("<MAC>:Received Pkt at src:%u , slot: %u\r\n", src, call SlotterControl.getSlot() % gigaframe_length);

    //Change Schedule according to control Msg
    if( (call SlotterControl.getSlot() % gigaframe_length > controlschedule_start) && (call SlotterControl.getSlot() % gigaframe_length <= ((schedule_len + 1) + controlschedule_len)) &&(TOS_NODE_ID != coordinatorId))
    {   //CONTROL PERIOD
      ScheduleMsg = payload;
      databuffer=(TdmaMsg_t*)NULL;
      queueSize= call forwardQueue.size();

      databuffer=call SubSend.getPayload(&forwardPktBuffer[queueSize],sizeof(TdmaMsg_t));


      if((call AMPacket.destination(msg) == AM_BROADCAST_ADDR) && (parentId[src]==TOS_NODE_ID) && (src != coordinatorId) && (TOS_NODE_ID != coordinatorId))
      {   //receive broadcast controlmsg from relay nodes except for coordinatorId!
        printf("Received ControlMsg from other node:%u at %u\r\n",src, call SlotterControl.getSlot() % gigaframe_length);

        Receive_controlmsg(ScheduleMsg);  //store schedule data in schedule[][]

      }else if((call AMPacket.destination(msg) == AM_BROADCAST_ADDR) && (src == coordinatorId) && (TOS_NODE_ID != coordinatorId))
      { //receive controlmsg from 0
        printf("Received ControlMsg from %u at %u\r\n", src, call SlotterControl.getSlot() % gigaframe_length);

        Store_in_buffer(ScheduleMsg, databuffer);  //store ScheduleMsg in buffer
        Receive_controlmsg(ScheduleMsg);  //change schedule information in schedule[][]

        //printf("src: %u-> {%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u}\r\n",src, ScheduleMsg->Schedule[0],ScheduleMsg->Schedule[1],ScheduleMsg->Schedule[2],ScheduleMsg->Schedule[3],ScheduleMsg->Schedule[4],ScheduleMsg->Schedule[5],ScheduleMsg->Schedule[6],ScheduleMsg->Schedule[7],ScheduleMsg->Schedule[8],ScheduleMsg->Schedule[9],ScheduleMsg->Schedule[10],ScheduleMsg->Schedule[11],ScheduleMsg->Schedule[12],ScheduleMsg->Schedule[13],ScheduleMsg->Schedule[14],ScheduleMsg->Schedule[15],ScheduleMsg->Schedule[16],ScheduleMsg->Schedule[17],ScheduleMsg->Schedule[18],ScheduleMsg->Schedule[19]);

        //store in Q
        call forwardQueue.enqueue(databuffer);
        queueSize = (uint8_t) call forwardQueue.size();
      }//end controlmsg from 0

    }else if((call SlotterControl.getSlot() % gigaframe_length < controlschedule_start))      //DATA PERIOD
    {     //store rcved data to Q during data period
      TdmaMsg= payload;
      databuffer2=(TdmaMsg_t*)NULL;
      queueSize= call forwardQueue.size();


      if(TOS_NODE_ID != coordinatorId)
      {
        databuffer2=call SubSend.getPayload(&forwardPktBuffer[queueSize],sizeof(TdmaMsg_t));

        databuffer2->Data = TdmaMsg->Data;    //*****
        call forwardQueue.enqueue(databuffer2);
        queueSize = (uint8_t) call forwardQueue.size();
      }

      //gateway node and actuator node print rcved pkt info
      if(TOS_NODE_ID==coordinatorId){
  			printf("FLOW:%u RECEIVED: %u->%u, SLOT:%u\r\n", flow_id_rcv, src, TOS_NODE_ID, call SlotterControl.getSlot() % gigaframe_length);
        printf("Received Data:%u\r\n",TdmaMsg->Data);
  		}else if(TOS_NODE_ID == actuatorId1 || TOS_NODE_ID == actuatorId2)    //*** when you change actuator node, consider here
      {
        printf("FLOW:%u RECEIVED: %u->%u, SLOT:%u\r\n", flow_id_rcv, src, TOS_NODE_ID, call SlotterControl.getSlot() % gigaframe_length);
        printf("Received Data:%u\r\n",TdmaMsg->Data);
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

//////////////////////////////////////////////////
  //check previous transmission status true or not
  uint8_t get_last_hop_status(uint8_t flow_id_t, uint8_t access_type_t, uint8_t hop_count_t){
    uint8_t last_hop_status=0;
    uint8_t i;
    for (i=0; i<schedule_len; i++){
      if(schedule[i][0] <= call SlotterControl.getSlot() % gigaframe_length){
        if (schedule[i][6]==flow_id_t){   //check flow ID
        if(schedule[i][10] == (hop_count_t-1)){   //check the previous hop-count
          if(schedule[i][9]==1){
            last_hop_status = schedule[i][9];
            //printf("Sensor:%u, GETTING RECEIVE, Slot:%u, %u, %u, %u, %u, %u , %u, %u, %u, %u, %u.\n", TOS_NODE_ID, schedule[i][0], schedule[i][1], schedule[i][2], schedule[i][3], schedule[i][4], schedule[i][5], schedule[i][6], schedule[i][7], schedule[i][8], schedule[i][9], schedule[i][10]);
          }
        }
        }
      }
  }
  return last_hop_status;
  }//end of get_last_hop_status

////////////////////////////////////////////////////////////////////////////////////////
  //change schedule[][9] info. If [9] is not true, then next transmission is not occurred.
  void set_current_hop_status(uint32_t slot_t, uint8_t sender, uint8_t receiver){
    uint8_t i;
    for (i=0; i<schedule_len; i++){
      if(schedule[i][0]==slot_t){// check send-receive pairs 1 slot before/after current slot
        if(schedule[i][1] == sender){//check sender
        if(schedule[i][2] == receiver){//check receiver
          schedule[i][9]=1;
          //printf("Sensor:%u, SETTING RECEIVE, Slot:%u, %u, %u, %u, %u, %u , %u, %u, %u, [9]%u, %u.\n", TOS_NODE_ID, schedule[i][0], schedule[i][1], schedule[i][2], schedule[i][3], schedule[i][4], schedule[i][5], schedule[i][6], schedule[i][7], schedule[i][8], schedule[i][9], schedule[i][10]);
        }
      }
      }
  }

  }// end of set_current_hop_status

///////////////////////////////////////////////////////////////////////////////////////
  uint8_t get_flow_id(uint32_t slot_t, uint8_t sender, uint8_t receiver){
    uint8_t i;
    uint8_t flow_id_t=0;
    for (i=0; i<schedule_len; i++){
      if(schedule[i][0]==slot_t){// check send-receive pairs 1 slot before/after current slot
        if(schedule[i][1] == sender){//check sender
        if(schedule[i][2] == receiver){//check receiver
          flow_id_t=schedule[i][6];
        }
      }
      }
  }
  return flow_id_t;
  } // end of get_flow_id

////////////////////////////////////////////////////////////////////////////////////
void set_send_status(uint32_t slot_at_send_done, uint8_t ack_at_send_done){
    uint8_t k, i;
    uint8_t flow_id_at_send_done;
    uint8_t root_id_at_send_done;
    uint8_t access_type_at_send_done;

  for (k=0; k<schedule_len; k++){
    if(schedule[k][0] == slot_at_send_done && schedule[k][1] ==TOS_NODE_ID){
      flow_id_at_send_done=schedule[k][6];
      root_id_at_send_done=schedule[k][7];
      access_type_at_send_done=schedule[k][4]; // get the right line of the schedule
    }
  }

  //printf("SENSOR:%u, Slot:%u, i:%u\n", TOS_NODE_ID, slot_at_send_done, i);
  if(access_type_at_send_done == 0 || access_type_at_send_done == 2){ // if this is a dedicated slot
  //if(access_type_at_send_done == 0){ // if this is a dedicated slot
    if (ack_at_send_done==1){
      for (i=0; i<schedule_len; i++){
        if (schedule[i][6]==flow_id_at_send_done){ //check flow id
          if(schedule[i][7] == root_id_at_send_done){//check root
            // if (TOS_NODE_ID == 10 || TOS_NODE_ID == 69 || TOS_NODE_ID == 14 || TOS_NODE_ID == 73){
            // 	printf("$$$SENSOR: %u has successfully sent a packet in SLOT:%u.\n", TOS_NODE_ID, slot_at_send_done);
            // }
            schedule[i][8]=1;
            //printf("***DEDICATED: SENSOR: %u, KILLING POTENTIAL SEND: Slot:%u, %u, %u, %u, %u, %u , %u, %u, %u.\n", TOS_NODE_ID, schedule[i][0], schedule[i][1], schedule[i][2], schedule[i][3], schedule[i][4], schedule[i][5], schedule[i][6], schedule[i][7], schedule[i][8]);
          }
        }
      }
    }
    else{
    }
  }else if(access_type_at_send_done==1){//if this is a shared slot
    //printf("SHARED: SENSOR: %u, DISABLING: Slot:%u, %u, %u, %u, %u, %u , %u, %u, %u.\n", TOS_NODE_ID, schedule[i][0], schedule[i][1], schedule[i][2], schedule[i][3], schedule[i][4], schedule[i][5], schedule[i][6], schedule[i][7], schedule[i][8]);
    if (ack_at_send_done==1){
      //printf("SHARED111: SENSOR: %u, KILLING POTENTIAL SEND: Slot:%u, %u, %u, %u, %u, %u , %u, %u, %u.\n", TOS_NODE_ID, schedule[i][0], schedule[i][1], schedule[i][2], schedule[i][3], schedule[i][4], schedule[i][5], schedule[i][6], schedule[i][7], schedule[i][8]);
    }
    else{
      for (i=0; i<schedule_len; i++){
        if (schedule[i][6]==flow_id_at_send_done){ //check flow id
            schedule[i][8]=1;
        }
      }
      //printf("SHARED222: SENSOR: %u, KILLING POTENTIAL SEND: Slot:%u, %u, %u, %u, %u, %u , %u, %u, %u.\n", TOS_NODE_ID, schedule[i][0], schedule[i][1], schedule[i][2], schedule[i][3], schedule[i][4], schedule[i][5], schedule[i][6], schedule[i][7], schedule[i][8]);
    }
  }
 }// end of set_send_status

//////////////////////////////////////////////////////////////////////////////////////
  //transmit data at their slot
  void set_send (uint32_t slot_t){
  uint8_t i;
  uint32_t slot_norm = slot_t; //Here slot_norm is the real time slot normalized by superframe length
  // bool idleStatus;
  TdmaMsg_t* tdmamsg;
  TdmaMsg_t* forwardDataMsg;

  tdmamsg = call SubSend.getPayload(&packet,sizeof(TdmaMsg_t));

  for (i=0; i<schedule_len; i++){
      if (slot_norm == schedule[i][0]){//check slot
        if(TOS_NODE_ID == schedule[i][1] || TOS_NODE_ID == schedule[i][2]){//check sender & receiver id
          if(schedule[i][10]>1){ //check if this is on a multi-hop path
            if(TOS_NODE_ID == schedule[i][1] && schedule[i][8]==0){//No. 8 in the schedule is Send status in sendDone
                if (get_last_hop_status(schedule[i][6], schedule[i][4], schedule[i][10])){// if above so, check delivery status of last hop
                  //**call CC2420Config.setChannel(schedule[i][3]);
                  //**call CC2420Config.sync();

                  //Q data transmission
                  forwardDataMsg = call forwardQueue.head();
                  call forwardQueue.dequeue();

                  tdmamsg->Data = forwardDataMsg->Data;       //*****store data

                  call AMPacket.setSource(&packet, TOS_NODE_ID);
                  call AMPacket.setDestination(&packet, schedule[i][2]);
                  //**call PacketAcknowledgements.requestAck(&packet);

                  if(call SubSend.send(&packet, sizeof(TdmaMsg_t)) == SUCCESS)
                  {
                    printf("Multihop Subsend success!\r\n");
                  }else
                  {
                    printf("Multihop Subsend fail\r\n");
                  }

                  //sequence, sender, receiver, access type, slot, channel
                  //printf("Node: %u, Link Failure detection.\n");
                  //printf("%u, %u, %u, %u, %u, %u, %u, %u\n", 0, TOS_NODE_ID, schedule[i][2], schedule[i][4], slot_norm, call CC2420Config.getChannel(), schedule[i][5], schedule[i][6]);

                  // print out multihop send status
                  //printf("SENDER, HOP >1: %u->%u, Flow:%u, AccessType:%u, slot: %u, channel: %u\r\n", TOS_NODE_ID, call AMPacket.destination(&packet), schedule[i][6], schedule[i][4], slot_norm, schedule[i][3]);
                }// end check last hop
              }// end sender check
              if(TOS_NODE_ID == schedule[i][2] && schedule[i][8]==0){
                //printf("RECEIVER, HOP >1: %u, slot: %u, channel: %u, time: %s\n", TOS_NODE_ID, slot_norm, schedule[i][3], sim_time_string());
                //**call CC2420Config.setChannel(schedule[i][3]);
                //**call CC2420Config.sync();
            }//end receiver check
          }else{
            if(TOS_NODE_ID == schedule[i][1] && schedule[i][8]==0){   //root node transmission
              //**call CC2420Config.setChannel(schedule[i][3]);
              //**call CC2420Config.sync();
              //**call PacketAcknowledgements.requestAck(&packet);

              tdmamsg->Data = 777;  //*****store state value

              call AMPacket.setSource(&packet, TOS_NODE_ID);
              call AMPacket.setDestination(&packet, schedule[i][2]);

              if(call SubSend.send(&packet, sizeof(TdmaMsg_t)) == SUCCESS)
              {
                printf("Subsend success!\r\n");
              }else
              {
                printf("Subsend fail\r\n");
              }

            }
            if(TOS_NODE_ID == schedule[i][2] && schedule[i][8]==0){
              //printf("RECEIVER, HOP =1: %u, slot: %u, channel: %u, time: %s\n", TOS_NODE_ID, slot_norm, schedule[i][3], sim_time_string());
                //**call CC2420Config.setChannel(schedule[i][3]);
              //**call CC2420Config.sync();
            }
          }//end else
        }//end slot check
      }//end sender || receiver check
    }//end for
  }//end set_send

///////////////////////////////////////////////////////////////////////////////////
  void broadcast_controlmsg(uint8_t slot_t, uint8_t Nodeoffset)   //gateway reschedule and broadcast schedule info
  {
    uint8_t i;
    TdmaMsg_t* tdmamsg;
    TdmaMsg_t* forwardControlMsg;

    tdmamsg = call SubSend.getPayload(&packet, sizeof(TdmaMsg_t));



    //DURING: controlschedule_start ~ controlschedule_start(schedule_len + 1) + controlschedule_len
      if((slot_t) == controlschedule_start)    //only coordinatorId reschedule EDF
      {
        if(TOS_NODE_ID == coordinatorId)    // + if current haven task period(or deadline) is different with rcved task period(or deadline)
        {                                   // then reschedule  //*****
          printf("controlschedule_start at %u\r\n",controlschedule_start);
          call IntegrateSchedule.SetSchedule(schedule, Task_charater,schedule_len, flowid_root ,TaskNumber,hopcount_len);
        }else
        {//Q Clean at controlschedule_start slot
          for(i=0; i<queueSize; i++)
            call forwardQueue.dequeue();
          if(call forwardQueue.empty())
          {
            printf("Q clean!\r\n");
          }
        }

      }else if(((slot_t) > controlschedule_start) && ((slot_t) <= ((schedule_len + 1) + controlschedule_len)))
      {
        //printf("control Msg broadcast period: %u ~ %u, broadcast node: %u \r\n",controlschedule_start+1,(schedule_len +1) + controlschedule_len, (slot_t) - (controlschedule_start + 1) + (Nodeoffset));
        if((slot_t) - (controlschedule_start + 1) + (Nodeoffset) == coordinatorId && (TOS_NODE_ID == coordinatorId))  //check my controlmsg broadcast slot
        {   //coordinatorId only broadcast controlmsg


            //store schedule information in payload
            //We can store 5 schedule info in a payload, so devide each flow and store it.
            //broadcast_controlmsg() function is operated once per gigaframe, gateway transmit one flow schedule info in a gigaframe, and in next gigaframe, transmit another flow info.
            if(dummyFlag == FALSE)    //*** if # of flow increase, then Find_flow(flowid, payload) should be added according to below form.
            {
              Find_flow(1,tdmamsg); //***** find flow 1
                                  // store schedule data in payload
              dummyFlag = TRUE;
            }else{
              Find_flow(2,tdmamsg); //***** store flow 2 schedule in payload
              dummyFlag = FALSE;
            }

            call AMPacket.setSource(&packet, TOS_NODE_ID);
            call AMPacket.setDestination(&packet, AM_BROADCAST_ADDR);
            if(call SubSend.send(&packet, sizeof(TdmaMsg_t)) == SUCCESS)
            {
              printf("node:%u, broadcast success at slot %u!\r\n", TOS_NODE_ID, call SlotterControl.getSlot() % gigaframe_length);
            }else
            {
              printf("node:%u, broadcast fail at slot %u\r\n", TOS_NODE_ID, call SlotterControl.getSlot() % gigaframe_length);
            }

        }else if((TOS_NODE_ID != coordinatorId) && (!call forwardQueue.empty()))
        {
          if( ((schedule_len + 2) - ((slot_t) - (controlschedule_start + 1) + (Nodeoffset)) == TOS_NODE_ID))
          {
            //printf("node:%u broadcast controlmsg at slot: %u\r\n",TOS_NODE_ID, slot_t);
            //printfflush();

            //store coordinatorId msg in packet
            forwardControlMsg = call forwardQueue.head();
            call forwardQueue.dequeue();

            printf("slot: %u\r\n",call SlotterControl.getSlot() % gigaframe_length);
            printf("Q data before trans:-> {%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u}\r\n", forwardControlMsg->Schedule[0],forwardControlMsg->Schedule[1],forwardControlMsg->Schedule[2],forwardControlMsg->Schedule[3],forwardControlMsg->Schedule[4],forwardControlMsg->Schedule[5],forwardControlMsg->Schedule[6],forwardControlMsg->Schedule[7],forwardControlMsg->Schedule[8],forwardControlMsg->Schedule[9],forwardControlMsg->Schedule[10],forwardControlMsg->Schedule[11],forwardControlMsg->Schedule[12],forwardControlMsg->Schedule[13],forwardControlMsg->Schedule[14],forwardControlMsg->Schedule[15],forwardControlMsg->Schedule[16],forwardControlMsg->Schedule[17],forwardControlMsg->Schedule[18],forwardControlMsg->Schedule[19]);

            //before trans, store Q data in pkt payload
            Store_in_buffer(forwardControlMsg,tdmamsg);

            call AMPacket.setSource(&packet,TOS_NODE_ID);
            call AMPacket.setDestination(&packet, AM_BROADCAST_ADDR);
            if(call SubSend.send(&packet, sizeof(TdmaMsg_t)) == SUCCESS)
            {
              printf("node:%u, broadcast success at slot %u!\r\n", TOS_NODE_ID, call SlotterControl.getSlot() % gigaframe_length);
            }else
            {
              printf("node:%u, broadcast fail at slot %u\r\n", TOS_NODE_ID, call SlotterControl.getSlot() % gigaframe_length);
            }
          }

        }//end coordinatorId
        //end controlmsg broadcast
      }
  }//end broadcast_controlmsg

  void Find_flow(uint8_t flowid, TdmaMsg_t* payload) //store slot, sender, rcv, hopcount in msg payload according to flowid
  {
    uint8_t i;
    uint8_t OffsetCount;
    uint8_t data_len;
    //TdmaMsg_t tdmamsg;

    data_len = 4;   //*** length of store data in payload (slot, sender, receiver, hopcount)
    OffsetCount = 0;

    for(i=0; i<20; i++)
      payload->Schedule[i] = 0;   //clean payload before store schedule_info

    payload->Flowid = flowid;

    for(i=0; i<schedule_len; i++)
    {
      if(schedule[i][6] == flowid)
      {
        payload->Schedule[0+(data_len*OffsetCount)] = schedule[i][0]; //[0]:slot
        payload->Schedule[1+(data_len*OffsetCount)] = schedule[i][1]; //[1]:sender
        payload->Schedule[2+(data_len*OffsetCount)] = schedule[i][2]; //[2]:receiver
        payload->Schedule[3+(data_len*OffsetCount)] = schedule[i][10]; //[10]:hopcount
        OffsetCount = OffsetCount + 1;
      }
    }
  }

  void Receive_controlmsg(TdmaMsg_t* payload)   //change schedule information in schedule[][]
  {
    uint8_t i;
    uint8_t OffsetCount;
    uint8_t OffsetCount_max;
    uint8_t data_len;
    uint8_t StoreCount;
    uint8_t flowroot;
    bool StoreFlag;

    flowroot = 0;
    data_len = 4;   // length of store data in payload (slot, sender, receiver, hopcount)
    OffsetCount = 0;
    OffsetCount_max = 5; //*** "5":payload->Schedule[][] max length /4
    StoreCount = 0;
    StoreFlag = FALSE;

    for(OffsetCount=0; OffsetCount<OffsetCount_max; OffsetCount++)
    {
      if(payload->Schedule[3+(data_len*OffsetCount)] == 1)
      {
        flowroot = payload->Schedule[1+(data_len*OffsetCount)]; //sender == flow root
      }
      if((payload->Schedule[1+(data_len*OffsetCount)] == TOS_NODE_ID) || (payload->Schedule[2+(data_len*OffsetCount)] == TOS_NODE_ID))
      {   //if payload has sender info that same with my id
        StoreFlag = TRUE;
      }
    }

    if(StoreFlag == TRUE)
    {
      //default setting ([3]Channel,[4]Access type,[5]Flow type,[8]Send status,[9]lasthop status )
      for(OffsetCount=0; OffsetCount<OffsetCount_max; OffsetCount++)   //"20": length of payload->Schedule[]
      {
        schedule[StoreCount][0] = payload->Schedule[0+(data_len*OffsetCount)];    //slot
        schedule[StoreCount][1] = payload->Schedule[1+(data_len*OffsetCount)];    //sender
        schedule[StoreCount][2] = payload->Schedule[2+(data_len*OffsetCount)];    //receiver
        schedule[StoreCount][3] = 22; //channels
        schedule[StoreCount][4] = 0;   //Accesstype : TDMA
        schedule[StoreCount][5] = 1;   //flowtype: regular
        schedule[StoreCount][6] = payload->Flowid;    //flow id
        schedule[StoreCount][7] = flowroot;    //flow root
        schedule[StoreCount][8] = 0;    //send status
        schedule[StoreCount][9] = 0;   //last hop status
        schedule[StoreCount][10] = payload->Schedule[3+(data_len*OffsetCount)];   //hopcount

        StoreCount = StoreCount + 1;
        //OffsetCount = OffsetCount + 1;
      }
    }



    printf("schedule - node:%u \r\n",TOS_NODE_ID);
    for(i=0; i<schedule_len; i++)
    {
      printf("[%u]: {%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u}\r\n",i, schedule[i][0],schedule[i][1],schedule[i][2],schedule[i][3],schedule[i][4],schedule[i][5],schedule[i][6],schedule[i][7],schedule[i][8],schedule[i][9],schedule[i][10]);
    }


  }

  void Store_in_buffer(TdmaMsg_t* payload, TdmaMsg_t* buffer)   //only store {slot,sender,receiver,hopcount} info in buffer
  {
    uint8_t OffsetCount;
    uint8_t OffsetCount_max;
    uint8_t data_len;

    data_len = 4;   // length of store data in payload (slot, sender, receiver, hopcount)
    OffsetCount = 0;
    OffsetCount_max = 5; //*** "5":payload->Schedule[][] max length /4

    buffer->Flowid = payload->Flowid;

    for(OffsetCount=0; OffsetCount<OffsetCount_max; OffsetCount++)   //"20": length of payload->Schedule[]
    {
      buffer->Schedule[0+(data_len*OffsetCount)] = payload->Schedule[0+(data_len*OffsetCount)];    //slot
      buffer->Schedule[1+(data_len*OffsetCount)] = payload->Schedule[1+(data_len*OffsetCount)];    //sender
      buffer->Schedule[2+(data_len*OffsetCount)] = payload->Schedule[2+(data_len*OffsetCount)];    //receiver
      buffer->Schedule[3+(data_len*OffsetCount)] = payload->Schedule[3+(data_len*OffsetCount)];   //hopcount

    }

    //printf("src:-> {%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u}\r\n", buffer->Schedule[0],buffer->Schedule[1],buffer->Schedule[2],buffer->Schedule[3],buffer->Schedule[4],buffer->Schedule[5],buffer->Schedule[6],buffer->Schedule[7],buffer->Schedule[8],buffer->Schedule[9],buffer->Schedule[10],buffer->Schedule[11],buffer->Schedule[12],buffer->Schedule[13],buffer->Schedule[14],buffer->Schedule[15],buffer->Schedule[16],buffer->Schedule[17],buffer->Schedule[18],buffer->Schedule[19]);
  }

}
