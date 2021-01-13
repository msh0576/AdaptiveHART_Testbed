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
		interface CC2420Packet as linkIndicator;
		interface AsyncStdControl as GenericSlotterPowerControl;
		interface Slotter;
		interface SlotterControl;
		interface FrameConfiguration;
		interface Queue<dmamac_data_t*> as forwardQueue;
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
		interface GlobalTime<T32khz> as GlobalTime;
		interface TimeSyncInfo;
		interface PacketTimeStamp<T32khz,uint32_t>;
	  //interface TimeSyncPacket<T32khz,uint32_t>;
	}

}

implementation {

	/** @brief Radio operation variables*/
	bool init;
	bool requestStop;
	bool radioOff;
	bool phyLock; 			// Radio lock to prevent double send

	/** @brief Time sync info */
	uint32_t rxTimestamp ;
	uint32_t myLocalTime,myGlobalTime,myOffset,mySkew;
	uint32_t gap;
	uint8_t jms = 0;

	uint8_t value;
	uint8_t inSync; 		// Is the node synchronized on time with sink

	/** @brief Message to transmit From TestAcks */
	message_t forwardPktBuffer[MAX_CHILDREN];
	message_t forwardNotifyPktBuffer[MAX_CHILDREN];
	message_t selfDataPkt;
	message_t selfNotificationPkt;
	message_t selfSyncPkt;

	/** @brief Payload parts for packets */
	notification_t* notificationPacket;
	dmamac_data_t* dataPacket;
	sync_t* syncPacket;

	/** @brief Slot mechanism and superframe */
	bool slotterInit ;		// Slotter initialized after time-synchronized
	uint32_t slotSize;
	uint8_t currentSlot;
	uint8_t superFrameLength,currFrameSize;

	uint16_t sequenceCount;
	uint8_t queueSize;
	uint16_t nbReceivedPkts[MAX_FLOWS][MAX_NODES];
	uint16_t nbSentPkts;

	am_addr_t parentId[MAX_NODES];
	am_addr_t src;			// Sender source
	am_addr_t realSrc;		// Base source (origin)
	uint8_t i;
	uint8_t outputPower;	// TX power
	int16_t rssi;			// Received signal strength

	/** @briedf scheduling matrix for all nodes
	flownum , currentid, beforeID, nextID, slot
	scheduling_matrix*/

	uint8_t scheduling_matrix[100][5]={ // 이건 사실은 0번 노드만 가지고 있는 되는
	//{1, 0,  99, 1, 10},
	//{1, 1,  0, 2, 11},
	{2, 2, 99, 1, 13},
	{2, 1,  2, 0, 14},
	};
	uint8_t scheduling_num = 2; //  스케쥴링 개수(꼭 바꿔 써줘야함 )

	uint8_t flow_idx = 0;
	uint8_t data_scheduling_matrix[2][5]; //각 노드가 가지고 있는 스케쥴링 정보


	//나중에 flow 수가 늘어나면 첫번째 index를 수정해야함

	uint8_t broadcast_route[MAX_NODES][MAX_NODES] = {
	{0, 11, 11, 11, 11},
	{10, 0, 12, 12, 12},
	{11, 11, 0, 13, 13},
	{12, 12, 12, 0, 14},
	{13, 13, 13, 13, 0}
	};

	uint16_t nodeLv[MAX_NODES] = {0,1,2,3,4};		// level per the number of nodes , describe level in []
	uint16_t broadcastSlotLength;
	uint8_t notifySlot;
	uint8_t notifyForwardSlot;
	uint8_t schedule_idx;


	dmamac_data_t *forwardDataPacket;

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

		/** @brief Boolean */
		init = FALSE;
		requestStop = FALSE;
		radioOff = TRUE;
		phyLock = FALSE;
		slotterInit = FALSE;

		superFrameLength = 100;
		currFrameSize = 100; //Same as above used by Frame Configuration interface

		return SUCCESS;
	}

	/** @brief
	 * @source From PureTDMASchedulerP (UPMA)
	 * Interface to startup the MAC module
	 *  */
 	command error_t MacPowerControl.start() {

 		error_t err;
		int i,j;
		broadcastSlotLength = 0;
		notifySlot = 1; //4 이후부터 스케쥴링정보보냄
		notifyForwardSlot = -1;
		schedule_idx = 0;

		/** @brief Statistics counter and parent declaration **/
		for(j=0;j<MAX_FLOWS;j++)
			for(i=0;i<MAX_NODES;i++)
			{
				nbReceivedPkts[j][i] = 0;
			}

		for (i=0 ;i<scheduling_num ;i++ )
				broadcastSlotLength += nodeLv[scheduling_matrix[i][1]];


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
				if (TOS_NODE_ID % 10 == 0)
	 			{
					call GenericSlotterPowerControl.start();
		 			call SlotterControl.synchronize(1);
		 			call FrameConfiguration.setFrameLength(superFrameLength);
	 			}
				init = TRUE;
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

		notification_t *forwardNotifyPacket;
		atomic currentSlot = slot;

		//printf("S: %u, SFL %d \n",currentSlot,call FrameConfiguration.getFrameLength());
			if(slot == 0)
			{

			//	printf("Pkt rcvd flow1 0: %d, 1: %d, 2:%d, 3:%d, 4:%d \n",nbReceivedPkts[0][0],nbReceivedPkts[0][1],
		//		nbReceivedPkts[0][2],nbReceivedPkts[0][3],nbReceivedPkts[0][4]);

		//		printf("Pkt rcvd flow2 0: %d, 1: %d, 2:%d, 3:%d, 4:%d \n",nbReceivedPkts[1][0],nbReceivedPkts[1][1],
		//		nbReceivedPkts[1][2],nbReceivedPkts[1][3],nbReceivedPkts[1][4]);
			}

		while(data_scheduling_matrix[flow_idx][0] == 0 && flow_idx <= 100) flow_idx++;

		/** @brief Notification slot for sink */
	   if ((slot <= scheduling_num && TOS_NODE_ID % 10 == 0))
	    {
				if(scheduling_matrix[schedule_idx][1] == 0) // 스케쥴링하고자 하는 노드의 번호가 0번인 경우
				{
					data_scheduling_matrix[scheduling_matrix[schedule_idx][0]-1][0] = scheduling_matrix[schedule_idx][0];  // scheduling_matrix[schedule_idx][0] : FlowNum
					data_scheduling_matrix[scheduling_matrix[schedule_idx][0]-1][1] = scheduling_matrix[schedule_idx][1];
					data_scheduling_matrix[scheduling_matrix[schedule_idx][0]-1][2] = scheduling_matrix[schedule_idx][2];
					data_scheduling_matrix[scheduling_matrix[schedule_idx][0]-1][4] = scheduling_matrix[schedule_idx][4];
					data_scheduling_matrix[scheduling_matrix[schedule_idx][0]-1][3] = scheduling_matrix[schedule_idx][3];
					schedule_idx++;
				}
				notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
				notificationPacket->rootId = TOS_NODE_ID;
				notificationPacket->currentSlot = currentSlot;
				notificationPacket->flowNum = scheduling_matrix[schedule_idx][0];
				notificationPacket->curId = scheduling_matrix[schedule_idx][1];
				notificationPacket->beforeId = scheduling_matrix[schedule_idx][2];
				notificationPacket->nextId = scheduling_matrix[schedule_idx][3];
				notificationPacket->slot = scheduling_matrix[schedule_idx][4];
				//  schedule_idx will be increase in phyNotificationSend.sendDone function

				if(!phyLock)
				{
					if(call phyNotificationSend.send
					(AM_BROADCAST_ADDR, &selfNotificationPkt, sizeof(notification_t)) == SUCCESS)  //broadcast_route[TOS_NODE_ID % 10][notificationPacket->curId]
					{
						atomic phyLock = TRUE;
						//printf("<Mac,notification>: Pkt -> Radio \n");
					}
					else
						printf("<Mac>:  notifPkt <FAIL>  \n");
				}
				else
						printf("<Mac-nofiPkt-Fail>: Radio locked  \n");
	   	}

			/** @brief Forward slot for notification packet (Manually fed) */
		if (slot == notifyForwardSlot	&& (!call forwardNotifyQueue.empty()))
			{
				notifyForwardSlot = -1;
				forwardNotifyPacket = call forwardNotifyQueue.head();
			 	call forwardNotifyQueue.dequeue();

			 	notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
				notificationPacket->rootId = forwardNotifyPacket->rootId;
				notificationPacket->currentSlot = currentSlot;
				notificationPacket->flowNum = forwardNotifyPacket->flowNum;
				notificationPacket->curId = forwardNotifyPacket->curId;
				notificationPacket->beforeId = forwardNotifyPacket->beforeId;
				notificationPacket->nextId = forwardNotifyPacket->nextId;
				notificationPacket->slot = forwardNotifyPacket->slot;

				if (phyLock == FALSE) {
					if(radioOff){
				//		printf("Waking up sleeing radio \n");
						call RadioPowerControl.start();
					}
					call ACK.requestAck(&selfNotificationPkt);
					if(call phyNotificationSend.send
						(broadcast_route[TOS_NODE_ID % 10][notificationPacket->curId], &selfNotificationPkt, sizeof(notification_t)) == SUCCESS)
						{
								atomic phyLock = TRUE;
					//		printf("MAC: Sent pkt to PHY \n");
						}
						else
						{
				  	//		printf("MAC: Pkt fail  \n");
						}
				}
	 			else
	  				printf("MAC: Radio Locked \n");
						flow_idx++;
			}

			/** @brief Transmission slots for nodes (Manually fed) */
			/** @brief Transmission slots for nodes (Manually fed) */
		if (data_scheduling_matrix[flow_idx][2] == 99 && data_scheduling_matrix[flow_idx][4] - 1 ==slot)
		{
			sequenceCount++;
			dataPacket = (dmamac_data_t*)call Packet.getPayload(&selfDataPkt, sizeof(dmamac_data_t));
			dataPacket->nodeId = TOS_NODE_ID;
			dataPacket->sequenceNo = sequenceCount;
			dataPacket->flowId = data_scheduling_matrix[flow_idx][0];
		}
		else if (data_scheduling_matrix[flow_idx][2] == 99 && data_scheduling_matrix[flow_idx][4] == slot)
		{
			if (phyLock == FALSE) {
				if(radioOff){
		//			printf("Waking up sleeing radio \n");
					call RadioPowerControl.start();
				}
				call ACK.requestAck(&selfDataPkt);
				if(call phyDataSend.send(data_scheduling_matrix[flow_idx][3]+10, &selfDataPkt, sizeof(dmamac_data_t)) == SUCCESS)
							{
								atomic phyLock = TRUE;
					//			printf("MAC: Sent packet to PHY \n");
							}
							else
							{
								printf("MAC: Packet failed  \n");
							}
			}
			else
					printf("MAC: Radio Locked \n");
					flow_idx++;
		}

		/** @brief Forward slot for data packet (Manually fed) */

		else if  (data_scheduling_matrix[forwardDataPacket->flowId -1][4] == slot && !call forwardQueue.empty())
		{
		 	dataPacket = (dmamac_data_t*)call Packet.getPayload(&selfDataPkt, sizeof(dmamac_data_t));
			dataPacket->nodeId = forwardDataPacket->nodeId;
			dataPacket->sequenceNo = forwardDataPacket->sequenceNo;
			dataPacket->flowId = forwardDataPacket->flowId;
			call forwardQueue.dequeue();

			if (phyLock == FALSE) {
				if(radioOff){
			//		printf("Waking up sleeing radio \n");
					call RadioPowerControl.start();
				}
				call ACK.requestAck(&selfDataPkt);

				if(call phyDataSend.send(data_scheduling_matrix[dataPacket->flowId -1][3]+10, &selfDataPkt, sizeof(dmamac_data_t)) == SUCCESS)
					{
							atomic phyLock = TRUE;
				//		printf("MAC: Sent pkt to PHY \n");
					}
					else
					{
			  	printf("MAC: Pkt fail  \n");
					}
			}
 			else
  				printf("MAC: Radio Locked \n");

			forwardDataPacket = call forwardQueue.head();
		}

		/** @brief Code for receive slot here if required (especially if radio needs wakeup) */
		if (slot == broadcastSlotLength + 1) {
		 flow_idx=0;
		}

		/** @brief Define sleep i.e radio off for the rest of the time for power usage test. */
		else if (slot == 61) {
			/** @brief Check if radio is already off */
			if(!radioOff)
			{
				requestStop = TRUE;
				//printf("MAC: Calling Radio to stop \n");
				call RadioPowerControl.stop();
			}
		}
		else if (slot == 99) // Last slot of the superframe
		{
			/** @brief Check if radio is already off */
			if(radioOff)
			{
			//	printf("MAC: Radio Start call \n");
				call RadioPowerControl.start();
			}
		}
	}

	/** @brief Interfaces provided by MAC */
	/** @brief Send interface provided by MAC */
	command error_t MacSend.send(message_t * msg, uint8_t len) {
	//	printf("Parent id : %d, Node id %d \n",parentId[TOS_NODE_ID],TOS_NODE_ID);

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

		if(TOS_NODE_ID % 10 ==0){
			notifySlot = notifySlot + nodeLv[scheduling_matrix[schedule_idx][1]];

			// 다음 notifySlot를 정하기 위해서 nodeLv를 이용하여 구한다. 스케쥴링 하고자하는 노드의 번호는 shceduling_matric에서 가지고 올 수 있다.

			schedule_idx++;
			if (schedule_idx == scheduling_num){
				notifySlot =1;
				schedule_idx=0;

			}
		}

			signal MacSend.sendDone(msg, error);
	}

	event message_t * phyNotificationReceive.receive(message_t *msg, void *payload, uint8_t len){
		/* prepare for forwarding for notification_t*/
		notification_t* notifyPacketTemp =(notification_t *)payload;
		notification_t* notifyBuffer =  (notification_t *)NULL;
		notification_t* queueHead;	// Used to verify the contents saved in the queue

		queueSize = call forwardNotifyQueue.size();

		notifyBuffer = (notification_t*)call Packet.getPayload(&forwardNotifyPktBuffer[queueSize], sizeof(notification_t));
		notifyBuffer->rootId = notifyPacketTemp->rootId;
		notifyBuffer->currentSlot = notifyPacketTemp->currentSlot;
		notifyBuffer->flowNum = notifyPacketTemp->flowNum;
		notifyBuffer->curId = notifyPacketTemp->curId;
		notifyBuffer->beforeId = notifyPacketTemp->beforeId;
		notifyBuffer->nextId = notifyPacketTemp->nextId;
		notifyBuffer->slot = notifyPacketTemp->slot;

		/** @brief realSrc is the original source, src on other hand could be intermediate hop */
		realSrc = (am_addr_t) notifyPacketTemp->curId;
		src = (am_addr_t )call AMPacket.source(msg);
		/* If schdeuling is not intended for me, store the received packets in forwarding queue */

		if(TOS_NODE_ID % 10 == realSrc)
		{
	/*		atomic notifyForwardSlot = notifyBuffer->currentSlot + 1;
			if(notifyBuffer != NULL)
			{
				call forwardNotifyQueue.enqueue(notifyBuffer);
				queueSize = (uint8_t) call forwardNotifyQueue.size();
				if(!call forwardNotifyQueue.empty())
				{
					queueHead = call forwardNotifyQueue.head();
				}
			}
		}
		else{*/
			data_scheduling_matrix[notifyBuffer->flowNum-1][0] = notifyBuffer->flowNum;
			data_scheduling_matrix[notifyBuffer->flowNum-1][1] = notifyBuffer->curId;
			data_scheduling_matrix[notifyBuffer->flowNum-1][2] = notifyBuffer->beforeId;
			data_scheduling_matrix[notifyBuffer->flowNum-1][3] = notifyBuffer->nextId;
			data_scheduling_matrix[notifyBuffer->flowNum-1][4] = notifyBuffer->slot;
		}



		if(TOS_NODE_ID % 10 != 0 &&  slotterInit == FALSE)
    {
		 call GenericSlotterPowerControl.start();
		 call SlotterControl.synchronize(notifyPacketTemp->currentSlot);
		 slotterInit = TRUE;
		 }
		else
		{
	  call SlotterControl.synchronize(notifyPacketTemp->currentSlot);
	  }


		 if(realSrc == TOS_NODE_ID)
	 {
		 /** @brief packet sent towards app */
		 return signal MacReceive.receive(msg, payload, len);
	 }
	 else
		 return msg;
	}


	event void phyDataSend.sendDone(message_t *msg, error_t error){
		nbSentPkts++;
		/** @brief ACK testing */
   if(call ACK.wasAcked(msg))
		{	printf("<ACK> received  %d \n", currentSlot);}
	 else
	 printf("<ACK FAIL> \n");

		/** @brief Clearing send lock once successfully sent on channel */
		atomic phyLock = FALSE;
		signal MacSend.sendDone(msg, error);
	}



	event message_t * phyDataReceive.receive(message_t *msg, void *payload, uint8_t len){

		dmamac_data_t* dataPacketTemp = (dmamac_data_t *)payload;
		dmamac_data_t* dataBuffer =  (dmamac_data_t *)NULL;
		dmamac_data_t* queueHead;	// Used to verify the contents saved in the queue

		am_addr_t tempSrc;
		queueSize = call forwardQueue.size();

		dataBuffer = (dmamac_data_t*)call Packet.getPayload(&forwardPktBuffer[queueSize], sizeof(dmamac_data_t));
		dataBuffer->nodeId = dataPacketTemp->nodeId;
		dataBuffer->sequenceNo = dataPacketTemp->sequenceNo;
		dataBuffer->flowId = dataPacketTemp->flowId;

		src = (am_addr_t )call AMPacket.source(msg);
		printf("receive at %d from nd %d seq %d flow %d,\n",currentSlot, (uint16_t)dataPacketTemp->nodeId,(uint16_t)dataPacketTemp->sequenceNo,(uint8_t)dataPacketTemp->flowId);

		/** @brief realSrc is the original source, src on other hand could be intermediate hop */
		realSrc = (am_addr_t) dataPacketTemp->nodeId;

		/** @brief Power level and RSSI */
		outputPower = call linkIndicator.getPower(msg);
		rssi = (uint16_t) call linkIndicator.getRssi(msg);

		/* If not sink store the received packets in forwarding queue */

		if(!(TOS_NODE_ID % 10 == 2 && realSrc % 10 == 0) && !(TOS_NODE_ID % 10 == 0 && realSrc % 10 == 2))
		{
			if(dataBuffer != NULL)
		 	{
				call forwardQueue.enqueue(dataBuffer);
				forwardDataPacket = call forwardQueue.head();
				queueSize = (uint8_t) call forwardQueue.size();

				if(!call forwardQueue.empty())
				{
					queueHead = call forwardQueue.head();
				}
			}
		}

			if(realSrc ==12)
			nbReceivedPkts[0][src%10]++;
			else
			nbReceivedPkts[1][src%10]++;

			/** @brief multihop check (discarding packet not for me) */
			if(broadcast_route[src ][TOS_NODE_ID ] == TOS_NODE_ID)
			{
				/** @brief if multihop packet */
				if(realSrc != src)
			//	{	printf("<N>: Forwarded Pkt %d->%d->%d",realSrc,src,TOS_NODE_ID);}
				{
					tempSrc = realSrc;
		//			printf("<N>: Forwarded Pkt ");
					while(tempSrc != TOS_NODE_ID)
					{
		//				printf("%d -> ",tempSrc);
						tempSrc = broadcast_route[tempSrc ][TOS_NODE_ID ];
					}
		//			printf("%d\n",tempSrc);
				}
				else
				{
		//			printf("<N>: Forwarded Pkt %d->%d\n",src,TOS_NODE_ID);
				}
				/** @brief packet sent towards app */
				return signal MacReceive.receive(msg, payload, len);

			}
			else
				return msg;
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

	/** @brief Use tasks for non pre-empted execution (not used now) */

}
