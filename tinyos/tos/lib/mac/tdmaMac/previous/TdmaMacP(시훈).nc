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
	uint8_t jms2 = 0;

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

	/** @brief Slot mechanism and superframe */
	bool slotterInit ;		// Slotter initialized after time-synchronized
	uint32_t slotSize;
	uint8_t currentSlot;
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

	/** @briedf scheduling matrix for all nodes
	flownum , currentid, beforeID, nextID, slot
	scheduling_matrix*/


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

		/** @brief Statistics counter and parent declaration **/
		for(j=0;j<MAX_FLOWS;j++)
			for(i=0;i<MAX_NODES;i++)
			{
				nbReceivedPkts[j][i] = 0;
			}

//	flownum , currentid, beforeID, nextID, slot

		if(TOS_NODE_ID == 5)
		{
			data_scheduling_matrix[0][0] = 1; //flow number
			data_scheduling_matrix[0][1] = 5; // 현재 노드번호
			data_scheduling_matrix[0][2] = 1; // 전에
			data_scheduling_matrix[0][3] = 6; // 그다음꺼
			data_scheduling_matrix[0][4] = 3; // 전송 슬롯
		}
		else if (TOS_NODE_ID == 6)
		{
			data_scheduling_matrix[0][0] = 1;
			data_scheduling_matrix[0][1] = 6;
			data_scheduling_matrix[0][2] = 5;
			data_scheduling_matrix[0][3] = 4;
			data_scheduling_matrix[0][4] = 4;
		}
		else if (TOS_NODE_ID == 7)
		{
			data_scheduling_matrix[0][0] = 2;
			data_scheduling_matrix[0][1] = 7;
			data_scheduling_matrix[0][2] = 3;
			data_scheduling_matrix[0][3] = 8;
			data_scheduling_matrix[0][4] = 8;
		}
		else if (TOS_NODE_ID == 8)
		{
			data_scheduling_matrix[0][0] = 2;
			data_scheduling_matrix[0][1] = 8;
			data_scheduling_matrix[0][2] = 7;
			data_scheduling_matrix[0][3] = 2;
			data_scheduling_matrix[0][4] = 9;
		}

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
 		dmamac_data_t *forwardDataPacket;
		uint8_t i;

		atomic currentSlot = slot;


		/** @brief Notification slot for sink */
	   if ((slot == 1 && TOS_NODE_ID  == 0))
	    {
				notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
				notificationPacket->rootId = TOS_NODE_ID;
				notificationPacket->currentSlot = currentSlot;

				//  schedule_idx will be increase in phyNotificationSend.sendDone function

				if(!phyLock)
				{
					if(call phyNotificationSend.send
					(AM_BROADCAST_ADDR, &selfNotificationPkt, sizeof(notification_t)) == SUCCESS)
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

			/** @brief Forward slot for data packet (Manually fed) */
			else if  (data_scheduling_matrix[0][4] ==slot && !call forwardQueue.empty())
			{
				forwardDataPacket = call forwardQueue.head();
			 	dataPacket = (dmamac_data_t*)call Packet.getPayload(&selfDataPkt, sizeof(dmamac_data_t));
		  	dataPacket->nodeId = forwardDataPacket->nodeId;
				dataPacket->data = forwardDataPacket->data;
				dataPacket->flowId = forwardDataPacket->flowId;

				if(!phyLock)
				{
					call ACK.requestAck(&selfDataPkt);

					if(call phyDataSend.send(data_scheduling_matrix[0][3], &selfDataPkt, sizeof(dmamac_data_t)) == SUCCESS)
					{
							atomic phyLock = TRUE;
					}
					else
					{
							printf("MAC: Pkt fail  \n");
					}

					call forwardQueue.dequeue();
				}
			}

			/** @brief Transmission slots for nodes (Manually fed) */
		else if (data_scheduling_matrix[0][2] == 99 && data_scheduling_matrix[0][4] - 1 ==slot)
		{
			sequenceCount++;
			dataPacket = (dmamac_data_t*)call Packet.getPayload(&selfDataPkt, sizeof(dmamac_data_t));
			dataPacket->nodeId = TOS_NODE_ID;
			dataPacket->data = sequenceCount;
			dataPacket->flowId = data_scheduling_matrix[0][0];
		}
		else if (data_scheduling_matrix[0][2] == 99 && data_scheduling_matrix[0][4] == slot)
		{
				call ACK.requestAck(&selfDataPkt);
				if(call phyDataSend.send(data_scheduling_matrix[0][3], &selfDataPkt, sizeof(dmamac_data_t)) == SUCCESS)
							{
								atomic phyLock = TRUE;

							}
							else
							{
				  		printf("MAC: Packet failed  \n");
							}


			flow_idx++;
		}

		/** @brief Define sleep i.e radio off for the rest of the time for power usage test. */
		else if (slot == 10) {
		  call forwardQueue.dequeue();
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

		signal MacSend.sendDone(msg, error);
	}

	event message_t * phyNotificationReceive.receive(message_t *msg, void *payload, uint8_t len){
		/* prepare for forwarding for notification_t*/
		notification_t* notifyPacketTemp =(notification_t *)payload;
		notification_t* notifyBuffer =  (notification_t *)NULL;

		notifyBuffer = (notification_t*)call Packet.getPayload(&forwardNotifyPktBuffer[queueSize], sizeof(notification_t));
		notifyBuffer->rootId = notifyPacketTemp->rootId;
		notifyBuffer->currentSlot = notifyPacketTemp->currentSlot;

		src = (am_addr_t )call AMPacket.source(msg);
		/* If schdeuling is not intended for me, store the received packets in forwarding queue */


	//	value = call TimeSyncInfo.getNumEntries();
	//	call GlobalTime.local2Global(&rxTimestamp);

		if(TOS_NODE_ID != 0 &&  slotterInit == FALSE)
    {
		 //  rxTimestamp = call PacketTimeStamp.timestamp(msg);
			// call GlobalTime.local2Global(&rxTimestamp);
			 call GenericSlotterPowerControl.start();
			 call SlotterControl.synchronize(1);
			 slotterInit = TRUE;
		}
		else
		  call SlotterControl.synchronize(notifyPacketTemp->currentSlot);

		 return msg;
	}


	event void phyDataSend.sendDone(message_t *msg, error_t error){
	//	printf("  <N>: Data sent : %u \n",currentSlot);
		//nbSentPkts++;
		/** @brief ACK testing */
   if(call ACK.wasAcked(msg))
		{	printf("<ACK> received %d \n",jms);}
	 else
	 	printf("<ACK FAIL> %d \n",jms);

		/** @brief Clearing send lock once successfully sent on channel */
		atomic phyLock = FALSE;
		signal MacSend.sendDone(msg, error);
	}



	event message_t * phyDataReceive.receive(message_t *msg, void *payload, uint8_t len){

		dmamac_data_t* dataPacketTemp = (dmamac_data_t *)payload;
	dmamac_data_t* dataBuffer =  (dmamac_data_t *)NULL;
	//	dmamac_data_t* queueHead;	// Used to verify the contents saved in the queue

	//	queueSize = call forwardQueue.size();
		dataBuffer = (dmamac_data_t*)call Packet.getPayload(&forwardPktBuffer[queueSize], sizeof(dmamac_data_t));
		dataBuffer->nodeId = dataPacketTemp->nodeId;
		dataBuffer->data = dataPacketTemp->data;
		dataBuffer->flowId = dataPacketTemp->flowId;

		printf("  recieved curSlot = %d\n",currentSlot);
		


		/** @brief realSrc is the original source, src on other hand could be intermediate hop */
		realSrc = (am_addr_t) dataPacketTemp->nodeId;

		/** @brief Power level and RSSI */
		//outputPower = call linkIndicator.getPower(msg);
		//rssi = (uint16_t) call linkIndicator.getRssi(msg);
		//당장 실험할 때는 필요가없다 누군가 필요하면 사용하도록

		/* If not sink store the received packets in forwarding queue */

			if(dataPacketTemp != NULL)
		 	{
				call forwardQueue.enqueue(dataBuffer);
			}

		if  (data_scheduling_matrix[0][4] == currentSlot)
		{
		 	dataPacket = (dmamac_data_t*)call Packet.getPayload(&selfDataPkt, sizeof(dmamac_data_t));
	  	dataPacket->nodeId = dataBuffer->nodeId;
			dataPacket->data = dataBuffer->data;
			dataPacket->flowId = dataBuffer->flowId;

			if(!phyLock)
			{
					call ACK.requestAck(&selfDataPkt);

						if(call phyDataSend.send(data_scheduling_matrix[0][3], &selfDataPkt, sizeof(dmamac_data_t)) == SUCCESS)
							{
									atomic phyLock = TRUE;
							}
							else
							{
							printf("MAC: Pkt fail  \n");
							}
							call forwardQueue.dequeue();
					}
		}

		jms++;
		/** @brief multihop check (discarding packet not for me) */
		if(realSrc == TOS_NODE_ID)
		{
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
