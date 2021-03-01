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

#include "TdmaMac.h"
#include "Timer.h"
#include "printf.h"

module WHMacP {

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
		interface PacketAcknowledgements as ACK;
    interface Leds;
	}

	uses { //Radio Control
		interface SplitControl as RadioPowerControl;
		interface AMSend as phyNotificationSend;	//It seems like broadcast Send
		interface Receive as phyNotificationReceive;
		interface AMSend as phyDataSend;		//It seems like data Send
		interface Receive as phyDataReceive;
	}

	uses {//Time sync stuff from Ftps library
		interface GlobalTime<TMilli> as GlobalTime;
		interface TimeSyncInfo;
	}
}

implementation {

	/** @brief Radio operation variables*/
	bool init;
	bool requestStop;
	bool radioOff;
	bool phyLock; 			// Radio lock to prevent double send

	/** @brief Time sync info */
	uint32_t myLocalTime,myGlobalTime,myOffset,mySkew;
	uint8_t value;
	uint8_t inSync; 		// Is the node synchronized on time with sink

	/** @brief Message to transmit From TestAcks */
	message_t forwardPktBuffer[MAX_CHILDREN];
	message_t selfDataPkt;
	message_t selfNotificationPkt;

	/** @brief Payload parts for packets */
	notification_t* notificationPacket;
	dmamac_data_t* dataPacket;

	/** @brief Slot mechanism and superframe */
	bool slotterInit;		// Slotter initialized after time-synchronized
	uint32_t slotSize;
	uint8_t currentSlot;
	uint8_t superFrameLength,currFrameSize;

	uint16_t sequenceCount;
	uint8_t queueSize;
	uint16_t nbReceivedPkts[MAX_NODES];

	am_addr_t parentId[MAX_NODES];
	am_addr_t src;			// Sender source
	am_addr_t realSrc;		// Base source (origin)
	uint8_t i;
	uint8_t outputPower;	// TX power
	int16_t rssi;			// Received signal strength

//////////////
        uint8_t phyLockCount=0;

	/** @brief Boot Events
	 * @source TestAcks in cc2420 tests
	 */
	event void Boot.booted() {
		//printf_init();
		call RadioPowerControl.start();
	}

	/** @brief
	 * @source From PureTDMASchedulerP (UPMA)
	 * When the module is initialized
	 */
	command error_t Init.init() {

		/** @brief Initialization of necessary variables */
		currentSlot = 0xff;
		slotSize = 10 * 32; // 10ms standard -> 500ms
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

		/** @brief Statistics counter and parent declaration **/
		for(i=0;i<MAX_NODES;i++)
		{
			nbReceivedPkts[i] = 0;
			parentId[i] = 0;
		}

		return SUCCESS;
	}

	/** @brief
	 * @source From PureTDMASchedulerP (UPMA)
	 * Interface to startup the MAC module
	 *  */
 	command error_t MacPowerControl.start() {

 		error_t err;
 		if (init == FALSE) {
	 		/** Configuring frame @aaks */
 		}
 		err = call RadioPowerControl.start();
		printf("<Mac>: We are up and running \n\r");
		printfflush();
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
/////
    //phyLock=FALSE;
 		radioOff = FALSE;
 		printf("<PHY>: Radio startup done");
 		/** @brief Rest is 0 thus for 3 we define 1 */
		if (TOS_NODE_ID == 3)
			parentId[TOS_NODE_ID] = (am_addr_t) 2;
/////////////////////////////
    if (TOS_NODE_ID == 2)
      parentId[TOS_NODE_ID] = (am_addr_t) 1;
      if (TOS_NODE_ID == 1)
        parentId[TOS_NODE_ID] = (am_addr_t) 0;

 		/** @brief Sink starts right away, rest of the nodes synchronize to the sink */
 		atomic{
	 		if (init == FALSE) {
	 			if (TOS_NODE_ID == 0)
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
			printf("<MAC>: Radio stopped \n\r");
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

		atomic currentSlot = slot;
		printf("S: %u, SFL %d \n\r",currentSlot,call FrameConfiguration.getFrameLength());
		printfflush();

		if(slot == 0)
		{
			if(TOS_NODE_ID == 0)
			{
				printf("<SINK>: Packets received until now \n\r");
				printf("1: %d, 2:%d, 3:%d, 4:%d \n",nbReceivedPkts[1],
				nbReceivedPkts[2],nbReceivedPkts[3],nbReceivedPkts[4]);
				printfflush();

			}
		}

		/** @brief Notification slot for sink */
	    if (slot == 1 && TOS_NODE_ID == 0)
	    {
	    	/** @brief Code to handle any sending or receiving of control info (TimeSync etc) */
	    	/** @brief Precisely sending notification to change MOVE to notification slot */
			notificationPacket = (notification_t*)call Packet.getPayload(&selfNotificationPkt, sizeof(notification_t));
			notificationPacket->rootId = TOS_NODE_ID;
			notificationPacket->currentSlot = currentSlot;
/////////////////////////////////
			printf("What is the current phyLock: %d\r\n",phyLock);
			printfflush();
			if(!phyLock)
			{
				if(call phyNotificationSend.send
				(AM_BROADCAST_ADDR, &selfNotificationPkt, sizeof(notification_t)) == SUCCESS)
				{
					atomic phyLock = TRUE;
					printf("<Mac>: Pkt -> Radio \n\r");
				}
				else
					printf("<Mac>: Pkt <FAIL>  \n\r");
			}
			else
					printf("<Mac-Pkt-Fail>: Radio locked  \n\r");
	   	}
	   	/** @brief Transmission slots for nodes (Manually fed) */
		else if ((TOS_NODE_ID == 0 && slot == 5)  || (TOS_NODE_ID == 3 && (slot == 10)))
		{
			printf("<Mac>: My Transmit Slot : %d \n\r",slot);

			sequenceCount++;

			dataPacket = (dmamac_data_t*)call Packet.getPayload(&selfDataPkt, sizeof(dmamac_data_t));
			dataPacket->nodeId = TOS_NODE_ID;
			dataPacket->sequenceNo = sequenceCount;
			dataPacket->rootId = 0;

			if (phyLock == FALSE) {
				if(radioOff){
					printf("Waking up sleeing radio \n\r");
					call RadioPowerControl.start();
				}
				call ACK.requestAck(&selfDataPkt);
///why node 0 transmit data pkt in 5slot.
				if(TOS_NODE_ID == 0)
				{
					if(call phyDataSend.send(AM_BROADCAST_ADDR, &selfDataPkt, sizeof(dmamac_data_t)) == SUCCESS)
						{
							atomic phyLock = TRUE;
							printf("MAC: Sent packet to PHY \n\r");
						}
						else
							printf("MAC: Packet failed  \n\r");
				}
				else
			    {
					if(call phyDataSend.send(parentId[TOS_NODE_ID], &selfDataPkt, sizeof(dmamac_data_t)) == SUCCESS)
							{
								atomic phyLock = TRUE;
								printf("MAC: Sent packet to PHY \n\r");
							}
							else
								printf("MAC: Packet failed  \n\r");
				}
			}
 			else
			{
/////////////////////////
        phyLockCount++;
				if(phyLockCount==5)
				{
					phyLock=FALSE;
					phyLockCount=0;
				}

///////////////////////
  				printf("MAC: Radio Locked and LockCount:%d\n",phyLockCount);
			}
		}
		/** @brief Forward slot for node 1 (Manually fed) */
		else if (TOS_NODE_ID == 1 && (slot == 20) && (!call forwardQueue.empty()))
		{
			printf("<N>: Forwarding slot : %d \n\r",slot);

			forwardDataPacket = call forwardQueue.head();
			printf("<N> Pointed address %d \r\n",(uint16_t)forwardDataPacket);
		 	call forwardQueue.dequeue();

		 	printf("<N> : Queue size is %d \r\n", call forwardQueue.size());  //.size() is the number of elements in Q


		 	dataPacket = (dmamac_data_t*)call Packet.getPayload(&selfDataPkt, sizeof(dmamac_data_t));
			dataPacket->nodeId = forwardDataPacket->nodeId;
			dataPacket->sequenceNo = forwardDataPacket->sequenceNo;
			dataPacket->rootId = forwardDataPacket->rootId;

			printf("<N>: Source: %u, ", (uint16_t)forwardDataPacket->nodeId);
			printf("Seq No : %u, ", (uint16_t)forwardDataPacket->sequenceNo);
			printf("Root : %d \n\r", (uint8_t)forwardDataPacket->rootId);
			printfflush();

			//toSend != NULL &&

			if (phyLock == FALSE) {
				if(radioOff){
					printf("Waking up sleeing radio \n");
					call RadioPowerControl.start();
				}
				call ACK.requestAck(&selfDataPkt);
////////////
        call Leds.led2Toggle();
				if(call phyDataSend.send(parentId[TOS_NODE_ID], &selfDataPkt, sizeof(dmamac_data_t)) == SUCCESS)
					{
						atomic phyLock = TRUE;
						printf("MAC: Sent pkt to PHY \n\r");
					}
					else
						printf("MAC: Pkt fail  \n\r");
			}
 			else
  				printf("MAC: Radio Locked \n\r");

		}
    else if (TOS_NODE_ID == 2 && (slot == 15) && (!call forwardQueue.empty()))
		{
///////////////
			printf("<N>: Forwarding slot : %d \n\r",slot);

			forwardDataPacket = call forwardQueue.head();
			printf("<N> Pointed address %d \r\n",(uint16_t)forwardDataPacket);
		 	call forwardQueue.dequeue();

		 	printf("<N> : Queue size is %d \r\n", call forwardQueue.size());  //.size() is the number of elements in Q


		 	dataPacket = (dmamac_data_t*)call Packet.getPayload(&selfDataPkt, sizeof(dmamac_data_t));
			dataPacket->nodeId = forwardDataPacket->nodeId;
			dataPacket->sequenceNo = forwardDataPacket->sequenceNo;
			dataPacket->rootId = forwardDataPacket->rootId;

			printf("<N>: Source: %u, ", (uint16_t)forwardDataPacket->nodeId);
			printf("Seq No : %u, ", (uint16_t)forwardDataPacket->sequenceNo);
			printf("Root : %d \n\r", (uint8_t)forwardDataPacket->rootId);
			printfflush();

			//toSend != NULL &&

			if (phyLock == FALSE) {
				if(radioOff){
					printf("Waking up sleeing radio \n");
					call RadioPowerControl.start();
				}
				call ACK.requestAck(&selfDataPkt);
/////////////
        call Leds.led2Toggle();
				if(call phyDataSend.send(parentId[TOS_NODE_ID], &selfDataPkt, sizeof(dmamac_data_t)) == SUCCESS)
					{
						atomic phyLock = TRUE;
						printf("MAC: Sent pkt to PHY \n\r");
					}
					else
						printf("MAC: Pkt fail  \n\r");
			}
 			else
  				printf("MAC: Radio Locked \n\r");
///////////////
		}
		/** @brief Code for receive slot here if required (especially if radio needs wakeup) */
		else if (slot == 27) {
			/** @brief Currently prints Time Sync update to check status */

			myGlobalTime = call GlobalTime.getLocalTime();
			inSync = (uint8_t) call GlobalTime.local2Global(&myGlobalTime);
			myOffset = call TimeSyncInfo.getOffset();
			mySkew = (uint32_t) call TimeSyncInfo.getSkew();
			value = call TimeSyncInfo.getNumEntries();

			printf("Global : %lu, Offset : %lu \n\r",myGlobalTime,myOffset);
			printf("Skew : %lu,",mySkew);
			printf(" Entries : %u,",value);
			printf(" SYNC : %u \n\r,",inSync);
			printfflush();
		}
    /*
		else if (slot == 61) {

			if(!radioOff)
			{
				requestStop = TRUE;
				printf("MAC: Calling Radio to stop \n\r");
				call RadioPowerControl.stop();
			}
		}
		else if (slot == 99) // Last slot of the superframe
		{

			if(radioOff)
			{
				printf("MAC: Radio Start call \n\r");
				call RadioPowerControl.start();
			}
		}*/

	}

	/** @brief Interfaces provided by MAC */
	/**{*/
	/** @brief Send interface provided by MAC */
	command error_t MacSend.send(message_t * msg, uint8_t len) {
		printf("Parent id : %d, Node id %d \n",parentId[TOS_NODE_ID],TOS_NODE_ID);

		/** @brief APP packet is dummy, will be created here and sent by MAC
		 * if proper App is written its payload can be fit into the MAC packet */
		atomic {
				printf("MAC: Prepare APP packet \r\n");
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
    atomic
    {
    printf("SinkMAC: Notification sent : %u \n\r",error);

		/** @brief Clearing send lock once successfully sent on channel */
		 phyLock = FALSE;
    }
		signal MacSend.sendDone(msg, error);
	}

	event message_t * phyNotificationReceive.receive(message_t *msg, void *payload, uint8_t len){

		src = (am_addr_t )call AMPacket.source(msg);

		/** @brief Checking change value received from sink */
		notificationPacket = (notification_t*)call Packet.getPayload(msg, sizeof(notification_t));

		printf("<Notification-Pkt>RootId: %d \n\r", notificationPacket->rootId);
		printf("currentSlot at sink: %d \r\n", notificationPacket->currentSlot);

		/** @brief part used to synchronize rest of the nodes
		 * Makes sure nodes are time synced through FTSP with minimum 8 table entries
		 * */
		/**{*/
		myOffset = call TimeSyncInfo.getOffset();
		value = call TimeSyncInfo.getNumEntries();

    printf("Before starting Sync, Entries: %d\r\n",value);
    printfflush();
		if(TOS_NODE_ID != 0 && value == 8 && slotterInit == FALSE && src == 0)
		{
			call GlobalTime.local2Global(&myLocalTime);
			call GenericSlotterPowerControl.start();
			/** @brief Synchronizing to the send slot of the sink that is 1 currently (MOVE planned) */
			call SlotterControl.synchronize(1);
			slotterInit = TRUE;
		}
		/**}*/
		printf("  \n");
		printf("<MAC>: Pkt received from %d \n\r",src);

		return msg;
	}

	event void phyDataSend.sendDone(message_t *msg, error_t error){
		printf("<N>: Data sent : %u \n\r",error);

		/** @brief ACK testing */
		if(call ACK.wasAcked(msg))
		{	printf("<ACK> received \n\r");}
		else
			printf("<ACK> failed \n\r");

		/** @brief Clearing send lock once successfully sent on channel */
		atomic phyLock = FALSE;

		signal MacSend.sendDone(msg, error);
	}

	event message_t * phyDataReceive.receive(message_t *msg, void *payload, uint8_t len){
		dmamac_data_t* dataPacketTemp = (dmamac_data_t *)payload;
		dmamac_data_t* dataBuffer =  (dmamac_data_t *)NULL;
		dmamac_data_t* queueHead;	// Used to verify the contents saved in the queue

		queueSize = call forwardQueue.size();

		dataBuffer = (dmamac_data_t*)call Packet.getPayload(&forwardPktBuffer[queueSize], sizeof(dmamac_data_t));
		dataBuffer->nodeId = dataPacketTemp->nodeId;
		dataBuffer->sequenceNo = dataPacketTemp->sequenceNo;
		dataBuffer->rootId = dataPacketTemp->rootId;

		src = (am_addr_t )call AMPacket.source(msg);

		printf("<N>: Source: %d,", (uint16_t)dataPacketTemp->nodeId);
		printf("Seq No : %d,", (uint16_t)dataPacketTemp->sequenceNo);
		printf("Root: %d \n\r", (uint8_t)dataPacketTemp->rootId);
        printfflush();
		/** @brief realSrc is the original source, src on other hand could be intermediate hop */
		realSrc = (am_addr_t) dataPacketTemp->nodeId;

		/** @brief Power level and RSSI */
		outputPower = call linkIndicator.getPower(msg);
		rssi = (uint16_t) call linkIndicator.getRssi(msg);
		printf("<N> Power : %u, RSSI : %d dBm \n\r",outputPower,(rssi-RSSI_OFFSET));
        printfflush();
		/* If not sink store the received packets in forwarding queue */
		if(TOS_NODE_ID != 0 && src != 0 && parentId[TOS_NODE_ID] != src)
		{
////////
      call Leds.led1Toggle();
////
      printf("<N>: Received right before node %d \r\n",src);
			printf("<N>: Source: %d,", (uint16_t)dataBuffer->nodeId);
			printf("<N>: Seq No : %d,", (uint16_t)dataBuffer->sequenceNo);
			printf("<N>: Root: %d \n\r", (uint8_t)dataBuffer->rootId);
            printfflush();
			if(dataBuffer != NULL)
		 	{
				call forwardQueue.enqueue(dataBuffer);
				queueSize = (uint8_t) call forwardQueue.size();
				printf("<N>: Pointed: %d, Real: %d \n\r",(uint16_t)dataBuffer,(uint16_t)dataPacketTemp);
				printf("<N>: Queue size at node %d is %d: \n\r",TOS_NODE_ID,queueSize);
				printfflush();

				if(!call forwardQueue.empty())
				{

					queueHead = call forwardQueue.head();
					printf("<N>: Dummy Source: %d,", (uint16_t)queueHead->nodeId);
					printf("<N>: Seq No : %d,", (uint16_t)queueHead->sequenceNo);
					printf("<N>: Root: %d \n\r", (uint8_t)queueHead->rootId);
					printfflush();
				}
			}
		}
/*
    if(TOS_NODE_ID == 0)
      nbReceivedPkts[realSrc]++;
*/
		/** @brief multihop check (discarding packet not for me) */
		if(parentId[src] == TOS_NODE_ID)
		{
			/** @brief if multihop packet */
			if(realSrc != src)
			{	printf("<N>: Forwarded Pkt %d->%d->%d\n\r",realSrc,src,TOS_NODE_ID);}
			else
				printf("<N>: Forwarded Pkt %d->%d\n\r",src,TOS_NODE_ID);

/////////////////
        if(TOS_NODE_ID == 0)
          nbReceivedPkts[realSrc]++;

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
