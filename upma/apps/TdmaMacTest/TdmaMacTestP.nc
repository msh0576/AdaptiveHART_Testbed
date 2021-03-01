//**************************************************************************
// * file:        TdmaMacTest Application Implementation file
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



module TdmaMacTestP{

  uses { //General
	interface Boot;
	interface Leds;
	interface Packet;
	interface SplitControl as MacControl;
	interface PacketAcknowledgements;
  }

  uses { //Radio Control
	interface Send as MacSend;
	interface Receive as MacReceive;
	interface FrameConfiguration;
	interface Timer<TMilli> as PacketTimer;
  }
  #

}

implementation {

  /** @brief Message variables */
  message_t packet;
  uint8_t *nodeId;
  uint16_t packetsSent,packetsReceived;
  bool sends;

  /** @brief New variables */
  uint32_t slotSize;
  uint8_t curFrameSize;  	//Superframe length


   #define DELAY_BETWEEN_MESSAGES 30720 //(30 seconds) delay

  /***************** Prototypes ****************/
  task void send();

  /***************** Boot Events ****************/
  event void Boot.booted() {
  	sends = FALSE;
  	packetsSent = 0;
  	packetsReceived = 0;
	printf("APP: Attempting to start \n");
	call MacControl.start();

	// 1024 is one second
	call PacketTimer.startOneShot(1024 * 30);
  }

  	/***************** SplitControl Events ****************/
 	 event void MacControl.startDone(error_t error) {

	  	/** @brief New initialization part @aaks */
		slotSize = 500 * 32; 	//500 ms  -- 10ms standard
		curFrameSize = 100; 	// Superframe length is set to 100 slots

	  	/** @brief Configuring frame @aaks */
		call FrameConfiguration.setSlotLength(slotSize);
		call FrameConfiguration.setFrameLength(curFrameSize);
	    post send();
  	}

 	/**  @brief Timer block */
	event void PacketTimer.fired(){
		printf("APP: Timer fired  \n");
		call PacketTimer.startOneShot(DELAY_BETWEEN_MESSAGES);
		post send();
		return;
	}

 	/** @brief Handler for event sendDone for packet send interface to MAC */
  	event void MacSend.sendDone(message_t *msg, error_t error){
	  	printf("APP: Packets sent : %d \n", packetsSent);
	  	if (&packet == msg) {
	  		packetsSent++;
	  		sends = FALSE;
		}
 	}

  	/** @brief After MAC is stopped */
	event void MacControl.stopDone(error_t error) {
		/** @brief To be used to fill in Mac stopping procedure */
	}

	/***************** Receive Events ****************/
  	event message_t * MacReceive.receive(message_t *msg, void *payload, uint8_t len) {
		printf("APP: Packets received @aaks : %d \n", packetsReceived);
		packetsReceived++;
	    return msg;
	}

  	/** @brief Send task for sending packets to MAC */
  	task void send() {
	  	if(sends == FALSE)
	  	{
	    	if(call MacSend.send(&packet, 1) != SUCCESS) {
	      		printf("APP: Packet failed \n");
	    	} else {
	    		printf("APP: Pkt passed to MAC \n");
			}
	    }
    }
}
