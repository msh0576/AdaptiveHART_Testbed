#include "printf.h"
#include <Timer.h>

// FOR BASESTATION
#include "AM.h"
#include "Serial.h"
// END

module TestPureTdmaP {
  uses {
    interface Boot;
    interface Leds;
    interface AMSend as AMSender;
    interface Receive as AMReceiver;
    //interface AsyncSend as AMSender;
    //interface AsyncReceive as AMReceiver;
    interface Packet;
    interface Timer<TMilli> as SendTimer;
    interface Timer<TMilli> as FlushTimer;
    interface Timer<TMilli> as BenchmarkTimer;
    interface Timer<TMilli> as StartTimer;
    interface Counter<T32khz, uint32_t> as Counter;
    interface SplitControl;

    interface HplMsp430GeneralIO as Pin;
    interface HplMsp430GeneralIO as RcvPin;

    interface Routing;
    //interface AMSend as CommSender;
    //interface GlobalTime<TMilli>;
    //interface TimeSyncInfo;

    /* for integrating the basestation app in this app*/
    interface SplitControl as SerialControl;
    interface SplitControl as RadioControl;
    interface AMSend as UartSend[am_id_t id];
    interface Receive as UartReceive[am_id_t id];
    interface Packet as UartPacket;
    interface AMPacket as UartAMPacket;
    
    interface AMSend as RadioSend[am_id_t id];
    interface Receive as RadioReceive[am_id_t id];
    interface Receive as RadioSnoop[am_id_t id];
    interface Packet as RadioPacket;
    interface AMPacket as RadioAMPacket;
  }
}

implementation {
	enum {
		BENCHMARK_LENGTH = 10 * 1024,
		// FOR BASESTATION
		UART_QUEUE_LEN = 12,
		RADIO_QUEUE_LEN = 12,
		// END
	};

	message_t  packet;
	uint8_t    *nodeId;
	uint16_t   packetsSent, packetsReceived;
	bool       sends;
	uint32_t   sendTime;
	uint32_t   latency;

	am_addr_t  parentId[MAX_NODES];
	uint8_t    i;
	uint16_t   SequenceCount;
	am_addr_t  *neighborTable;

	// FOR BASESTATION
	message_t  uartQueueBufs[UART_QUEUE_LEN];
	message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
	uint8_t    uartIn, uartOut;
	bool       uartBusy, uartFull;

	message_t  radioQueueBufs[RADIO_QUEUE_LEN];
	message_t  * ONE_NOK radioQueue[RADIO_QUEUE_LEN];
	uint8_t    radioIn, radioOut;
	bool       radioBusy, radioFull;
	// END

	task void sendTask();

	// FOR BASESTATION
	task void uartSendTask();
	task void radioSendTask();

	void dropBlink() { call Leds.led2Toggle(); }
  	void failBlink() { call Leds.led2Toggle(); }
  	// END

	event void Boot.booted() {
		sends = FALSE;
		packetsSent = 0;
		packetsReceived  = 0;
		latency = 0;
		SequenceCount=0;

		post sendTask();
		call FlushTimer.startPeriodic(250);
		call SplitControl.start();

		for (i = 0; i < MAX_NODES; i++) {
			//nbReceivedPkts[i] = 0;
			parentId[i] = call Routing.MakeParent((am_addr_t)i, NULL);
		}
    		printf("<APP>:My parent ID: %u\r\n", parentId[TOS_NODE_ID]);
    		printfflush();

		// FOR BASESTATION
		for (i = 0; i < UART_QUEUE_LEN; i++) 
			uartQueue[i] = &uartQueueBufs[i];

    		uartIn = uartOut = 0;
    		uartBusy = FALSE;
    		uartFull = TRUE;

		for (i = 0; i < RADIO_QUEUE_LEN; i++) 
			radioQueue[i] = &radioQueueBufs[i];

		radioIn = radioOut = 0;
		radioBusy = FALSE;
		radioFull = TRUE;

    		if (call RadioControl.start() == EALREADY) radioFull = FALSE;
    		if (call SerialControl.start() == EALREADY) uartFull = FALSE;
  		// END

	} // end[Boot.booted()]

	task void sendTask() {
		// printf("send task\n");
		if(sends == FALSE) {
			//call Pin.set();
			//call Leds.led0Toggle();
			printf("app send\r\n");

			if(TOS_NODE_ID == 0) {
				TdmaMsg_t *TdmaMsg=call Packet.getPayload(&packet, sizeof(TdmaMsg_t));
				//TdmaMsg->ParentID=parentId[TOS_NODE_ID];
				//TdmaMsg->NodeID=TOS_NODE_ID;

				SequenceCount++;
				TdmaMsg->SequenceNo=SequenceCount;

				//printf("P.ID = %u  and  N.ID = &u\r\n",TdmaMsg->ParentID, TdmaMsg->NodeID);
				//printfflush();


			        if (call AMSender.send(AM_BROADCAST_ADDR, &packet, sizeof(TdmaMsg_t)) != SUCCESS) {
	  				post sendTask();
	  			} else {
	  				sends = TRUE;
	  				sendTime = call Counter.get();
	  			}

			} else if(TOS_NODE_ID != 0) {
		      		TdmaMsg_t *TdmaMsg=call Packet.getPayload(&packet, sizeof(TdmaMsg_t));
		        	//TdmaMsg->ParentID=parentId[TOS_NODE_ID];
        			//TdmaMsg->NodeID=TOS_NODE_ID;

        			//printf("P.ID = %u  and  N.ID = %u\r\n",TdmaMsg->ParentID, TdmaMsg->NodeID);
      				//printfflush();

      				SequenceCount++;
        			TdmaMsg->SequenceNo=SequenceCount;


				/*
				if (call AMSender.send(parentId[TOS_NODE_ID], &packet, sizeof(TdmaMsg_t)) != SUCCESS) {
				  post sendTask();
				} else {
				  sends = TRUE;
				  sendTime = call Counter.get();
				}
				*/

				if (call AMSender.send(AM_BROADCAST_ADDR, &packet, sizeof(TdmaMsg_t)) != SUCCESS) {
					post sendTask();
				} else {
				  	sends = TRUE;
				  	sendTime = call Counter.get();
				}
      			} // end[if:TOS_NODE_ID]
		} // end[if:sends]
	} // end[task:sendTask]


	event void AMSender.sendDone(message_t * bufPtr, error_t error)	{
		if (&packet == bufPtr) {
			uint32_t now = call Counter.get();
			latency = latency + (now - sendTime);

			printf("app send done\r\n");
			printf("app send done now=%lu sendTime=%lu latency=%lu\r\n", now, sendTime, (now - sendTime));
      			printfflush();
			//call Leds.led0Toggle();
			packetsSent++;
			sends = FALSE;
			//call Pin.toggle();
			post sendTask();
		}
	} // end[AMSender.sendDone]

	task void flush() {
		printfflush();
	}

	async event void Counter.overflow() {
		printf("Overflow");
		post flush();
	}

	event void FlushTimer.fired() {
		//call Leds.led2Toggle();
		printfflush();
	}

	event void SendTimer.fired() {
		//post sendTask();
	}

	event message_t * AMReceiver.receive(message_t * message, void * payload, uint8_t length) {
		//call RcvPin.toggle();
		packetsReceived++;
		return message;
	}

	event void SplitControl.startDone(error_t err) {
		nodeId = (uint8_t *)call Packet.getPayload(&packet, sizeof(uint8_t));
		*nodeId = TOS_NODE_ID;

		printf("starting\r\n");
    		printfflush();

		if (TOS_NODE_ID != 0)  {
			call SendTimer.startPeriodic(3000);
		}
		call StartTimer.startOneShot(1000);
	}

	event void SplitControl.stopDone(error_t err) {
	}

	event void StartTimer.fired() {
		call BenchmarkTimer.startPeriodic(BENCHMARK_LENGTH);
	}

	event void BenchmarkTimer.fired() {
		//call Leds.led0On();
		uint32_t avg_latency = latency / packetsSent;

		//printf("STATS: %u %u %lu\r\n", packetsSent, packetsReceived, avg_latency);
		packetsSent = 0;
		packetsReceived  = 0;
		latency = 0;
		//printfflush();
	}

  // FOR BASESTATION
  event void RadioControl.startDone(error_t error) {
    if (error == SUCCESS) {
      radioFull = FALSE;
    }
  }

  event void SerialControl.startDone(error_t error) {
    if (error == SUCCESS) {
      uartFull = FALSE;
    }
  }

  event void SerialControl.stopDone(error_t error) {}
  event void RadioControl.stopDone(error_t error) {}

  uint8_t count = 0;

  message_t* ONE receive(message_t* ONE msg, void* payload, uint8_t len);
  
  event message_t *RadioSnoop.receive[am_id_t id](message_t *msg,
						    void *payload,
						    uint8_t len) {
    return receive(msg, payload, len);
  }
  
  event message_t *RadioReceive.receive[am_id_t id](message_t *msg,
						    void *payload,
						    uint8_t len) {
    return receive(msg, payload, len);
  }

	message_t* receive(message_t *msg, void *payload, uint8_t len) {
		message_t *ret = msg;

    		atomic {
    			if (!uartFull) {
	  			ret = uartQueue[uartIn];
	  			uartQueue[uartIn] = msg;

	 			uartIn = (uartIn + 1) % UART_QUEUE_LEN;
	
	  			if (uartIn == uartOut)
	    				uartFull = TRUE;

	  			if (!uartBusy) {
	    				post uartSendTask();
	    				uartBusy = TRUE;
	    			}
     			} else
				dropBlink();
     		}
    
	    	return ret;
	}

  uint8_t tmpLen;
  
  task void uartSendTask() {
    uint8_t len;
    am_id_t id;
    am_addr_t addr, src;
    message_t* msg;
    am_group_t grp;
    atomic
      if (uartIn == uartOut && !uartFull)
	{
	  uartBusy = FALSE;
	  return;
	}

    msg = uartQueue[uartOut];
    tmpLen = len = call RadioPacket.payloadLength(msg);
    id = call RadioAMPacket.type(msg);
    addr = call RadioAMPacket.destination(msg);
    src = call RadioAMPacket.source(msg);
    grp = call RadioAMPacket.group(msg);
    call UartPacket.clear(msg);
    call UartAMPacket.setSource(msg, src);
    call UartAMPacket.setGroup(msg, grp);

    if (call UartSend.send[id](addr, uartQueue[uartOut], len) == SUCCESS)
      call Leds.led1Toggle();
    else
      {
	failBlink();
	post uartSendTask();
      }
  }

  event void UartSend.sendDone[am_id_t id](message_t* msg, error_t error) {
    if (error != SUCCESS)
      failBlink();
    else
      atomic
	if (msg == uartQueue[uartOut])
	  {
	    if (++uartOut >= UART_QUEUE_LEN)
	      uartOut = 0;
	    if (uartFull)
	      uartFull = FALSE;
	  }
    post uartSendTask();
  }

  event message_t *UartReceive.receive[am_id_t id](message_t *msg,
						   void *payload,
						   uint8_t len) {
    message_t *ret = msg;
    bool reflectToken = FALSE;

    atomic
      if (!radioFull) {
	  reflectToken = TRUE;
	  ret = radioQueue[radioIn];
	  radioQueue[radioIn] = msg;
	  if (++radioIn >= RADIO_QUEUE_LEN)
	    radioIn = 0;
	  if (radioIn == radioOut)
	    radioFull = TRUE;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  if (!radioBusy) {
	    post radioSendTask();
	    radioBusy = TRUE;
	  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      } else
	dropBlink();

      if (reflectToken) {
        //call UartTokenReceive.ReflectToken(Token);
      }
    
    return ret;
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  task void radioSendTask() {
    uint8_t len;
    am_id_t id;
    am_addr_t addr,source;
    message_t* msg;
    
    atomic
      if (radioIn == radioOut && !radioFull)
	{
	  radioBusy = FALSE;
	  return;
	}

    msg = radioQueue[radioOut];
    len = call UartPacket.payloadLength(msg);
    addr = call UartAMPacket.destination(msg);
    source = call UartAMPacket.source(msg);
    id = call UartAMPacket.type(msg);

    call RadioPacket.clear(msg);
    call RadioAMPacket.setSource(msg, source);
    
    if (call RadioSend.send[id](addr, msg, len) == SUCCESS)
      call Leds.led0Toggle();
    else
      {
	failBlink();
	post radioSendTask();
      }
  }

  event void RadioSend.sendDone[am_id_t id](message_t* msg, error_t error) { 
    if (error != SUCCESS)
      failBlink();
    else
      atomic
	if (msg == radioQueue[radioOut])
	  {
	    if (++radioOut >= RADIO_QUEUE_LEN)
	      radioOut = 0;
	    if (radioFull)
	      radioFull = FALSE;
	  }
    
    post radioSendTask();
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// END BASESTATION

}
