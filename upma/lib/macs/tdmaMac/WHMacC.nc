
#include "TdmaMac.h"
#include <stdio.h>

configuration WHMacC{

	provides { //General purpose
		interface Init;
		interface FrameConfiguration;
	}

	provides { // Radio Control
		interface Send as MacSend;
		interface Receive as MacReceive;
		interface SplitControl as MacPowerControl;
	}


}
implementation {
	components WHMacP;
	components GenericSlotterC;
	components CC2420ActiveMessageC as Phy;
	components new TimerMilliC();
	components new QueueC(dmamac_data_t *, 10);
    components SerialPrintfC;
		components LedsC;
	/** @brief Powering up */
	Init = WHMacP.Init;

	/** @brief Power control and frame configuration interface for applications */
	MacPowerControl = WHMacP;
	FrameConfiguration = WHMacP.Frame;

	/** @brief Generic Slotter and slotter control stuff */

	WHMacP.Slotter -> GenericSlotterC;
	WHMacP.SlotterControl -> GenericSlotterC;
	WHMacP.FrameConfiguration -> GenericSlotterC;
	WHMacP.GenericSlotterPowerControl -> GenericSlotterC.AsyncStdControl;   //this is the error

	/** @brief Pkt send/receive interface for applications */
	MacSend = WHMacP;
	MacReceive = WHMacP;



	//Uses
	WHMacP.RadioPowerControl -> Phy;
	WHMacP.AMPacket -> Phy;
	WHMacP.linkIndicator -> Phy;
	WHMacP.Packet -> Phy;
	WHMacP.ACK -> Phy;    								// Provided by CC2420Packet
	WHMacP.phyDataSend -> Phy.AMSend[AM_DMAMAC_DATA]; 	// 1 is the id used for data packets
	WHMacP.phyDataReceive -> Phy.Receive[AM_DMAMAC_DATA];

	WHMacP.phyNotificationSend -> Phy.AMSend[AM_DMAMAC_NOTIFICATION]; // 11 is the id used for notification packets
	WHMacP.phyNotificationReceive -> Phy.Receive[AM_DMAMAC_NOTIFICATION];

	WHMacP.forwardQueue -> QueueC;

	/** @brief Time Sync stuff */
	components TimeSyncC as BackGroundSync;

	WHMacP.GlobalTime -> BackGroundSync;
	WHMacP.TimeSyncInfo -> BackGroundSync;

	WHMacP.Leds -> LedsC;

	//components new GenericSlotSenderC(0, 0, FALSE) as TDMASlotSender;
	//TDMASlotSender.SubSend = SubSend;
/*
	components TDMASlotSenderC;
	components SenderDispatcherC;

	SenderDispatcherC.SubSend = SubSend;
	TDMASlotSenderC.SubSend -> SenderDispatcherC.Send[TDMA_SLOT];
	WHMacP.phyDataSend -> TDMASlotSenderC.Send;
*/
}
