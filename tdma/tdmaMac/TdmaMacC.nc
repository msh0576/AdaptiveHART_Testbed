//**************************************************************************
// * file:        TdmaMac configuration file
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

configuration TdmaMacC{

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
	components TdmaMacP;
	components GenericSlotterC;
	components CC2420ActiveMessageC as Phy;
	components new TimerMilliC();
	components new QueueC(dmamac_data_t *, 2);
	
	/** @brief Powering up */	
	Init = TdmaMacP.Init;
	
	/** @brief Power control and frame configuration interface for applications */
	MacPowerControl = TdmaMacP;
	FrameConfiguration = TdmaMacP.Frame;
	
	/** @brief Pkt send/receive interface for applications */
	MacSend = TdmaMacP;
	MacReceive = TdmaMacP;
	
	/** @brief Generic Slotter and slotter control stuff */
	TdmaMacP.GenericSlotterPowerControl -> GenericSlotterC;
	TdmaMacP.Slotter -> GenericSlotterC;
	TdmaMacP.SlotterControl -> GenericSlotterC;
	TdmaMacP.FrameConfiguration -> GenericSlotterC;
	
	//Uses
	TdmaMacP.RadioPowerControl -> Phy;
	TdmaMacP.AMPacket -> Phy;
	TdmaMacP.linkIndicator -> Phy;
	TdmaMacP.Packet -> Phy;
	TdmaMacP.ACK -> Phy;    								// Provided by CC2420Packet
	TdmaMacP.phyDataSend -> Phy.AMSend[AM_DMAMAC_DATA]; 	// 1 is the id used for data packets
	TdmaMacP.phyDataReceive -> Phy.Receive[AM_DMAMAC_DATA];
	
	TdmaMacP.phyNotificationSend -> Phy.AMSend[AM_DMAMAC_NOTIFICATION]; // 11 is the id used for notification packets
	TdmaMacP.phyNotificationReceive -> Phy.Receive[AM_DMAMAC_NOTIFICATION];
	
	TdmaMacP.forwardQueue -> QueueC;
	
	/** @brief Time Sync stuff */
	components TimeSyncC as BackGroundSync; 
	
	TdmaMacP.GlobalTime -> BackGroundSync;
	TdmaMacP.TimeSyncInfo -> BackGroundSync;
}

