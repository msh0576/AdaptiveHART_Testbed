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

#include "SenderDispatcher.h"
#include "TestPureTdma.h"

configuration PureTDMASchedulerC {
	provides {
		interface Init;
		interface AsyncSend as Send;
		interface AsyncReceive as Receive;
		interface SplitControl;
		interface CcaControl[am_id_t amId];
		interface FrameConfiguration;
	}
	uses {
		interface AsyncReceive as SubReceive;
		interface AsyncSend as SubSend;
		interface RadioPowerControl;
		interface AMPacket;
		interface Resend;
		interface PacketAcknowledgements;
		interface CcaControl as SubCcaControl[am_id_t];
		interface ChannelMonitor;
    //interface AMSend;
	}
}
implementation {
	components MainC;
	components PureTDMASchedulerP as App;
	components TDMASlotSenderC;
	components GenericSlotterC;
	components LedsC;
	components SenderDispatcherC;
	components DummyChannelMonitorC;
	components BeaconSlotC;
	components new Alarm32khz32C();


	//provides
	Init = App.Init;
	SplitControl = App.SplitControl;
	Send = App.Send;
	Receive = App.Receive;
	FrameConfiguration = App.Frame;

	//wire-in SchedulerP
	MainC.SoftwareInit -> App;

	App.GenericSlotter -> GenericSlotterC.AsyncStdControl;
	App.Slotter -> GenericSlotterC.Slotter;
	App.SlotterControl ->GenericSlotterC.SlotterControl;
	App.RadioPowerControl = RadioPowerControl;
	App.FrameConfiguration -> GenericSlotterC.FrameConfiguration;
	App.Resend = Resend;
	App.ACK = PacketAcknowledgements;
	App.AMPacket = AMPacket;
	App.Leds -> LedsC;
	App.CcaControl = CcaControl;

	SenderDispatcherC.SubSend = SubSend;
	SenderDispatcherC.SubCcaControl = SubCcaControl;
	TDMASlotSenderC.SubSend -> SenderDispatcherC.Send[TDMA_SLOT];
	TDMASlotSenderC.SubCcaControl -> SenderDispatcherC.SlotsCcaControl[TDMA_SLOT];

	TDMASlotSenderC.AMPacket = AMPacket;
	App.SubSend -> TDMASlotSenderC.Send;
	App.BeaconSend -> BeaconSlotC.Send;
	BeaconSlotC.SubReceive = SubReceive;
	App.SubReceive -> BeaconSlotC.Receive;


	//BeaconSlotC.ChannelMonitor = ChannelMonitor;
	BeaconSlotC.AMPacket = AMPacket;
	BeaconSlotC.SubCcaControl -> SenderDispatcherC.SlotsCcaControl[BEACON_SLOT];
	BeaconSlotC.SubSend -> SenderDispatcherC.Send[BEACON_SLOT];
	BeaconSlotC.SlotterControl ->GenericSlotterC.SlotterControl;
	BeaconSlotC.SyncAlarm -> Alarm32khz32C;

  ////////
  //App.CommandSend -> TDMASlotSenderC.Send;


	DummyChannelMonitorC.ChannelMonitor = ChannelMonitor;

	//components HplMsp430GeneralIOC;

	//App.Pin -> HplMsp430GeneralIOC.Port26;

	// For AdaptiveHART
	components new QueueC(adaptivehart_msg_t *, 1) as HIQueueC;
	components new QueueC(adaptivehart_msg_t *, 1) as LOQueueC;
	components new QueueC(adaptivehart_msg_t *, 1) as RootQueueC;
	App.HIforwardQ -> HIQueueC;
	App.LOforwardQ -> LOQueueC;
	App.RootQ -> RootQueueC;

	components new Alarm32khz16C() as offset_alarm;
	App.TxOffset_alarm -> offset_alarm;

	components AdaptiveHART_UtilC;
	App.AH_Util -> AdaptiveHART_UtilC;

	// For offset algorithm
	components CC2420ReceiveC;
	App.ReceiveIndicator -> CC2420ReceiveC;
}
