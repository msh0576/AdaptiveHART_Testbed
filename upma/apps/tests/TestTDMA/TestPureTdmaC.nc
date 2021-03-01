/*
 * "Copyright (c) 2007 Washington University in St. Louis.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author Appear in all copies of this software.
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
#include <stdio.h>
configuration TestPureTdmaC
{

}
implementation
{

components SerialStartC;
components PrintfC;
components MainC;
components ActiveMessageC;
components HplMsp430GeneralIOC;
//components SerialPrintfC;  // I disabled it to work properly (by bm)

components TestPureTdmaP as App;
components new AMSenderC(240) as AMSender;
//components new AMSenderC(130) as CommSender;
components LedsC;
components new TimerMilliC();
components new TimerMilliC() as FlushTimerC;
components new TimerMilliC() as BenchmarkTimerC;
components new TimerMilliC() as StartTimerC;
components new AMReceiverC(240) as AMReceiver;

components Counter32khz32C;

App.BenchmarkTimer -> BenchmarkTimerC;
App.StartTimer -> StartTimerC;
App.Pin -> HplMsp430GeneralIOC.Port27;
App.RcvPin -> HplMsp430GeneralIOC.Port26;
App.Boot -> MainC;

App.FlushTimer -> FlushTimerC;
App.Boot -> MainC;
App.Leds -> LedsC;
App.AMSender -> AMSender;
App.AMReceiver -> AMReceiver;
App.Packet -> ActiveMessageC;
App.SendTimer -> TimerMilliC;
App.SplitControl -> ActiveMessageC;
App.Counter -> Counter32khz32C;


// for intergrating the basestation application
components SerialActiveMessageC as Serial;
components ActiveMessageC as Radio;

  App.RadioControl -> Radio;
  App.SerialControl -> Serial;
  
  App.UartSend -> Serial;
  App.UartReceive -> Serial.Receive;
  App.UartPacket -> Serial;
  App.UartAMPacket -> Serial;
  
  App.RadioSend -> Radio;
  App.RadioReceive -> Radio.Receive;
  App.RadioSnoop -> Radio.Snoop;
  App.RadioPacket -> Radio;
  App.RadioAMPacket -> Radio;
  

components RoutingTableC;
App.Routing -> RoutingTableC;

//
//App.CommSender -> CommSender;

////
//components TimeSyncC;

//App.GlobalTime -> TimeSyncC;
//App.TimeSyncInfo -> TimeSyncC;
}
