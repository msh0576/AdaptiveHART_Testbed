//**************************************************************************
// * file:        TdmaMacTest application configuration file
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
#include "printf.h"

configuration TdmaMacTestC {

}

implementation {
  components MainC;
  components TdmaMacTestP;
  components LedsC;
  components TdmaMacC as MAC;
//  components MacC as MAC;
  components new TimerMilliC();

  TdmaMacTestP.Boot -> MainC;
  TdmaMacTestP.Leds -> LedsC;
  TdmaMacTestP.MacControl -> MAC;
  TdmaMacTestP.MacSend -> MAC;
  TdmaMacTestP.MacReceive -> MAC;
  TdmaMacTestP.FrameConfiguration -> MAC;

  TdmaMacTestP.PacketTimer -> TimerMilliC;

  components TimeSyncC;
  MainC.SoftwareInit -> TimeSyncC;
  TimeSyncC.Boot -> MainC;

  //Uart
/*
  components SerialActiveMessageC as Serial;
  TdmaMacTestP.UartReceive -> Serial.Receive;
  TdmaMacTestP.UartPacket -> Serial;
  TdmaMacTestP.UartAMPacket -> Serial;
*/


}
