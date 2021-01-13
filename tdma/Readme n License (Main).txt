@author 	: A. Ajith Kumar S.
@homepage 	: http://home.hib.no/ansatte/aaks/
@date 		: June 2015
@place		: Bergen University College, Norway
@copyright  : (c) A. Ajith Kumar S. 
@description: 
======================================================================
@License
======================================================================
This file is part of TinyOS MAC tutorial with a TDMA example.

TinyOS MAC tutorial: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

TinyOS MAC tutorial is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with TinyOS MAC tutorial.  If not, see <http://www.gnu.org/licenses/>.
======================================================================
@contentDescription
======================================================================
Main file : Mac tutorial, describes stepwise procedure to create a new MAC
protocol implementation in tinyos. The files need to be created and proper placeholders
for the created files.

MAC  Sources: TdmaMacC.nc, TdmaMacP.nc, Makefile
TdmaMacC.nc is the configuration file for the TDMA MAC 
TdmaMacP.nc is the implementation module for the mac protocol.
Five interfaces are provided by the protocol.
	Init - First initialization.
	PowerControl - Start/stop control of MAC.
	Send - For sending packets via the MAC.
	Receive - For receiving packets via the MAC.
	FrameConfiguration - Setup of slot length and frame length for every round.
	
Component graph diagrams are also provided.
TdmaMacC.png and TdmaMacTestC.png
	
Also an example application to use the MAC is provided.
TdmaMacTestC.nc (Configuration) and TdmaMacTestP.nc. (Implementation)
The application sends packets to MAC continuously every 30 second.
Also, initializes the time synchronization protocol FTSP in the background.
Time synchronization is an important part of a TDMA protocol.
======================================================================