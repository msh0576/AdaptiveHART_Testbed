//**************************************************************************
// * file:        TdmaMac header file
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
 
#ifndef TDMAMAC_H
#define TDMAMAC_H

#define PERIOD 2000
#define MAX_CHILDREN 3 
#define MAX_NODES 5
#define RSSI_OFFSET 45

typedef nx_struct notify
{
	nx_uint16_t rootId;
	nx_uint8_t currentSlot;
} notification_t;

typedef nx_struct dmamacData
{
	nx_uint16_t nodeId;
	nx_uint16_t sequenceNo;
	nx_uint8_t rootId;
} dmamac_data_t;

enum {
  AM_DMAMAC_NOTIFICATION = 10,
  AM_DMAMAC_DATA = 11,
};

#endif /* TDMAMAC_H */

