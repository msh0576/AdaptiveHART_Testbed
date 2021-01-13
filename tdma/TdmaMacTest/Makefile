#**************************************************************************
# * file:        Makefile for the application TdmaMacTest
# *
# * author:      A. Ajith Kumar S.
# * copyright:   (c) A. Ajith Kumar S.
# * homepage:    www.hib.no/ansatte/aaks
# * email:       aji3003 @ gmail.com
# **************************************************************************
# * part of:     TinyOS MAC tutorial.
# * Refined on:  26-June-2015
# **************************************************************************
# *This file is part of TinyOS MAC tutorial.
# *
# *TinyOS MAC tutorial is free software: you can redistribute it and/or modify
# *it under the terms of the GNU General Public License as published by
# *the Free Software Foundation, either version 3 of the License, or
# *(at your option) any later version.
# *
# *TinyOS MAC tutorial is distributed in the hope that it will be useful,
# *but WITHOUT ANY WARRANTY; without even the implied warranty of
# *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# *GNU General Public License for more details.
# *
# *You should have received a copy of the GNU General Public License
# *along with TinyOS MAC tutorial.  If not, see <http://www.gnu.org/licenses/>./
# **************************************************************************
COMPONENT=TdmaMacTestC
#Printing enabled
#CFLAGS += -DPRINTFUART_ENABLED
CFLAGS += -I$(TOSDIR)/lib/printf
PFLAGS += -DNEW_PRINTF_SEMANTICS -DPRINTFUART_ENABLED

#Time Synchronization stuff Sync_rate gives the rate at which message exhange is done
PFLAGS += -I$(TOSDIR)/lib/ftsp
PFLAGS += -DTIMESYNC_RATE=1

#####by HS
PFLAGS += -DCC2420_DEF_CHANNEL=20

#Including the Mac make rules and other make rules
#Using <<TdmaMac>> designed for tutorial
include $(MAKERULES)
include $(TOSDIR)/lib/mac/tdmaMac/Makefile.include
