#########################################################################
#                                                                       #
# DrawStuff Library, Copyright (C) 2001 Russell L. Smith.               #
#   Email: russ@q12.org   Web: www.q12.org                              #
#                                                                       #
# This library is free software; you can redistribute it and/or         #
# modify it under the terms of the GNU Lesser General Public            #
# License as published by the Free Software Foundation; either          #
# version 2.1 of the License, or (at your option) any later version.    #
#                                                                       #
# This library is distributed in the hope that it will be useful,       #
# but WITHOUT ANY WARRANTY; without even the implied warranty of        #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU      #
# Lesser General Public License for more details.                       #
#                                                                       #
# You should have received a copy of the GNU Lesser General Public      #
# License along with this library (see the file LICENSE.TXT); if not,   #
# write to the Free Software Foundation, Inc., 59 Temple Place,         #
# Suite 330, Boston, MA 02111-1307 USA.                                 #
#                                                                       #
#########################################################################

MAKEFILE_INC=../../build/Makefile.inc
include $(MAKEFILE_INC)

INCLUDE_PATHS = ../../include

LIB=../../lib/$(call fLIBNAME,drawstuff)
SRC = drawstuff.cpp

ifeq ($(PLATFORM_IS_WINDOWS),1)
  SRC += windows.cpp resources.rc
else
  SRC += x11.cpp
endif

all: $(LIB)
ifeq ($(PLATFORM_IS_WINDOWS),1)
	$(COPY_CMD) resources.RES ..$(DIRSEP)..$(DIRSEP)lib$(DIRSEP)drawstuff.res
endif

$(LIB): $(call fTARGETS,$(SRC))
