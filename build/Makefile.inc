#########################################################################
#                                                                       #
# Open Dynamics Engine, Copyright (C) 2001 Russell L. Smith.            #
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

# common makefile rules
#
# variables marked defined externally (variables marked (*) must be defined
# *before* this file is included):
#   INCLUDE_PATHS : extra include paths
#   LIB_PATHS : extra library paths
#   DEFINES : extra C/C++ defines
#   EXTRA_CLEAN : extra stuff to clean up for `clean' target
#   CPP_RTTI : defined to 1 if C++ RTTI should be used
#   CPP_EX : defined to 1 if C++ exceptions should be used
#   CPP_OPT : defined to 0,1,2 to set the level of C and C++ optimization
#   CPP_FLAGS : extra flags to C/C++ compiler
#   FAST : defined to 1 to apply extra non-detrimental optimizations
#   SUBDIRS : subdirectories to build (*)
#   SUBDIR_TARGET : target to build in subdirectories
#   SUBSYSTEM : set to `WINDOWS' or `CONSOLE', for Win32 targets only
#   COPY_CMD : the name of the copy command (typically "cp" or "copy")
#   DIRSEP : the directory separator character (typically "\" or "/")
#
# variables defined here
#   PLATFORM : win32, linux, etc
#
#################
#
# note: can do this to use different compilation variables for different
# targets:
#
#	$(LIB): $(call fTARGETS,$(SRC1)) $(call fTARGETS,$(SRC2))
#	$(call fTARGETS,$(SRC2)): CPP_EX=1
#
#############################################################################

include $(dir $(MAKEFILE_INC))/Makefile.config

default: all

.PHONY: clean clean_subdirs depend depend_subdirs make_depend subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)
     
$(SUBDIRS):
	$(MAKE) -C $@ $(SUBDIR_TARGET)

clean_subdirs: SUBDIR_TARGET=clean
clean_subdirs: subdirs

depend_subdirs: SUBDIR_TARGET=depend
depend_subdirs: subdirs

all: SUBDIR_TARGET=all
all: subdirs

depend: make_depend depend_subdirs

include $(dir $(MAKEFILE_INC))/Makefile.$(PLATFORM)

-include Dependencies
