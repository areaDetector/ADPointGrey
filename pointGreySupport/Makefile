TOP=../
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

INC += AVIRecorder.h
INC += BusManager.h
INC += Camera.h
INC += CameraBase.h
INC += Error.h
INC += FlyCapture2.h
INC += FlyCapture2Defs.h
INC += FlyCapture2Platform.h
INC += GigECamera.h
INC += Image.h
INC += ImageStatistics.h
INC += TopologyNode.h
INC += Utilities.h

# Note, the following files were manually copied from the Point Grey distribution lib directories on Windows
# and from /usr/lib on Linux.
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS    += ../os/windows-x64/FlyCapture2.lib

else ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_INSTALLS    += ../os/win32-x86/FlyCapture2.lib

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS    += ../os/linux-x86_64/libflycapture.so
LIB_INSTALLS    += ../os/linux-x86_64/libflycapture.so.2
LIB_INSTALLS    += ../os/linux-x86_64/libflycapture.so.2.8.3.1

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS    += ../os/linux-x86/libflycapture.so
LIB_INSTALLS    += ../os/linux-x86/libflycapture.so.2
LIB_INSTALLS    += ../os/linux-x86/libflycapture.so.2.8.3.1

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

