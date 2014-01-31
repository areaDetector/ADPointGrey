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

# Note, the following files were manually copied from the Perkin Elmer distribution XISL SDK32 and SKD64 directories
ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
#LIB_INSTALLS_WIN32    += ../os/win32-x86/XISL.lib
#BIN_INSTALLS_WIN32    += ../os/win32-x86/XISL.dll

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/windows-x64/FlyCapture2.lib
#BIN_INSTALLS_WIN32    += ../os/windows-x64/XISL.dll

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
