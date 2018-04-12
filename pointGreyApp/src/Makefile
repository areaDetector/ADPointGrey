TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += pointGrey
LIBRARY_IOC_Linux += pointGrey

LIB_SRCS += pointGrey.cpp

LIB_LIBS_WIN32     += FlyCapture2
# We use LIB_SYS_LIBS even though libflycapture.so is in the ADPointGrey tree 
# because we are doing static build but the file is a .so file, not a .a file.
LIB_SYS_LIBS_Linux += flycapture

DBD += pointGreySupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

