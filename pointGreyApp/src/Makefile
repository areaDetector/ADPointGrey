TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
  LIBRARY_IOC += pointGrey
endif

LIB_SRCS += pointGrey.cpp

DBD += pointGreySupport.dbd

ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
  PROD += GrabCallbackEx
endif
PROD_SRCS += GrabCallbackEx.cpp
PROD_LIBS += FlyCapture2

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

