TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = pointGreyApp
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
  PROD_IOC += $(PROD_NAME)
else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
  PROD_IOC += $(PROD_NAME)

endif

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += pointGreySupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Add locally compiled object code
PROD_LIBS += pointGrey
PROD_LIBS_WIN32 += FlyCapture2
PROD_SYS_LIBS_Linux += flycapture

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

