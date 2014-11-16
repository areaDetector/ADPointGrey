TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

PROD_NAME = pointGreyApp
PROD_IOC_WIN32 += $(PROD_NAME)
PROD_IOC_Linux += $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += pointGreySupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

# Test program
PROD_IOC_WIN32 += GigEGrabEx
PROD_IOC_Linux += GigEGrabEx
GigEGrabEx_SRCS += GigEGrabEx

PROD_IOC_WIN32 += ExtendedShutterEx
PROD_IOC_Linux += ExtendedShutterEx
ExtendedShutterEx_SRCS += ExtendedShutterEx

PROD_IOC_WIN32 += PGTest
PROD_IOC_Linux += PGTest
PGTest_SRC += PGTest.cpp
PROD_IOC_WIN32 += PGTest1
PROD_IOC_Linux += PGTest1
PGTest_SRC += PGTest1.cpp

# Add locally compiled object code
PROD_LIBS += pointGrey
PROD_LIBS_WIN32 += FlyCapture2
# We use PROD_SYS_LIBS even though libflycapture.so is in the ADPointGrey tree 
# because we are doing static build but the file is a .so file, not a .a file.
PROD_SYS_LIBS_Linux += flycapture

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

