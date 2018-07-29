< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/pointGreyApp.dbd")
pointGreyApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "13PG1:")
# Use this line for the first Point Grey camera in the system
#epicsEnvSet("CAMERA_ID", "0")
# Use this line for a specific camera by serial number, in this case a Flea2 Firewire camera
#epicsEnvSet("CAMERA_ID", "9211601")
# Use this line for a specific camera by serial number, in this case a Grasshopper3 USB-3.0 camera
# This is the GSECARS tomography camera
#epicsEnvSet("CAMERA_ID", "13510305")
# This is another GSECARS tomography camera
#epicsEnvSet("CAMERA_ID", "15355695")
# This is another GSECARS tomography camera
#epicsEnvSet("CAMERA_ID", "15337483")
# This is the 13-ID-D Grasshopper3 GigE camera, gse-pointgrey2
#epicsEnvSet("CAMERA_ID", "14481221")
# This is the 13-ID-D Grasshopper3 GigE camera, gse-pointgrey3
#epicsEnvSet("CAMERA_ID", "14481209")
# This is the GSECARS LVP CMOS camera
#epicsEnvSet("CAMERA_ID", "14120134")
# This is the GSECARS LVP CCD camera
#epicsEnvSet("CAMERA_ID", "15452742")
# This is the 2-BM camera
#epicsEnvSet("CAMERA_ID", "15355637")
# This is the 2-BM GS3-U3-91S6M-C camera
#epicsEnvSet("CAMERA_ID", "13510309")
# Use this line for a specific camera by serial number, in this case a BlackFly GigE camera
#epicsEnvSet("CAMERA_ID", "13481965")
#epicsEnvSet("CAMERA_ID", "16292610")
epicsEnvSet("CAMERA_ID", "18402100")
#epicsEnvSet("CAMERA_ID", "1624484")
# Use this line for a specific camera by serial number, in this case a Flea3 GigE camera
# epicsEnvSet("CAMERA_ID", "14273040")
#epicsEnvSet("CAMERA_ID", "17476170")

# The port name for the detector
epicsEnvSet("PORT",   "PG1")
# Really large queue so we can stream to disk at full camera speed
epicsEnvSet("QSIZE",  "2000")   
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "2048")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "2048")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")
# Define NELEMENTS to be enough for a 2048x2048x3 (color) image
epicsEnvSet("NELEMENTS", "12592912")

# pointGreyConfig(const char *portName, int cameraId, int traceMask, int memoryChannel,
#                 int maxBuffers, size_t maxMemory, int priority, int stackSize)
pointGreyConfig("$(PORT)", $(CAMERA_ID), 0x1, 0)
asynSetTraceIOMask($(PORT), 0, 2)
#asynSetTraceMask($(PORT), 0, 0xFF)
#asynSetTraceFile($(PORT), 0, "asynTrace.out")
#asynSetTraceInfoMask($(PORT), 0, 0xf)

dbLoadRecords("$(ADPOINTGREY)/db/pointGrey.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadTemplate("pointGrey.substitutions")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
# Use this line for 8-bit data only
#dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int8,FTVL=CHAR,NELEMENTS=$(NELEMENTS)")
# Use this line for 8-bit or 16-bit data
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS)")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADPOINTGREY)/pointGreyApp/Db")

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")

# Wait for enum callbacks to complete
epicsThreadSleep(1.0)

# Records with dynamic enums need to be processed again because the enum values are not available during iocInit.  
dbpf("$(PREFIX)cam1:Format7Mode.PROC", "1")
dbpf("$(PREFIX)cam1:PixelFormat.PROC", "1")

# Wait for callbacks on the property limits (DRVL, DRVH) to complete
epicsThreadSleep(1.0)

# Records that depend on the state of the dynamic enum records or property limits also need to be processed again
# Other property records may need to be added to this list
dbpf("$(PREFIX)cam1:FrameRate.PROC", "1")
dbpf("$(PREFIX)cam1:FrameRateValAbs.PROC", "1")
dbpf("$(PREFIX)cam1:AcquireTime.PROC", "1")

