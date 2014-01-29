< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/pixiradApp.dbd")
pixiradApp_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("PREFIX", "13PR1:")
epicsEnvSet("COMMAND_PORT", "PIXI_CMD")
epicsEnvSet("STATUS_PORT", "2224")
epicsEnvSet("DATA_PORT", "2223")
epicsEnvSet("DATA_PORT_BUFFERS", "1500")
epicsEnvSet("PORT",   "PIXI")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "476")
epicsEnvSet("YSIZE",  "512")
epicsEnvSet("NCHANS", "2048")

###
# Create the asyn port to talk to the Pixirad box on port 2222.
drvAsynIPPortConfigure("$(COMMAND_PORT)","192.168.0.1:2222 HTTP", 0, 0, 0)
asynOctetSetOutputEos($(COMMAND_PORT), 0, "\n")
asynSetTraceIOMask($(COMMAND_PORT), 0, 2)
#asynSetTraceMask($(COMMAND_PORT), 0, 9)

pixiradConfig("$(PORT)", "$(COMMAND_PORT)", "$(DATA_PORT)", "$(STATUS_PORT)", $(DATA_PORT_BUFFERS), $(XSIZE), $(YSIZE))
asynSetTraceIOMask($(PORT), 0, 2)
#asynSetTraceMask($(PORT), 0, 255)

dbLoadRecords("$(ADCORE)/db/ADBase.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADCORE)/db/NDFile.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("$(ADPIXIRAD)/db/pixirad.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,SERVER_PORT=$(COMMAND_PORT)")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDPluginBase.template","P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=243712")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADPIXIRAD)/pixiradApp/Db")



iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX),D=cam1:")
