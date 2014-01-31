/*
 * pointGrey.cpp
 *
 * This is a driver for PointGrey GigE, Firewire, and USB3 cameras using their FlyCapture2 SDK
 *
 * Author: Mark Rivers
 *         Univerity of Chicago
 *
 * Created: January 29, 2014
 *
 */

#include <stdio.h>
#include <string.h>
#ifdef _WIN32
  #include <tchar.h>
  #include <windows.h>
#else
  #include <unistd.h>
#endif

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExit.h>

#include <FlyCapture2.h>
using namespace FlyCapture2;

#include "ADDriver.h"

#include <epicsExport.h>

static const char *driverName = "pointGrey";

class pointGrey : public ADDriver {
public:
    pointGrey(const char *portName, int cameraId, int maxBuffers, 
              size_t maxMemory, int priority, int stackSize);

    /* Override ADDriver methods */ 
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual void report(FILE *fp, int details);

    /* "private methods" that need to be called from C */
    void shutdown();
    void imageTask();

protected:
    int PGFrameRate_;
    #define FIRST_PG_PARAM PGFrameRate_
    int PGImagesDropped_;
    int PGSerialNumber_;
    int PGFirmwareVersion_;
    int PGSoftwareVersion_;
    #define LAST_PG_PARAM PGSoftwareVersion_
private:
    inline asynStatus checkError(Error error, const char *functionName, const char *message);
    asynStatus connectCamera();
    asynStatus disconnectCamera();
    asynStatus setROI();
    asynStatus getROI();
    asynStatus readStatus();
    asynStatus setPropertyDouble(PropertyType propertyType, double value);
    int cameraId_;
    PGRGuid *pGuid_;
    Camera *pCamera_;
    BusManager *pBusMgr_;
    int exiting_;
    epicsEventId startEvent_;
};
#define NUM_PG_PARAMS ((int)(&LAST_PG_PARAM - &FIRST_PG_PARAM + 1))

/* PointGrey driver specific parameters */
#define PGFrameRateString        "PG_FRAME_RATE"        /* asynFloat64  rw */
#define PGImagesDroppedString    "PG_IMAGES_DROPPED"    /* asynInt32    ro */
#define PGSerialNumberString     "PG_SERIAL_NUMBER"     /* asynInt32    ro */
#define PGFirmwareVersionString  "PG_FIRMWARE_VERSION"  /* asynOctet    ro */
#define PGSoftwareVersionString  "PG_SOFTWARE_VERSION"  /* asynOctet    ro */

static void c_shutdown(void *arg)
{
  pointGrey *p = (pointGrey *)arg;
  p->shutdown();
}

static void c_imageTask(void *arg)
{
  pointGrey *p = (pointGrey *)arg;
  p->imageTask();
}

extern "C" int pointGreyConfig(const char *portName, int cameraId, int maxBuffers,
                               size_t maxMemory, int priority, int stackSize)
{
    new pointGrey(portName, cameraId, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

/** Constructor for PointGrey driver; most parameters are simply passed to
  * ADDriver::ADDriver.
  *
  * After calling the base class constructor this method creates a thread to
  * collect the images from the detector and sets reasonable default values for
  * the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  *
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] cameraId The id number of the PointGrey camera (see listdevices
  *            example for number).
  * \param[in] maxBuffers The maximum number of NDArray buffers that the
  *            NDArrayPool for this driver is allowed to allocate. Set this to
  *            -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for
  *            this driver is allowed to allocate. Set this to -1 to allow an
  *            unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread
  *            if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if
  *            ASYN_CANBLOCK is set in asynFlags.
  */
pointGrey::pointGrey(const char *portName, int cameraId, int maxBuffers,
                     size_t maxMemory, int priority, int stackSize)
    : ADDriver(portName, 1, NUM_PG_PARAMS, maxBuffers, maxMemory,
               asynEnumMask, asynEnumMask, 
               ASYN_CANBLOCK,  /* ASYN_CANBLOCK=1 ASYN_MULTIDEVICE=0 */
               1,              /* autoConnect=1 */
               priority, stackSize),
      cameraId_(cameraId), pGuid_(0), exiting_(0)
{
    static const char *functionName = "pointGrey";
    asynStatus status;

    /* create PointGrey specific parameters */
    createParam(PGFrameRateString,        asynParamFloat64, &PGFrameRate_);
    createParam(PGImagesDroppedString,    asynParamInt32,   &PGImagesDropped_);
    createParam(PGSerialNumberString,     asynParamInt32,   &PGSerialNumber_);
    createParam(PGFirmwareVersionString,  asynParamOctet,   &PGFirmwareVersion_);
    createParam(PGSoftwareVersionString,  asynParamOctet,   &PGSoftwareVersion_);

    /* set read-only parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");
    
    // Create bus manager
    pBusMgr_ = new BusManager;
    pCamera_ = new Camera;
    pGuid_   = new PGRGuid;

    // Call report() to get a list of available cameras
    report(stdout, 1);

    status = connectCamera();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        return;
    }

    startEvent_ = epicsEventCreate(epicsEventEmpty);

    /* launch image read task */
    epicsThreadCreate("PointGreyImageTask", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_imageTask, this);

    /* shutdown on exit */
    epicsAtExit(c_shutdown, this);

}

inline asynStatus pointGrey::checkError(Error error, const char *functionName, const char *PGRFunction)
{
    if (error != PGRERROR_OK) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling %s = %s\n",
            driverName, functionName, PGRFunction, error.GetDescription());
        return asynError;
    }
    return asynSuccess;
}

void pointGrey::report(FILE *fp, int details)
{
    unsigned int numCameras;
    Error error;
    Camera cam;
    CameraInfo camInfo;
    static const char *functionName = "report";
    
    error = pBusMgr_->GetNumOfCameras(&numCameras);
    if (checkError(error, functionName, "GetNumOfCameras")) return;

    fprintf(fp, "  Number of cameras detected: %u\n", numCameras);

    for (unsigned int i=0; i<numCameras; i++) {
        PGRGuid guid;
        error = pBusMgr_->GetCameraFromIndex(i, &guid);
        if (checkError(error, functionName, "GetCameraFromIndex")) return;

        // Connect to camera
        error = cam.Connect(&guid);
        if (checkError(error, functionName, "cam.Connect")) return;

        // Get the camera information
        error = cam.GetCameraInfo(&camInfo);
        if (checkError(error, functionName, "GetCameraInfo")) return;

        fprintf(fp, 
            "Serial number - %u\n"
            "Camera model - %s\n"
            "Camera vendor - %s\n"
            "Sensor - %s\n"
            "Resolution - %s\n"
            "Firmware version - %s\n"
            "Firmware build time - %s\n\n",
            camInfo.serialNumber,
            camInfo.modelName,
            camInfo.vendorName,
            camInfo.sensorInfo,
            camInfo.sensorResolution,
            camInfo.firmwareVersion,
            camInfo.firmwareBuildTime);
    }

    if (details < 1) return;
    ADDriver::report(fp, details);
}    

void pointGrey::shutdown(void)
{
    exiting_ = 1;
    if (pGuid_) {
        disconnectCamera();
    }
}

void pointGrey::imageTask()
{
    epicsTimeStamp imageStamp;
    Image *pPGImage = new(Image);
    Error error;
    unsigned int nRows, nCols, stride;
    PixelFormat pixelFormat;
    BayerTileFormat bayerFormat;
    int acquire;
    int total;
    int number;
    int count;
    int imageMode;
    int callback;
    NDArray *pImage;
    size_t dims[3];
    int pixelSize;
    size_t dataSize, dataSizePG;
    NDDataType_t dataType;
    NDColorMode_t colorMode;
    void *pData;
    int nDims;

    static const char *functionName = "imageTask";

    lock();
    while(!exiting_) {

        getIntegerParam(ADAcquire, &acquire);
        if (!acquire) {
            unlock();
            epicsEventWait(startEvent_);
            lock();
            error = pCamera_->StartCapture();
            if (checkError(error, functionName, "StartCapture")) continue;
        }
        
        unlock();
        error = pCamera_->RetrieveBuffer(pPGImage);
        lock();
        if (error == PGRERROR_ISOCH_NOT_STARTED) {
            // This is an expected error if acquisition was stopped externally
            setIntegerParam(ADAcquire, 0);
            callParamCallbacks();
        } else {
            if (checkError(error, functionName, "RetrieveBuffer")) continue;
        }
        epicsTimeGetCurrent(&imageStamp);

        getIntegerParam(ADNumImagesCounter, &number);
        number++;
        setIntegerParam(ADNumImagesCounter, number);
        getIntegerParam(NDArrayCounter, &count);
        count++;
        setIntegerParam(NDArrayCounter, count);
        callParamCallbacks();
        
        pPGImage->GetDimensions(&nRows, &nCols, &stride, &pixelFormat, &bayerFormat);

        getIntegerParam(NDArrayCallbacks, &callback);
        if (callback) {
            switch (pixelFormat) {
                case PIXEL_FORMAT_MONO8:
                    dataType = NDUInt8;
                    colorMode = NDColorModeMono;            
                    dims[0] = nCols;
                    dims[1] = nRows;
                    nDims = 2;
                    pixelSize = 1;
                    break;
                    
                case PIXEL_FORMAT_RGB8:
                    dataType = NDUInt8;
                    colorMode = NDColorModeRGB1;
                    dims[0] = 3;
                    dims[1] = nCols;
                    dims[2] = nRows;
                    nDims = 3;
                    pixelSize = 1;
                    break;

                case PIXEL_FORMAT_MONO16:
                    dataType = NDUInt16;
                    colorMode = NDColorModeMono;            
                    dims[0] = nCols;
                    dims[1] = nRows;
                    nDims = 2;
                    pixelSize = 2;
                    break;
                    
                case PIXEL_FORMAT_S_MONO16:
                    dataType = NDInt16;
                    colorMode = NDColorModeMono;            
                    dims[0] = nCols;
                    dims[1] = nRows;
                    nDims = 2;
                    pixelSize = 2;
                    break;
                    
                case PIXEL_FORMAT_RGB16:
                    dataType = NDUInt16;
                    colorMode = NDColorModeRGB1;
                    dims[0] = 3;
                    dims[1] = nCols;
                    dims[2] = nRows;
                    nDims = 3;
                    pixelSize = 2;
                    break;

                case PIXEL_FORMAT_S_RGB16:
                    dataType = NDInt16;
                    colorMode = NDColorModeRGB1;
                    dims[0] = 3;
                    dims[1] = nCols;
                    dims[2] = nRows;
                    nDims = 3;
                    pixelSize = 2;
                    break;
                    
                default:
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: unsupported pixel format=%d\n",
                        driverName, functionName, pixelFormat);
                    continue;
            }

            dataSize = dims[0] * dims[1] * pixelSize;
            if (nDims == 3) dataSize *= dims[2];
            dataSizePG = pPGImage->GetDataSize();
            if (dataSize != dataSizePG) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: data size mismatch: calculated=%u, reported=%u\n",
                    driverName, functionName, dataSize, dataSizePG);
                continue;
            }
            setIntegerParam(NDDataType,dataType);
            setIntegerParam(NDColorMode, colorMode);
            pImage = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
            pImage->uniqueId = count;
            pImage->timeStamp = imageStamp.secPastEpoch + (imageStamp.nsec / 1.0e9);
            updateTimeStamp(&pImage->epicsTS);
            pData = pPGImage->GetData();
            memcpy(pImage->pData, pData, dataSize);
            getAttributes(pImage->pAttributeList);

            unlock();
            doCallbacksGenericPointer(pImage, NDArrayData, 0);
            lock();

            pImage->release();
        }
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(ADNumImages, &total);
        if ((imageMode == ADImageSingle) ||
            ((imageMode == ADImageMultiple) && number == total)) {
            setShutter(0);
            error = pCamera_->StopCapture();
            checkError(error, functionName, "StopCapture");
            setIntegerParam(ADAcquire, 0);
        }
        callParamCallbacks();
    }
}

asynStatus pointGrey::readStatus()
{
    asynStatus status = asynSuccess;
    Property tempProp(TEMPERATURE);
    CameraStats camStats;
    Error error;
    static const char *functionName = "readStatus";

    error = pCamera_->GetProperty(&tempProp);
    if (checkError(error, functionName, "GetProperty")) 
        status = asynError;
    else if (tempProp.present)
        setDoubleParam(ADTemperatureActual, tempProp.absValue);
printf("%s:%s: temp=%f\n", driverName, functionName, tempProp.absValue);
    error = pCamera_->GetStats(&camStats);
    if (checkError(error, functionName, "GetStats")) 
        status = asynError;
    else
        setIntegerParam(PGImagesDropped_, camStats.imageDropped);
printf("%s:%s: dropped=%d\n", driverName, functionName, camStats.imageDropped);
    return status;
}

asynStatus pointGrey::setROI()
{
    asynStatus status=asynSuccess;
    int minX, sizeX, binX;
    int minY, sizeY, binY;
    static const char *functionName = "setAOI";

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
        "%s:%s: entry\n",
        driverName, functionName);

    getIntegerParam(ADSizeX, &sizeX);
    getIntegerParam(ADSizeY, &sizeY);
    getIntegerParam(ADBinX, &binX);
    getIntegerParam(ADBinY, &binY);
    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADMinY, &minY);
    return status;
}

asynStatus pointGrey::getROI()
{
    asynStatus status=asynSuccess;
    int minX, sizeX, binX;
    int minY, sizeY, binY;
    static const char *functionName = "getROI";

    setIntegerParam(ADBinX, binX);
    setIntegerParam(ADBinY, binY);
    setIntegerParam(ADSizeX, (int)(sizeX*binX));
    setIntegerParam(ADMinX,  (int)minX);
    setIntegerParam(ADSizeY, (int)(sizeY*binY));
    setIntegerParam(ADMinY,  (int)minY);

    /* Set NDArray parameters */
    setIntegerParam(NDArraySizeX, sizeX);
    setIntegerParam(NDArraySizeY, sizeY);
    callParamCallbacks();
    return status;
}

asynStatus pointGrey::connectCamera(void)
{
    asynStatus status = asynSuccess;
    Error error;
    CameraInfo camInfo;
    int sizeX, sizeY;
    FC2Version version;
    unsigned int numCameras;
    char tempString[sk_maxStringLength];
    static const char *functionName = "connectCamera";

    error = pBusMgr_->GetNumOfCameras(&numCameras);
    if (checkError(error, functionName, "GetNumOfCameras")) return asynError;
    
    if (numCameras <= 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: no cameras found\n",
            driverName, functionName);
    }

    if (cameraId_ == 0) {
        error = pBusMgr_->GetCameraFromIndex(0, pGuid_);
        if (checkError(error, functionName, "GetCameraFromIndex")) return asynError;
    } else { 
        error = pBusMgr_->GetCameraFromSerialNumber(cameraId_, pGuid_);
        if (checkError(error, functionName, "GetCameraFromSerialNumber")) return asynError;
    }
    // Connect to camera
    error = pCamera_->Connect(pGuid_);
    if (checkError(error, functionName, "Connect")) return asynError;

    // Get the camera information
    error = pCamera_->GetCameraInfo(&camInfo);
    if (checkError(error, functionName, "GetCameraInfo")) return asynError;
    setIntegerParam(PGSerialNumber_, camInfo.serialNumber);
    setStringParam(ADManufacturer, camInfo.vendorName);
    setStringParam(ADModel, camInfo.modelName);
    setStringParam(PGFirmwareVersion_, camInfo.firmwareVersion);
    
    Utilities::GetLibraryVersion(&version);
    sprintf(tempString, "%d.%d.%d", version.major, version.minor, version.type);
    setStringParam(PGSoftwareVersion_, tempString);
    sscanf(camInfo.sensorResolution, "%dx%d", &sizeX, &sizeY);
    setIntegerParam(ADMaxSizeX, sizeX);
    setIntegerParam(ADMaxSizeY, sizeY);
    return asynSuccess;
}

asynStatus pointGrey::disconnectCamera(void) 
{
    Error error;
    static const char *functionName = "disconnectCamera";

    if (pCamera_->IsConnected()) {
        error = pCamera_->Disconnect();
        if (checkError(error, functionName, "Disconnect")) return asynError;
    }
    return asynSuccess;
}

asynStatus pointGrey::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    asynStatus status;
    int function = pasynUser->reason;
    const char *paramName;
    Error error;
    static const char *functionName = "writeInt32";

    getParamName(function, &paramName);

    /* Set parameter from value */
    setIntegerParam(function, value);

    if (function == ADAcquire) {
        if (value) {
            setIntegerParam(ADNumImagesCounter, 0);
            setShutter(1);
            epicsEventSignal(startEvent_);
        } else {
            setShutter(0);
            error = pCamera_->StopCapture();
            if (checkError(error, functionName, "StopCapture")) status = asynError;
        }
    }

    else if (function == ADTriggerMode) {
        // Need to implement
    }
    else if (function == ADNumImages) {
        // Does the SDK support acquiring a fixed number of images?
    }
    else if 
       ((function == ADBinX)  ||
        (function == ADMinX)  ||
        (function == ADSizeX) ||
        (function == ADBinY)  ||
        (function == ADMinY)  ||
        (function == ADSizeY)) {
        status = setROI();
    }
    else if (function == ADReadStatus) {
        status = readStatus();
    }
    else {
        if (function < FIRST_PG_PARAM) {
            status = ADDriver::writeInt32(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: error, status=%d param=%s(%d) value=%d\n",
            driverName, functionName, status, paramName, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: param=%s(%d) value=%d\n",
            driverName, functionName, paramName, function, value);
    }
    return (asynStatus)status;
}

asynStatus pointGrey::setPropertyDouble(PropertyType propertyType, double value)
{
    Error error;
    Property property(propertyType);
    PropertyInfo propertyInfo(propertyType);
    static const char *functionName = "setPropertyDouble";
    
    error = pCamera_->GetPropertyInfo(&propertyInfo);
    if (!propertyInfo.present) 
        return asynSuccess;
    // For now we force the property On and Manual
    property.onOff = true;
    property.autoManualMode = false;
    property.absControl = true;
    property.absValue = (float)value;
    error = pCamera_->SetProperty(&property);
    if (checkError(error, functionName, "SetProperty")) 
        return asynError;
    return asynSuccess;
}

asynStatus pointGrey::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    asynStatus status = asynSuccess;
    const char *paramName;
    int function = pasynUser->reason;
    static const char *functionName = "writeFloat64";

    getParamName(function, &paramName);

    /* set parameter from value */
    setDoubleParam(function, value);

    if (function == ADTemperature) {
        status = setPropertyDouble(TEMPERATURE, value);
    }
    else if (function == ADAcquireTime) {
        // Point Grey units are ms
        status = setPropertyDouble(SHUTTER, value*1000.);
    }
    else if (function == ADAcquirePeriod) {
        double temp = value ? 1.0/value : 0;
        setDoubleParam(PGFrameRate_, temp);
        status = setPropertyDouble(FRAME_RATE, temp);
    }
    else if (function == ADGain) {
        status = setPropertyDouble(GAIN, value);
    }
    else if (function == PGFrameRate_) {
        setDoubleParam(ADAcquirePeriod, value ? 1./value : 0);
        status = setPropertyDouble(FRAME_RATE, value);
    }
    else {    
        if (function < FIRST_PG_PARAM) {
            status = ADDriver::writeFloat64(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: error, status=%d param=%s(%d) value=%f\n",
            driverName, functionName, status, paramName, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: param=%s(%d) value=%f\n",
            driverName, functionName, paramName, function, value);
    }
    return status;
}

/* Code for iocsh registration */
static const iocshArg pointGreyConfigArg0 = {"Port name", iocshArgString};
static const iocshArg pointGreyConfigArg1 = {"CameraId", iocshArgInt};
static const iocshArg pointGreyConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg pointGreyConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg pointGreyConfigArg4 = {"priority", iocshArgInt};
static const iocshArg pointGreyConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const pointGreyConfigArgs[] =  {&pointGreyConfigArg0,
                                                        &pointGreyConfigArg1,
                                                        &pointGreyConfigArg2,
                                                        &pointGreyConfigArg3,
                                                        &pointGreyConfigArg4,
                                                        &pointGreyConfigArg5};
static const iocshFuncDef configPointGrey = {"pointGreyConfig", 6, pointGreyConfigArgs};
static void configPointGreyCallFunc(const iocshArgBuf *args)
{
    pointGreyConfig(args[0].sval, args[1].ival, args[2].ival,  args[3].ival, 
                    args[4].ival, args[5].ival);
}

static void pointGreyRegister(void)
{

    iocshRegister(&configPointGrey, configPointGreyCallFunc);
}

extern "C" {
epicsExportRegistrar(pointGreyRegister);
}
