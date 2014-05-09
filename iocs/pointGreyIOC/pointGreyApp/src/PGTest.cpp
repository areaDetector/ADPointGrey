/*
 * PGTest.cpp
 *
 * Test program to try to track down problem on Linux where areaDetector driver appears to set parameters
 * in camera that cause it to fail the next time it is run.
 *
 * Author: Mark Rivers
 *         Univerity of Chicago
 *
 * Created: April 20, 2014
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#ifdef _WIN32
  #include <tchar.h>
  #include <windows.h>
#else
  #include <unistd.h>
#endif

#include <FlyCapture2.h>
using namespace FlyCapture2;

static const char *driverName = "PGTest";

#define MAX(x,y) ((x)>(y)?(x):(y))


// Point Grey does not define a NUM_PROPERTIES constant, but it can be set as follows
#define NUM_PROPERTIES UNSPECIFIED_PROPERTY_TYPE

#define NUM_GIGE_PROPERTIES 4

#define NUM_TRIGGER_MODES 17

// The maximum value of the asyn "addr" is the largest of NUM_PROPERTIES, NUM_PIXEL_FORMATS, NUM_MODES, NUM_VIDEO_MODES
#define MAX_ADDR NUM_MODES

// The maximum number of pins for strobe
#define NUM_GPIO_PINS 4

// The maximum number of binning modes
#define NUM_BINNING_MODES 3

// The maximum number of pixel formats to convert to when the input pixel format is PIXEL_FORMAT_RAW[8,12,16]
#define NUM_CONVERT_PIXEL_FORMATS 5

// Default packet delay in microseconds
#define DEFAULT_PACKET_DELAY 400

#define PGSuccess 0
#define PGError -1
#define PG_TRACE_ERROR 1
#define PG_TRACE_FLOW  2

#define PGTriggerInternal 0

typedef enum {
    propValueA,
    propValueB
} propValue_t;

static const PixelFormat pixelFormatValues[NUM_PIXEL_FORMATS] = {
    PIXEL_FORMAT_MONO8,
    PIXEL_FORMAT_411YUV8,
    PIXEL_FORMAT_422YUV8,
    PIXEL_FORMAT_444YUV8,
    PIXEL_FORMAT_RGB8,
    PIXEL_FORMAT_MONO16,
    PIXEL_FORMAT_RGB16,
    PIXEL_FORMAT_S_MONO16,
    PIXEL_FORMAT_S_RGB16,
    PIXEL_FORMAT_RAW8,
    PIXEL_FORMAT_RAW16,
    PIXEL_FORMAT_MONO12,
    PIXEL_FORMAT_RAW12,
    PIXEL_FORMAT_BGR,
    PIXEL_FORMAT_BGRU,
    PIXEL_FORMAT_RGB,
    PIXEL_FORMAT_RGBU,
    PIXEL_FORMAT_BGR16,
    PIXEL_FORMAT_BGRU16,
    PIXEL_FORMAT_422YUV8_JPEG
};

typedef enum {
    BINNING_1X1,
    BINNING_2X2,
    BINNING_4X4
} binningMode_t;

static const unsigned int binningModeValues[NUM_BINNING_MODES] = {
    1,
    2,
    4,
};

static const char *propertyTypeStrings[NUM_PROPERTIES] = {
    "Brightness",
    "AutoExposuure",
    "Sharpness",
    "WhiteBalance",
    "Hue",
    "Saturation",
    "Gamma",
    "Iris",
    "Focus",
    "Zoom",
    "Pan",
    "Tilt",
    "Shutter",
    "Gain",
    "TriggerMode",
    "TriggerDelay",
    "FrameRate",
    "Temperature"
};

static const char *gigEPropertyTypeStrings[NUM_GIGE_PROPERTIES] = {
    "Heartbeat",
    "HeartbeatTimeout",
    "PacketSize",
    "PacketDelay"
};


class PGTest
{
public:
    PGTest(int cameraId, int traceMask, int memoryChannel);

    int PGMinX;
    int PGMinY;
    int PGSizeX;
    int PGSizeY;
    int PGMaxSizeX;
    int PGMaxSizeY;
    int PGBinX;
    int PGBinY;
    int PGTriggerMode;
    int PGNumExposures;
    int PGNumImages;
    int PGNumImagesCounter;
    int PGAcquire;
    double PGAcquireTime;
    double PGAcquirePeriod;
    double PGGain;
    double PGTemperatureActual;
    int PGSerialNumber;           /** Camera serial number (int32 read) */
    int PGFirmwareVersion;        /** Camera firmware version                         (octet read) */
    int PGSoftwareVersion;        /** Camera software version                         (octet read) */
                                  /** The following PGProperty parameters 
                                      all have addr: 0-NUM_PROPERTIES-1 */
    int PGPropertyAvail[NUM_PROPERTIES];          /** Property is available                           (int32 read) */
    int PGPropertyAutoAvail[NUM_PROPERTIES];      /** Property auto mode available                    (int32 read) */
    int PGPropertyManAvail[NUM_PROPERTIES];       /** Property manual mode available                  (int32 read) */
    int PGPropertyAutoMode[NUM_PROPERTIES];       /** Property control mode: 0:manual or 1:automatic  (int32 read/write) */
    int PGPropertyAbsAvail[NUM_PROPERTIES];       /** Property has absolute (floating point) controls (int32 read) */
    int PGPropertyAbsMode[NUM_PROPERTIES];        /** Property raw/absolute mode: 0:raw or 1:absolute (int32 read/write) */
    int PGPropertyValue[NUM_PROPERTIES];          /** Property value                                  (int32 read/write) */
    int PGPropertyValueB[NUM_PROPERTIES];         /** Property value B                                (int32 read/write) */
    int PGPropertyValueMax[NUM_PROPERTIES];       /** Property maximum value                          (int32 read) */
    int PGPropertyValueMin[NUM_PROPERTIES];       /** Property minimum value                          (int32 read) */
    double PGPropertyValueAbs[NUM_PROPERTIES];       /** Property absolute value                         (float64 read/write) */
    double PGPropertyValueAbsMax[NUM_PROPERTIES];    /** Property absolute maximum value                 (float64 read) */
    double PGPropertyValueAbsMin[NUM_PROPERTIES];    /** Property absolute minimum value                 (float64 read) */
    int PGGigEPropertyValue[NUM_GIGE_PROPERTIES];      /** GigE property value                             (int32 read/write) */
    int PGGigEPropertyValueMax[NUM_GIGE_PROPERTIES];   /** GigE property maximum value                     (int32 read) */
    int PGGigEPropertyValueMin[NUM_GIGE_PROPERTIES];   /** GigE property minimum value                     (int32 read) */
    int PGVideoMode;              /** Video mode enum VideoMode, 0-NUM_VIDEOMODES-1   (int32 read/write) */
    int PGFormat7Mode;            /** Format7 mode enum Mode, 0-NUM_MODES-1           (int32 read/write)  */
    int PGBinningMode;            /** Binning enum Binning, 0-NUM_BINNINGS-1          (int32 read/write) */
    int PGFrameRate;              /** Frame rate enum FrameRate, 0-NUM_FRAMERATES-1   (int32 read/write) */
    int PGPixelFormat;            /** The pixel format when VideoFormat=Format7
                                      enum PixelFormat, 0-NUM_PIXEL_FORMATS-1         (int32 read/write) */
    int PGConvertPixelFormat;     /** The pixel format to convert to when input                               
                                      pixel format is raw[8,12,16] 
                                      enum PixelFormat, 0-NUM_PIXEL_FORMATS-1         (int32 read/write) */
    int PGTriggerSource;          /** Trigger source                                  (int32 write/read) */
    int PGTriggerPolarity;        /** Trigger polarity                                (int32 write/read) */
    int PGSoftwareTrigger;        /** Issue a software trigger                        (int32 write/read) */
    int PGSkipFrames;             /** Frames to skip in trigger mode 3                (int32 write/read) */
    int PGStrobeSource;           /** Strobe source GPIO pin                          (int32 write/read) */
    int PGStrobePolarity;         /** Strobe polarity (low/high)                      (int32 write/read) */
    int PGStrobeEnable;           /** Strobe enable/disable strobe                    (int32 write/read) */
    double PGStrobeDelay;            /** Strobe delay                                    (float64 write/read) */
    double PGStrobeDuration;         /** Strobe duration                                 (float64 write/read) */
    int PGPacketSize;             /** Size of data packets from camera                (int32 write/read) */
    int PGPacketSizeActual;       /** Size of data packets from camera                (int32 write/read) */
    int PGMaxPacketSize;          /** Maximum size of data packets from camera        (int32 write/read) */
    int PGPacketDelay;            /** Packet delay in usec from camera, GigE only     (int32 write/read) */
    int PGPacketDelayActual;      /** Packet delay in usec from camera, GigE only     (int32 read) */
    int PGBandwidth;              /** Bandwidth in MB/s                               (float64 read) */
    int PGTimeStampMode;          /** Time stamp mode (PGTimeStamp_t)                 (int32 write/read) */
    int PGHeartbeat;              /** Hearbeat, GigE only                             (int32 write/read) */
    int PGHeartbeatTimeout;       /** Heartbeat timeout, GigE only                    (int32 write/read) */
    int PGCorruptFrames;          /** Number of corrupt frames                        (int32 read) */
    int PGDriverDropped;          /** Number of driver dropped frames                 (int32 read) */
    int PGTransmitFailed;         /** Number of transmit failures                     (int32 read) */
    int PGDroppedFrames;          /** Number of dropped frames                        (int32 read) */
    #define LAST_PG_PARAM PGDroppedFrames

    /* Local methods to this class */
    int grabImage();
    int startCapture();
    int stopCapture();

    inline int checkError(Error error, const char *functionName, const char *message);
    int connectCamera();
    int disconnectCamera();
    int readStatus();

    /* camera property control functions */
    int setPropertyValue(PropertyType propType, int value, propValue_t valType);
    int setGigEPropertyValue(GigEPropertyType propType, int value);
    int setPropertyAbsValue(PropertyType propType, double value);
    int setPropertyAutoMode(PropertyType propType, int value);
    int setVideoMode(int mode);
    int setFrameRate(int rate);
    int setVideoModeAndFrameRate(int mode, int frameRate);
    int setImageParams();
    int setFormat7Params();
    int setGigEImageParams();
    int createStaticEnums();
    int createDynamicEnums();
    int getAllProperties();
    int getAllGigEProperties();
    int getPixelFormatIndex(PixelFormat pixelFormat);
    int setTrigger();
    int softwareTrigger();
    int setStrobe();
    int printDebug(int traceReason, const char *pFormat, ...);

    /* Data */
    int cameraId_;
    BusManager            *pBusMgr_;
    PGRGuid               *pGuid_;
    CameraBase            *pCameraBase_;
    Camera                *pCamera_;
    GigECamera            *pGigECamera_;
    CameraInfo            *pCameraInfo_;
    Format7Info           *pFormat7Info_;
    GigEImageSettingsInfo *pGigEImageSettingsInfo_;
    Image                 *pPGRawImage_;
    Image                 *pPGConvertedImage_;
    TriggerMode           *pTriggerMode_;
    TriggerModeInfo       *pTriggerModeInfo_;
    CameraStats           *pCameraStats_;
    StrobeControl         *pStrobeControl_;
    StrobeInfo            *pStrobeInfo_;
    Property              *allProperties_[NUM_PROPERTIES];
    PropertyInfo          *allPropInfos_ [NUM_PROPERTIES];
    GigEProperty          *allGigEProperties_[NUM_GIGE_PROPERTIES];
    int numValidVideoModes_;
    int numValidFormat7Modes_;
    int numValidBinningModes_;
    int numValidFrameRates_;
    int numValidPixelFormats_;
    int numValidConvertPixelFormats_;
    int numValidTriggerModes_;
    int numValidTriggerSources_;
    int numValidStrobeSources_;
    char *pImage_;
    int exiting_;
    int traceMask_;
};

/** Constructor for the PGTest class
 */
PGTest::PGTest(int cameraId, int traceMask, int memoryChannel)
    : cameraId_(cameraId), pGuid_(0), pImage_(0), exiting_(0), traceMask_(traceMask)
{
    static const char *functionName = "PGTest";
    int i;
    Error error;
    PropertyType propType;
    int status;
    
    /* Set initial values of some parameters */
    PGMinX = 0;
    PGMinY = 0;
    PGVideoMode = 0;
    PGFormat7Mode = 0;
    PGFrameRate = 0;
    PGTriggerSource = 0;
    PGStrobeSource = 1;
    PGBinningMode = 1;
    
    // Create camera control objects
    pBusMgr_            = new BusManager;
    pGuid_              = new PGRGuid;
    pCameraInfo_        = new CameraInfo;
    pFormat7Info_       = new Format7Info;
    pGigEImageSettingsInfo_ = new GigEImageSettingsInfo;
    pPGRawImage_        = new Image;
    pPGConvertedImage_  = new Image;
    pTriggerMode_       = new TriggerMode;
    pTriggerModeInfo_   = new TriggerModeInfo;
    pCameraStats_       = new CameraStats;
    pStrobeControl_     = new StrobeControl;
    pStrobeInfo_        = new StrobeInfo;

    status = connectCamera();
    if (status) {
        printDebug( PG_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        return;
    }
    
    if (memoryChannel > 0) {
        unsigned int numMemoryChannels;
        error = pCameraBase_->GetMemoryChannelInfo(&numMemoryChannels);
        printDebug( PG_TRACE_FLOW,
            "%s::%s called GetMemoryChannelInfo, pCameraBase_=%p, numMemoryChannels=%u\n",
            driverName, functionName, pCameraBase_, numMemoryChannels);
        if (memoryChannel > (int) numMemoryChannels) {
            printDebug( PG_TRACE_ERROR,
                "%s::%s memory channel=%d invalid, numMemoryChannels=%d\n", 
                driverName, functionName, memoryChannel, numMemoryChannels);
        } else {
            printDebug( PG_TRACE_FLOW,
                "%s::%s calling RestoreFromMemoryChannel, pCameraBase_=%p, memoryChannel=%d\n",
                driverName, functionName, pCameraBase_, memoryChannel-1);
            error = pCameraBase_->RestoreFromMemoryChannel(memoryChannel-1);
            checkError(error, functionName, "RestoreFromMemoryChannel");
        }
    }
    
    // Create an array of property objects, one for each property
    for (i=0; i<NUM_PROPERTIES; i++) {
        propType = (PropertyType) i;
        allProperties_[i] = new Property(propType);
        allPropInfos_[i]  = new PropertyInfo(propType);
    }
    getAllProperties();

    for (i=0; i<NUM_GIGE_PROPERTIES; i++) {
        allGigEProperties_[i] = new GigEProperty();
        allGigEProperties_[i]->propType = (GigEPropertyType) i;
    }
    getAllGigEProperties();

    createStaticEnums();
    createDynamicEnums();

    // Get and set maximum packet size and default packet delay for GigE cameras
    if (pGigECamera_) {
        unsigned int packetSize;
        error = pGigECamera_->DiscoverGigEPacketSize(&packetSize);
        if (checkError(error, functionName, "DiscoverGigEPacketSize")) {
            // There was an error getting the packet size, use 1440 as default;
            packetSize = 1440;
        }
        printDebug( PG_TRACE_FLOW,
            "%s::%s called DiscoverGigEPacketSize, pGigECamera_=%p, packetSize=%d\n",
            driverName, functionName, pGigECamera_, packetSize);
        PGMaxPacketSize = packetSize;
        PGPacketDelay = DEFAULT_PACKET_DELAY;
        setGigEPropertyValue(PACKET_SIZE, packetSize);
        setGigEPropertyValue(PACKET_DELAY, DEFAULT_PACKET_DELAY);
    }

    return;
}

inline int PGTest::checkError(Error error, const char *functionName, const char *PGRFunction)
{
    if (error != PGRERROR_OK) {
        printDebug( PG_TRACE_ERROR,
            "%s:%s: ERROR calling %s Type=%d Description=%s\n",
            driverName, functionName, PGRFunction, error.GetType(), error.GetDescription());
        return PGError;
    }
    return PGSuccess;
}

inline int PGTest::printDebug(int reason, const char *pformat, ...)
{
    FILE *fp = stdout;
    int nout;
    va_list  pvar;

    if (!(reason & traceMask_)) return 0;
    va_start(pvar,pformat);
    nout = vfprintf(fp,pformat,pvar);
    va_end(pvar);
    fflush(fp);
    return nout;
}

int PGTest::connectCamera(void)
{
    Error error;
    Format7Info f7Info;
    FC2Version version;
    EmbeddedImageInfo embeddedInfo;
    InterfaceType interfaceType;
    unsigned int numCameras;
    char tempString[sk_maxStringLength];
    static const char *functionName = "connectCamera";

    error = pBusMgr_->GetNumOfCameras(&numCameras);
    if (checkError(error, functionName, "GetNumOfCameras")) return PGError;
    printDebug( PG_TRACE_FLOW,
        "%s::%s called BusManager::GetNumOfCameras, pBusMgr_=%p, numCameras=%d\n",
        driverName, functionName, pBusMgr_, numCameras);
    
    if (numCameras <= 0) {
        printDebug( PG_TRACE_ERROR, 
            "%s:%s: no cameras found\n",
            driverName, functionName);
    }

    if (cameraId_ == 0) {
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling BusManager::GetCameraFromIndex, pBusMgr_=%p, pGuid_=%p\n",
            driverName, functionName, pBusMgr_, pGuid_);
        error = pBusMgr_->GetCameraFromIndex(0, pGuid_);
        if (checkError(error, functionName, "GetCameraFromIndex")) return PGError;
    } else { 
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling BusManager::GetCameraFromSerialNumber, pBusMgr_=%p, cameraId_=%d, pGuid_=%p\n",
            driverName, functionName, pBusMgr_, cameraId_, pGuid_);
        error = pBusMgr_->GetCameraFromSerialNumber(cameraId_, pGuid_);
        if (checkError(error, functionName, "GetCameraFromSerialNumber")) return PGError;
    }
    error = pBusMgr_->GetInterfaceTypeFromGuid(pGuid_, &interfaceType);
    if (checkError(error, functionName, "GetInterfaceTypeFromGuid")) return PGError;
    printDebug( PG_TRACE_FLOW,
        "%s::%s called BusManager::GetInterfaceTypeFromGuid, pBusMgr_=%p, interfaceType=%d\n",
        driverName, functionName, pBusMgr_, interfaceType);
    
    // Create appropriate camera object
    if (interfaceType == INTERFACE_GIGE) {
        pCameraBase_ = new GigECamera;
        pCamera_ = NULL;
        pGigECamera_ = dynamic_cast<GigECamera*>(pCameraBase_);
        if (pGigECamera_ == NULL) {
            printDebug( PG_TRACE_ERROR,
                "%s::%s error casting camera to GigECamera\n",
                driverName, functionName);
            return PGError;
        }
    } else {
        pCameraBase_ = new Camera;
        pGigECamera_ = NULL;
        pCamera_ = dynamic_cast<Camera*>(pCameraBase_);
        if (pCamera_ == NULL) {
            printDebug( PG_TRACE_ERROR,
                "%s::%s error casting camera to Camera\n",
                driverName, functionName);
            return PGError;
        }   
    }

    // Connect to camera
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::Connect, pGuid_=%p\n",
        driverName, functionName, pGuid_);
    error = pCameraBase_->Connect(pGuid_);
    if (checkError(error, functionName, "Connect")) return PGError;

    // Get the camera information
    error = pCameraBase_->GetCameraInfo(pCameraInfo_);
    if (checkError(error, functionName, "GetCameraInfo")) return PGError;
    printDebug( PG_TRACE_FLOW,
        "%s::%s called CameraBase::GetCameraInfo, pCameraInfo_=%p, pCameraInfo_->serialNumber=%d\n",
        driverName, functionName, pCameraInfo_, pCameraInfo_->serialNumber);
    
    Utilities::GetLibraryVersion(&version);
    sprintf(tempString, "%d.%d.%d", version.major, version.minor, version.type);
    printDebug( PG_TRACE_FLOW,
        "%s::%s called Utilities::GetLibraryVersion, version=%s\n",
        driverName, functionName, tempString);
    
    // Get and set the embedded image info
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::GetEmbeddedImageInfo, &embeddedInfo=%p\n",
        driverName, functionName, &embeddedInfo);
    error = pCameraBase_->GetEmbeddedImageInfo(&embeddedInfo);
    if (checkError(error, functionName, "GetEmbeddedImageInfo")) return PGError;
    // Force the timestamp and frame counter information to be on
    embeddedInfo.timestamp.onOff = true;
    embeddedInfo.frameCounter.onOff = true;
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::SetEmbeddedImageInfo, &embeddedInfo=%p\n",
        driverName, functionName, &embeddedInfo);
    error = pCameraBase_->SetEmbeddedImageInfo(&embeddedInfo);
    if (checkError(error, functionName, "SetEmbeddedImageInfo")) return PGError;
    
    return PGSuccess;
}

int PGTest::disconnectCamera(void) 
{
    Error error;
    static const char *functionName = "disconnectCamera";

    if (pCameraBase_->IsConnected()) {
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling CameraBase::Disconnect, pCameraBase_=%p\n",
            driverName, functionName, pCameraBase_);
        error = pCameraBase_->Disconnect();
        if (checkError(error, functionName, "Disconnect")) return PGError;
    }
    printf("%s:%s disconnect camera OK\n", driverName, functionName);
    return PGSuccess;
}


/** Grabs one image off the dc1394 queue, notifies areaDetector about it and
 * finally clears the buffer off the dc1394 queue.
 * This function expects the driver to be locked already by the caller!
 */
int PGTest::grabImage()
{
    int status = PGSuccess;
    Error error;
    unsigned int nRows, nCols, stride;
    PixelFormat pixelFormat;
    BayerTileFormat bayerFormat;
    Image *pPGImage;
    ImageMetadata metaData;
    TimeStamp timeStamp;
    size_t dataSizePG;
    void *pData;
    static const char *functionName = "grabImage";

    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::RetrieveBuffer, pCameraBase_=%p, pPGRawImage_=%p\n",
        driverName, functionName, pCameraBase_, pPGRawImage_);
    error = pCameraBase_->RetrieveBuffer(pPGRawImage_);
    if (error != PGRERROR_OK) {
        if (error == PGRERROR_ISOCH_NOT_STARTED) {
            // This is an expected error if acquisition was stopped externally
            return PGError;
        } 
        checkError(error, functionName, "RetrieveBuffer");
        if (error == PGRERROR_IMAGE_CONSISTENCY_ERROR) {
            // For now we ignore this error
        } else {
            return PGError;
        }
    }
    pPGRawImage_->GetDimensions(&nRows, &nCols, &stride, &pixelFormat, &bayerFormat);    
    metaData = pPGRawImage_->GetMetadata();    
    timeStamp = pPGRawImage_->GetTimeStamp();    
    pPGImage = pPGRawImage_;
    // Calculate bandwidth
    dataSizePG = pPGRawImage_->GetReceivedDataSize();

    pPGImage->GetDimensions(&nRows, &nCols, &stride, &pixelFormat, &bayerFormat);
    dataSizePG = pPGImage->GetDataSize();

    if (pImage_) free(pImage_);
    pImage_ = (char *)malloc(dataSizePG);
    pData = pPGImage->GetData();
    memcpy(pImage_, pData, dataSizePG);

    return status;
}


int PGTest::setPropertyAutoMode(PropertyType propType, int value)
{
    Error error;
    Property *pProperty = allProperties_[propType];
    PropertyInfo *pPropInfo = allPropInfos_[propType];
    static const char *functionName = "setPropertyAutoMode";
    
    /* First check if the propertyType is valid for this camera */
    if (!pProperty->present) return PGError;

    /* Check if the desired mode is even supported by the camera on this propertyType */
    if (value == 0) {
        if (!pPropInfo->manualSupported) return PGError;
    } else {
        if (!pPropInfo->autoSupported) return PGError;
    }

    /* Send the propertyType mode to the cam */
    pProperty->autoManualMode = value ? true : false;
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::SetProperty, pCameraBase_=%p, pProperty=%p, pProperty->autoManualMode=%d\n",
        driverName, functionName, pCameraBase_, pProperty, pProperty->autoManualMode);
    error = pCameraBase_->SetProperty(pProperty);
    if (checkError(error, functionName, "SetProperty")) 
        return PGError;
    return PGSuccess;
}


int PGTest::setPropertyValue(PropertyType propType, int value, propValue_t valType)
{
    Error error;
    Property *pProperty = allProperties_[propType];
    PropertyInfo *pPropInfo = allPropInfos_[propType];
    static const char *functionName = "setPropertyValue";

    /* First check if the propertyType is valid for this camera */
    if (!pProperty->present) return PGError;

    /* Enable control for this propertyType */
    pProperty->onOff = true;

    /* Disable absolute mode control for this propertyType */
    pProperty->absControl = false;

    /* Disable automatic mode control for this propertyType */
    pProperty->autoManualMode = false;

    /* Check the value is within the expected boundaries */
    if (value < (int)pPropInfo->min || value > (int)pPropInfo->max) {
        printDebug( PG_TRACE_ERROR, 
            "%s::%s error setting propertyType %s, value %d is out of range [%d..%d]\n",
            driverName, functionName, propertyTypeStrings[propType], value, pPropInfo->min, pPropInfo->max);
        return PGError;
    }

    if (valType == propValueA) {
        pProperty->valueA = value;
    } else {
        pProperty->valueB = value;
    }
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::SetProperty, pCameraBase_=%p, pProperty=%p, pProperty->type=%d, pProperty->valueA=%d\n",
        driverName, functionName, pCameraBase_, pProperty, pProperty->type, pProperty->valueA);
    error = pCameraBase_->SetProperty(pProperty);
    if (checkError(error, functionName, "SetProperty")) 
        return PGError;

    /* Update all properties to see if any settings have changed */
    getAllProperties();
    return PGSuccess;
}


int PGTest::setGigEPropertyValue(GigEPropertyType propType, int value)
{
    Error error;
    GigEProperty *pProperty = allGigEProperties_[propType];
    static const char *functionName = "setGigEPropertyValue";

    if (pGigECamera_ == NULL) return PGSuccess;
    
    /* First check if the propertyType is writeable for this camera */
    if (!pProperty->isWritable) return PGError;

    /* Check the value is within the expected boundaries */
    if (value < (int)pProperty->min || value > (int)pProperty->max) {
        printDebug( PG_TRACE_ERROR, 
            "%s::%s error setting propertyType %s, value %d is out of range [%d..%d]\n",
            driverName, functionName, gigEPropertyTypeStrings[propType], value, pProperty->min, pProperty->max);
        return PGError;
    }

    pProperty->value = value;
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling GigECamera::SetGigEProperty, pGigECamera_=%p, pProperty=%p, pProperty->propType=%d, pProperty->value=%d\n",
        driverName, functionName, pGigECamera_, pProperty, pProperty->propType, pProperty->value);
    error = pGigECamera_->SetGigEProperty(pProperty);
    if (checkError(error, functionName, "SetGigeEProperty")) 
        return PGError;

    /* Update all properties to see if any settings have changed */
    getAllProperties();
    getAllGigEProperties();
    return PGSuccess;
}


int PGTest::setPropertyAbsValue(PropertyType propType, double value)
{
    Error error;
    Property *pProperty = allProperties_[propType];
    PropertyInfo *pPropInfo = allPropInfos_[propType];
    static const char *functionName = "setPropertyAbsValue";

    /* First check if the propertyType is valid for this camera */
    if (!pProperty->present) {
        printDebug( PG_TRACE_ERROR, 
            "%s::%s error setting propertyType %s: property is not supported by this camera\n",
            driverName, functionName, propertyTypeStrings[propType]);
        return PGError;
    }

    /* Disable automatic mode control for this propertyType */
    pProperty->autoManualMode = false;

   /* Check if the specific propertyType supports absolute values */
    if (!pPropInfo->absValSupported) { 
        printDebug( PG_TRACE_ERROR, 
            "%s::%s error setting propertyType %s: No absolute control for this propertyType\n",
            driverName, functionName, propertyTypeStrings[propType]);
        return PGError;
    }

    /* Enable control for this propertyType */
    pProperty->onOff = true;

     /* Enable absolute mode control for this propertyType */
    pProperty->absControl = true;

    /* Check the value is within the expected boundaries */
    if (value < pPropInfo->absMin || value > pPropInfo->absMax) {
        printDebug( PG_TRACE_ERROR, 
            "%s::%s error setting propertyType %s, value %.5f is out of range [%.3f..%.3f]\n",
            driverName, functionName, propertyTypeStrings[propType], value, pPropInfo->absMin, pPropInfo->absMax);
        return PGError;
    }

    /* Set the propertyType value in the camera */
    pProperty->absValue = (float)value;
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::SetProperty, pCameraBase_=%p, pProperty=%p, pProperty->type=%d, pProperty->absValue=%f\n",
        driverName, functionName, pCameraBase_, pProperty, pProperty->type, pProperty->absValue);
    error = pCameraBase_->SetProperty(pProperty);
    if (checkError(error, functionName, "SetProperty")) {
        printDebug( PG_TRACE_ERROR, 
            "%s::%s error setting propertyType %s, value %.5f\n",
            driverName, functionName, propertyTypeStrings[propType], value);      
        return PGError;
    }

    /* Update all properties to see if any settings have changed */
    getAllProperties();
    return PGSuccess;
}

int PGTest::setVideoMode(int mode)
{
    int frameRate;
    /* Get the frame rate */
    frameRate = PGFrameRate;
    return setVideoModeAndFrameRate(mode, frameRate);
}
 
int PGTest::setFrameRate(int frameRate)
{
    int videoMode;
    /* Get the video mode */
    videoMode = PGVideoMode;
    return setVideoModeAndFrameRate(videoMode, frameRate);
}
 
int PGTest::setVideoModeAndFrameRate(int videoModeIn, int frameRateIn)
{
    int status = PGSuccess;
    Error error;
    bool resumeAcquire;
    FrameRate frameRate = (FrameRate)frameRateIn;
    VideoMode videoMode = (VideoMode)videoModeIn;
    bool supported;
    static const char *functionName = "setVideoModeAndFrameRate";

    if (pCamera_ == NULL) return PGError;

    // VIDEOMODE_FORMAT7 must be treated differently
    if (videoMode == VIDEOMODE_FORMAT7) {
        setImageParams();
    } else {
        /* Must stop capture before changing the video mode */
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling Camera::StopCapture, pCamera_=%p\n",
            driverName, functionName, pCamera_);
        error = pCamera_->StopCapture();
        resumeAcquire = (error == PGRERROR_OK);
        /* Attempt to write the video mode to camera */
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling Camera::SetVideoModeAndFrameRate, pCamera_=%p, videoMode=%d, frameRate=%d\n",
            driverName, functionName, pCamera_, videoMode, frameRate);
        error = pCamera_->SetVideoModeAndFrameRate(videoMode, frameRate);
        checkError(error, functionName, "SetVideoModeAndFrameRate");
        if (resumeAcquire) {
            printDebug( PG_TRACE_FLOW,
                "%s::%s calling Camera::StartCapture, pCamera_=%p\n",
                driverName, functionName, pCamera_);
            error = pCamera_->StartCapture();
            checkError(error, functionName, "StartCapture");
        }
        error = pCamera_->GetVideoModeAndFrameRateInfo(videoMode, frameRate, &supported);
        if (checkError(error, functionName, "GetVideoModeAndFrameRateInfo")) 
            return PGError;
        printDebug( PG_TRACE_FLOW,
            "%s::%s called Camera::GetVideoModeAndFrameRateInfo, pCamera_=%p, videoMode=%d, frameRate=%d, supported=%d\n",
            driverName, functionName, pCamera_, videoMode, frameRate, supported);
        if (!supported) {
             printDebug( PG_TRACE_ERROR, 
                "%s::%s camera does not support mode %d with framerate %d\n",
                driverName, functionName, videoMode, frameRate);
            status = PGError;
            return PGError;
        }
        /* When the video mode changes the supported values of frame rate change */
        createDynamicEnums();
        /* When the video mode changes the available properties can also change */
        getAllProperties();
    }

    return status;
}

int PGTest::getPixelFormatIndex(PixelFormat pixelFormat)
{
    int i;
    // Find the pixel format index corresponding to the actual pixelFormat
    for(i=0; i<(int)NUM_PIXEL_FORMATS; i++) {
        if (pixelFormatValues[i] == pixelFormat) return i;
    }
    return -1;
}

int PGTest::setImageParams()
{
    static const char *functionName = "setImageParams";
    
    if (pCamera_) 
        return setFormat7Params();
    
    else if (pGigECamera_) 
        return setGigEImageParams();
    
    else
        printDebug( PG_TRACE_ERROR, 
           "%s::%s unknown camera type\n",
           driverName, functionName);
    return PGError;
}


int PGTest::setFormat7Params()
{
    Error error;
    int videoMode;
    Format7ImageSettings f7Settings;
    Format7PacketInfo f7PacketInfo;
    PixelFormat pixelFormat;
    bool supported;
    bool f7SettingsValid;
    bool resumeAcquire;
    int f7Mode;
    float percentage;
    unsigned int packetSize;
    unsigned int packetSizeActual;
    int sizeX, sizeY, minX, minY;
    unsigned short hsMax, vsMax, hsUnit, vsUnit;
    unsigned short hpMax, vpMax, hpUnit, vpUnit;
    static const char *functionName = "setFormat7Params";

    if (!pCamera_) return PGError;
    
    /* Get the current video mode from EPICS.  It may have just been changed */
    videoMode = PGVideoMode;
    /* If not format 7 then silently exit */
    if (videoMode != VIDEOMODE_FORMAT7) return PGSuccess;
    
    f7Mode = PGFormat7Mode;
    /* Get the format7 info */
    pFormat7Info_->mode = (Mode)f7Mode;
    error = pCamera_->GetFormat7Info(pFormat7Info_, &supported);
    if (checkError(error, functionName, "GetFormat7Info")) 
        return PGError;
    printDebug( PG_TRACE_FLOW,
        "%s::%s called Camera::GetFormat7Info, pCamera_=%p, pFormat7Info_=%p, supported=%d\n",
        driverName, functionName, pCamera_, pFormat7Info_, supported);
    if (!supported) {
         printDebug( PG_TRACE_ERROR, 
            "%s::%s camera does not support mode Format7 mode %d\n",
            driverName, functionName, f7Mode);
       return PGError;
    }
    
    sizeX = PGSizeX;
    sizeY = PGSizeY;
    minX = PGMinX;
    minY = PGMinY;
    pixelFormat = (PixelFormat)PGPixelFormat;

    /* Set the size limits */
    hsMax = pFormat7Info_->maxWidth;
    vsMax = pFormat7Info_->maxHeight;
    PGMaxSizeX = hsMax;
    PGMaxSizeY = vsMax;
    /* Set the size units (minimum increment) */
    hsUnit = pFormat7Info_->imageHStepSize;
    vsUnit = pFormat7Info_->imageVStepSize;
    /* Set the offset units (minimum increment) */
    hpUnit = pFormat7Info_->offsetHStepSize;
    vpUnit = pFormat7Info_->offsetVStepSize;
    
    // This logic probably needs work!!!
    hpMax = hsMax;
    vpMax = vsMax;
 
    printDebug( PG_TRACE_FLOW, 
        "%s:%s hsMax=%d, vsMax=%d, hsUnit=%d, vsUnit=%d, hpMax=%d, vpMax=%d, hpUnit=%d, vpUnit=%d\n", 
        driverName, functionName, hsMax, vsMax, hsUnit, vsUnit,  hpMax, vpMax, hpUnit, vpUnit);

    /* Force the requested values to obey the increment and range */
    if (sizeX % hsUnit) sizeX = (sizeX/hsUnit) * hsUnit;
    if (sizeY % vsUnit) sizeY = (sizeY/vsUnit) * vsUnit;
    if (minX % hpUnit)  minX  = (minX/hpUnit)  * hpUnit;
    if (minY % vpUnit)  minY  = (minY/vpUnit)  * vpUnit;
    
    if (sizeX < hsUnit) sizeX = hsUnit;
    if (sizeX > hsMax)  sizeX = hsMax;
    if (sizeY < vsUnit) sizeY = vsUnit;
    if (sizeY > vsMax)  sizeY = vsMax;
    
    if (minX < 0) minX = 0;
    if (minX > hpMax)  minX = hpMax;
    if (minY < 0) minY = 0;
    if (minY > vpMax)  minY = vpMax;
    
    f7Settings.mode    = (Mode)f7Mode;
    f7Settings.offsetX = minX;
    f7Settings.offsetY = minY;
    f7Settings.width   = sizeX;
    f7Settings.height  = sizeY;
    f7Settings.pixelFormat = pixelFormat;
 
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling Camera::ValidateFormat7Settings, pCamera_=%p, &f7Settings=%p, &f7SettingsValid=%p, &f7PacketInfo=%p\n",
        driverName, functionName, pCamera_, &f7Settings, &f7SettingsValid, &f7PacketInfo);
    error = pCamera_->ValidateFormat7Settings(&f7Settings, &f7SettingsValid, &f7PacketInfo);
    if (checkError(error, functionName, "ValidateFormat7Settings")) 
        return PGError;

    /* Attempt to write the parameters to camera */
    printDebug( PG_TRACE_FLOW, 
        "%s::%s setting format 7 parameters sizeX=%d, sizeY=%d, minX=%d, minY=%d, pixelFormat=0x%x\n",
        driverName, functionName, sizeX, sizeY, minX, minY, pixelFormat);

    printDebug( PG_TRACE_FLOW,
        "%s:%s bytes per packet: min=%d, max=%d, recommended=%d, actually setting=%d\n", 
        driverName, functionName, f7PacketInfo.unitBytesPerPacket, f7PacketInfo.maxBytesPerPacket, 
        f7PacketInfo.recommendedBytesPerPacket, f7PacketInfo.recommendedBytesPerPacket);

    /* Must stop acquisition before changing the video mode */
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling Camera::StopCapture, pCamera_=%p\n",
        driverName, functionName, pCamera_);
    error = pCamera_->StopCapture();
    resumeAcquire = (error == PGRERROR_OK);
    packetSize = PGPacketSize;
    PGMaxPacketSize = pFormat7Info_->maxPacketSize;
    if (packetSize <= 0) {
        packetSize = f7PacketInfo.recommendedBytesPerPacket;
    } else {
        // Packet size must be a multiple of unitBytesPerPacket
        packetSize = (packetSize/f7PacketInfo.unitBytesPerPacket) * f7PacketInfo.unitBytesPerPacket;
        if (packetSize < pFormat7Info_->minPacketSize) packetSize = pFormat7Info_->minPacketSize;
        if (packetSize > pFormat7Info_->maxPacketSize) packetSize = pFormat7Info_->maxPacketSize;
    }
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling Camera::SetFormat7Configuration, pCamera_=%p, &f7Settings=%p, packetSize=%d\n",
        driverName, functionName, pCamera_, &f7Settings, packetSize);
    error = pCamera_->SetFormat7Configuration(&f7Settings, packetSize);
    checkError(error, functionName, "SetFormat7Configuration");
    if (resumeAcquire) {
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling Camera::StartCapture, pCamera_=%p\n",
            driverName, functionName, pCamera_);
        error = pCamera_->StartCapture();
        checkError(error, functionName, "StartCapture");
    }

    /* Read back the actual values */
    error = pCamera_->GetFormat7Configuration(&f7Settings, &packetSizeActual, &percentage);
    if (checkError(error, functionName, "GetFormat7Configuration")) 
        return PGError;
    printDebug( PG_TRACE_FLOW,
        "%s::%s called Camera::GetFormat7Configuration, pCamera_=%p, &f7Settings=%p, packetSizeActual=%d, percentage=%f\n",
        driverName, functionName, pCamera_, &f7Settings, packetSizeActual, percentage);
    PGMinX = f7Settings.offsetX;
    PGMinY = f7Settings.offsetY;
    PGSizeX = f7Settings.width;
    PGSizeY = f7Settings.height;
    PGPixelFormat = f7Settings.pixelFormat;
    PGPacketSizeActual = packetSizeActual;
    
    /* When the format7 mode changes the supported values of pixel format changes */
    createDynamicEnums();
    /* When the format7 mode changes the available properties can also change */
    getAllProperties();

    return PGSuccess;
}

int PGTest::setGigEImageParams()
{
    Error error;
    GigEImageSettings gigESettings;
    PixelFormat pixelFormat;
    bool resumeAcquire;
    int gigEMode;
    int minPacketSize, maxPacketSize;
    int minPacketDelay, maxPacketDelay;
    int packetSize;
    int packetDelay;
    int sizeX, sizeY, minX, minY;
    unsigned int binX, binY;
    int hsMax, vsMax, hsUnit, vsUnit;
    int hpMax, vpMax, hpUnit, vpUnit;
    int binningMode;
    static const char *functionName = "setGigEImageParams";

    if (!pGigECamera_) return PGError;

    /* Must stop acquisition before changing these settings */
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling GigECamera::StopCapture, pGigECamera_=%p\n",
        driverName, functionName, pGigECamera_);
    error = pGigECamera_->StopCapture();
    resumeAcquire = (error == PGRERROR_OK);

    // Set the GigE imaging mode    
    gigEMode = PGFormat7Mode;
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling GigECamera::SetGigEImagingMode, pGigECamera_=%p, gigEMode=%d\n",
        driverName, functionName, pGigECamera_, gigEMode);
    error = pGigECamera_->SetGigEImagingMode((Mode)gigEMode);
    if (checkError(error, functionName, "SetGigEImagingMode")) 
        goto cleanup;

    /* Get the GigE image settings info */
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling GigECamera::GetGigEImageSettingsInfo, pGigECamera_=%p, pGigEImageSettingsInfo_=%p\n",
        driverName, functionName, pGigECamera_, pGigEImageSettingsInfo_);
    error = pGigECamera_->GetGigEImageSettingsInfo(pGigEImageSettingsInfo_);
    if (checkError(error, functionName, "GetGigEImageSettingsInfo")) 
        goto cleanup;
    
    sizeX = PGSizeX;
    sizeY = PGSizeY;
    minX = PGMinX;
    minY = PGMinY;
    pixelFormat = (PixelFormat)PGPixelFormat;

    /* Set the size limits */
    hsMax = pGigEImageSettingsInfo_->maxWidth;
    vsMax = pGigEImageSettingsInfo_->maxHeight;
    PGMaxSizeX = hsMax;
    PGMaxSizeY = vsMax;
    /* Set the size units (minimum increment) */
    hsUnit = pGigEImageSettingsInfo_->imageHStepSize;
    vsUnit = pGigEImageSettingsInfo_->imageVStepSize;
    /* Set the offset units (minimum increment) */
    hpUnit = pGigEImageSettingsInfo_->offsetHStepSize;
    vpUnit = pGigEImageSettingsInfo_->offsetVStepSize;
    
    // This logic probably needs work!!!
    hpMax = hsMax;
    vpMax = vsMax;
 
    printDebug( PG_TRACE_FLOW, 
        "%s:%s hsMax=%d, vsMax=%d, hsUnit=%d, vsUnit=%d, hpMax=%d, vpMax=%d, hpUnit=%d, vpUnit=%d\n", 
        driverName, functionName, hsMax, vsMax, hsUnit, vsUnit,  hpMax, vpMax, hpUnit, vpUnit);

    /* Force the requested values to obey the increment and range */
    if (sizeX % hsUnit) sizeX = (sizeX/hsUnit) * hsUnit;
    if (sizeY % vsUnit) sizeY = (sizeY/vsUnit) * vsUnit;
    if (minX % hpUnit)  minX  = (minX/hpUnit)  * hpUnit;
    if (minY % vpUnit)  minY  = (minY/vpUnit)  * vpUnit;
    
    if (sizeX < hsUnit) sizeX = hsUnit;
    if (sizeX > hsMax)  sizeX = hsMax;
    if (sizeY < vsUnit) sizeY = vsUnit;
    if (sizeY > vsMax)  sizeY = vsMax;
    
    if (minX < 0) minX = 0;
    if (minX > hpMax)  minX = hpMax;
    if (minY < 0) minY = 0;
    if (minY > vpMax)  minY = vpMax;
    
    gigESettings.offsetX = minX;
    gigESettings.offsetY = minY;
    gigESettings.width   = sizeX;
    gigESettings.height  = sizeY;
    gigESettings.pixelFormat = pixelFormat;
    memset(gigESettings.reserved, 0, 8*sizeof(unsigned int));
 
    /* Attempt to write the parameters to camera */
    printDebug( PG_TRACE_FLOW, 
        "%s::%s setting GigE parameters width=%d, height=%d, offsetX=%d, offsetY=%d, pixelFormat=0x%x\n",
        driverName, functionName, 
        gigESettings.width, gigESettings.height, 
        gigESettings.offsetX, gigESettings.offsetY, gigESettings.pixelFormat);

    printDebug( PG_TRACE_FLOW,
        "%s::%s calling GigECamera::SetGigEImageSettings, pGigECamera_=%p, &gigESettings=%p\n",
        driverName, functionName, pGigECamera_, &gigESettings);
    error = pGigECamera_->SetGigEImageSettings(&gigESettings);
    if (checkError(error, functionName, "SetGigEImageSettings"))
        goto cleanup;

    // Set the packet size and delay
    minPacketSize = PGGigEPropertyValueMin[PACKET_SIZE];
    maxPacketSize = PGMaxPacketSize;
    packetSize = PGPacketSize;
    if (packetSize <= 0) packetSize = maxPacketSize;
    if (packetSize < minPacketSize) packetSize = minPacketSize;
    if (packetSize > maxPacketSize) packetSize = maxPacketSize;
    setGigEPropertyValue(PACKET_SIZE, packetSize);
    PGPacketSizeActual = packetSize;

    packetDelay = PGPacketDelay;
    minPacketDelay = PGGigEPropertyValueMin[PACKET_DELAY];
    maxPacketDelay = PGGigEPropertyValueMax[PACKET_DELAY];
    if (packetDelay < minPacketDelay) packetDelay = minPacketDelay;
    if (packetDelay > maxPacketDelay) packetDelay = maxPacketDelay;
    setGigEPropertyValue(PACKET_DELAY, packetDelay);

    // Set the binning
    binningMode = PGBinningMode;
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling GigECamera::SetGigEImageBinningSettings, pGigECamera_=%p, binX=%d, binY=%d\n",
        driverName, functionName, pGigECamera_, binningMode, binningMode);
    error = pGigECamera_->SetGigEImageBinningSettings(binningMode, 
                                                      binningMode);
    if (checkError(error, functionName, "SetGigEImageBinningSettings")) 
        goto cleanup;

    /* Read back the actual values */
    error = pGigECamera_->GetGigEImagingMode((Mode *)&gigEMode);
    if (checkError(error, functionName, "GetGigEImagingMode")) 
        goto cleanup;
    printDebug( PG_TRACE_FLOW,
        "%s::%s called GigECamera::GetGigEImagingMode, pGigECamera_=%p, gigEMode=%d\n",
        driverName, functionName, pGigECamera_, gigEMode);
    PGFormat7Mode = gigEMode;

    error = pGigECamera_->GetGigEImageBinningSettings(&binX, &binY); 
    if (checkError(error, functionName, "GetGigEImageBinningSettings")) 
        goto cleanup;
    printDebug( PG_TRACE_FLOW,
        "%s::%s called GigECamera::GetGigEImageBinningSettings, pGigECamera_=%p, binX=%d, binY=%d\n",
        driverName, functionName, pGigECamera_, binX, binY);
    PGBinningMode = binX;
    PGBinX = binX;
    PGBinY = binY;

    printDebug( PG_TRACE_FLOW,
        "%s::%s calling GigECamera::GetGigEImageSettings, pGigECamera_=%p, &gigESettings=%p\n",
        driverName, functionName, pGigECamera_, &gigESettings);
    error = pGigECamera_->GetGigEImageSettings(&gigESettings);
    if (checkError(error, functionName, "GetGigEImageSettings")) 
        goto cleanup;

    PGMinX = gigESettings.offsetX;
    PGMinY = gigESettings.offsetY;
    PGSizeX = gigESettings.width;
    PGSizeY = gigESettings.height;
    PGPixelFormat = gigESettings.pixelFormat;
    
    /* When the GigE mode changes the supported values of pixel format changes */
    createDynamicEnums();
    /* When the GigE mode changes the available properties can also change */
    getAllProperties();
    getAllGigEProperties();

cleanup:
    if (resumeAcquire) {
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling GigECamera::StartCapture, pGigECamera_=%p\n",
            driverName, functionName, pGigECamera_);
        error = pGigECamera_->StartCapture();
        checkError(error, functionName, "StartCapture");
    }

    if (error != PGRERROR_OK) return PGError;
    return PGSuccess;
}


int PGTest::setTrigger()
{
    int numImages;
    int numExposures;
    int triggerMode;
    int triggerPolarity;
    int triggerSource;
    int skipFrames;
    Error error;
    static const char *functionName = "setTrigger";
    
    triggerMode = PGTriggerMode;
    triggerPolarity = PGTriggerPolarity;
    triggerSource = PGTriggerSource;
    error = pCameraBase_->GetTriggerMode(pTriggerMode_);
    if (checkError(error, functionName, "GetTriggerMode")) 
        return PGError;
    printDebug( PG_TRACE_FLOW,
        "%s::%s called CameraBase::GetTriggerMode, pCameraBase_=%p, pTriggerMode_=%p\n",
        driverName, functionName, pCameraBase_, pTriggerMode_);
    if (triggerMode == PGTriggerInternal) {
        pTriggerMode_->onOff = false;
    }
    else {
        pTriggerMode_->onOff = true;
        // The Point Grey values are 1 less than the enum value
        triggerMode--;
        pTriggerMode_->mode = triggerMode;
        pTriggerMode_->polarity = triggerPolarity;
        pTriggerMode_->source = triggerSource;
        switch (triggerMode) {
            case 3:
                skipFrames = PGSkipFrames;
                pTriggerMode_->parameter = skipFrames;
                break;
            case 4:
            case 5:
                numExposures = PGNumExposures;
                pTriggerMode_->parameter = numExposures;
                break;
            case 15:
                numImages = PGNumImages;
                pTriggerMode_->parameter = numImages;
                break;
        }
    }
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::SetTriggerMode, pCameraBase_=%p, pTriggerMode_=%p\n",
        driverName, functionName, pCameraBase_, pTriggerMode_);
    error = pCameraBase_->SetTriggerMode(pTriggerMode_);
    if (checkError(error, functionName, "SetTriggerMode")) 
        return PGError;
    /* When the trigger mode changes the properties can also change */
    getAllProperties();
    return PGSuccess;
}

int PGTest::softwareTrigger()
{
    Error error;
    static const char *functionName = "softwareTrigger";
    
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::FireSoftwareTrigger, pCameraBase_=%p\n",
        driverName, functionName, pCameraBase_);
    error = pCameraBase_->FireSoftwareTrigger();
    if (checkError(error, functionName, "FireSoftwareTrigger")) 
        return PGError;
    return PGSuccess;
}

int PGTest::setStrobe()
{
    int polarity;
    int source;
    int enable;
    double delay;
    double duration;
    Error error;
    static const char *functionName = "setStrobe";
    
    source = PGStrobeSource;
    enable = PGStrobeEnable;
    polarity = PGStrobePolarity;
    delay = PGStrobeDelay;
    duration = PGStrobeDuration;
    pStrobeControl_->source   = source;
    pStrobeControl_->onOff    = enable ? true : false;
    pStrobeControl_->polarity = polarity;
    pStrobeControl_->delay    = (float)(delay*1000.);
    pStrobeControl_->duration = (float)(duration*1000.);
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::SetStrobe, pCameraBase_=%p, pStrobeControl_=%p\n",
        driverName, functionName, pCameraBase_, pStrobeControl_);
    error = pCameraBase_->SetStrobe(pStrobeControl_);
    if (checkError(error, functionName, "SetStrobe")) 
        return PGError;
    return PGSuccess;
}

int PGTest::createStaticEnums()
{
    /* This function creates enum strings and values for all enums that are fixed for a given camera.
     * It is only called once at startup */
    int mode, rate, shift, pin, format;
    Error error;
    VideoMode videoMode;
    FrameRate frameRate;
    bool supported, modeSupported = false;
    static const char *functionName = "createStaticEnums";
     
    /* Video mode enums. A video mode is supported if it is supported for any frame rate */
    numValidVideoModes_ = 0;   
    for (mode=0; mode<NUM_VIDEOMODES; mode++) {
        videoMode = (VideoMode)mode;
        if (videoMode == VIDEOMODE_FORMAT7) {
            // We assume format7 is always supported for now
            modeSupported = true;
        } else if (pCamera_ != NULL) {
            modeSupported = false;
            for (rate=0; rate<NUM_FRAMERATES; rate++) {
                frameRate = (FrameRate)rate;
                if (frameRate == FRAMERATE_FORMAT7) continue;
                error = pCamera_->GetVideoModeAndFrameRateInfo(videoMode, frameRate, &supported);
                if (checkError(error, functionName, "GetVideoModeAndFrameRateInfo")) 
                    return PGError;
                printDebug( PG_TRACE_FLOW,
                    "%s::%s called Camera::GetVideoModeAndFrameRateInfo, pCamera_=%p, videoMode=%d, frameRate%d, supported=%d\n",
                    driverName, functionName, pCamera_, videoMode, frameRate, supported);
                if (supported) modeSupported = true;
            }                  
        }
        if (modeSupported) {
            numValidVideoModes_++;
        }
    }
    
    if (pCamera_ != NULL) {
        /* Format7 mode enums */
        /* Loop over modes */
        numValidFormat7Modes_ = 0;   
        for (mode=0; mode<NUM_MODES; mode++) {
            pFormat7Info_->mode = (Mode)mode;
            error = pCamera_->GetFormat7Info(pFormat7Info_, &supported);
            if (checkError(error, functionName, "GetFormat7Info")) 
                return PGError;
            printDebug( PG_TRACE_FLOW,
                "%s::%s called Camera::GetFormat7Info, pCamera_=%p, pFormat7Info_=%p, supported=%d\n",
                driverName, functionName, pCamera_, pFormat7Info_, supported);
            if (supported) {
                numValidFormat7Modes_++;
                if (numValidFormat7Modes_ == 1) {
                    // We assume that the lowest supported mode is the full chip size
                    PGMaxSizeX = pFormat7Info_->maxWidth;
                    PGMaxSizeY = pFormat7Info_->maxHeight;
                    PGSizeX = pFormat7Info_->maxWidth;
                    PGSizeY = pFormat7Info_->maxHeight;
                }
            }    
        }
    }
    
    if (pGigECamera_ != NULL) {
        /* GigE mode mode enums */
        /* Loop over modes */
        numValidFormat7Modes_ = 0;
        // Note: for GigE we cannot inquire about a mode without setting the camera to that mode.
        // So we need to remember the current mode
        Mode currentMode;   
        error = pGigECamera_->GetGigEImagingMode(&currentMode);
        if (checkError(error, functionName, "GetGigEImagingMode"))
            return PGError;
        printDebug( PG_TRACE_FLOW,
            "%s::%s called GigECamera::GetGigEImagingMode, pGigECamera_=%p, currentMode=%d\n",
            driverName, functionName, pGigECamera_, currentMode);
        for (mode=0; mode<NUM_MODES; mode++) {
            error = pGigECamera_->QueryGigEImagingMode((Mode)mode, &supported);
            if (checkError(error, functionName, "QueryGigEImagingMode")) 
                return PGError;
            printDebug( PG_TRACE_FLOW,
                "%s::%s called GigECamera::QueryGigEImagingMode, pGigECamera_=%p, mode=%d, supported=%d\n",
                driverName, functionName, pGigECamera_, mode, supported);
            if (supported) {
                printDebug( PG_TRACE_FLOW,
                    "%s::%s calling GigECamera::SetGigEImagingMode, pGigECamera_=%p, mode=%d\n",
                    driverName, functionName, pGigECamera_, mode);
                error = pGigECamera_->SetGigEImagingMode((Mode)mode);
                if (checkError(error, functionName, "SetGigEImagingMode"))
                    return PGError;
                printDebug( PG_TRACE_FLOW,
                    "%s::%s calling GigECamera::GetGigEImageSettingsInfo, pGigECamera_=%p, pGigEImageSettingsInfo_=%p\n",
                    driverName, functionName, pGigECamera_, pGigEImageSettingsInfo_);
                error = pGigECamera_->GetGigEImageSettingsInfo(pGigEImageSettingsInfo_);
                if (checkError(error, functionName, "GetGigEImageSettingsInfo")) 
                    return PGError;
                numValidFormat7Modes_++;
                if (numValidFormat7Modes_ == 1) {
                    // We assume that the lowest supported mode is the full chip size
                    PGMaxSizeX = pGigEImageSettingsInfo_->maxWidth;
                    PGMaxSizeY = pGigEImageSettingsInfo_->maxHeight;
                    PGSizeX = pGigEImageSettingsInfo_->maxWidth;
                    PGSizeY = pGigEImageSettingsInfo_->maxHeight;
                }
            }    
        }
    }
    
    /* Binning mode enums */
    if (pGigECamera_)
        numValidBinningModes_ = NUM_BINNING_MODES;
    else
        numValidBinningModes_ = 1;

    for (mode=0; mode<numValidBinningModes_; mode++) {
    }

    /* Convert pixel format enums */
    numValidConvertPixelFormats_ = NUM_CONVERT_PIXEL_FORMATS;
    for (format=0; format<numValidConvertPixelFormats_; format++) {
    }

    /* Trigger mode enums */
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::GetTriggerModeInfo, pCameraBase_=%p, pTriggerModeInfo_=%p\n",
        driverName, functionName, pCameraBase_, pTriggerModeInfo_);
    error = pCameraBase_->GetTriggerModeInfo(pTriggerModeInfo_);
    if (checkError(error, functionName, "GetTriggerModeInfo")) 
        return PGError;
    numValidTriggerModes_ = 0; 
    /* Loop over modes */
    for (mode=0; mode<NUM_TRIGGER_MODES; mode++) {
        // Internal trigger mode is always supported
        if (mode == 0) {
            supported = true;
        } else {
            shift = NUM_TRIGGER_MODES - mode - 1;
            supported = ((pTriggerModeInfo_->modeMask >> shift) & 0x1) == 1;
        }
        if (supported) {
            numValidTriggerModes_++;
        }    
    }

    /* Trigger source enums */
    numValidTriggerSources_ = 0; 
    for (pin=0; pin<NUM_GPIO_PINS; pin++) {
        shift = NUM_GPIO_PINS - pin - 1;
        supported = ((pTriggerModeInfo_->sourceMask >> shift) & 0x1) == 1;
        if (supported) {
            numValidTriggerSources_++;
        }    
    }

    /* Strobe source enums */
    numValidStrobeSources_ = 0; 
    for (pin=0; pin<NUM_GPIO_PINS; pin++) {
        pStrobeInfo_->source = pin;
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling CameraBase::GetStrobeInfo, pCameraBase_=%p, pStrobeInfo_=%p\n",
            driverName, functionName, pCameraBase_, pStrobeInfo_);
        error = pCameraBase_->GetStrobeInfo(pStrobeInfo_);
        if (checkError(error, functionName, "GetStrobeInfo")) 
            return PGError;
        if (pStrobeInfo_->present) {
            numValidStrobeSources_++;
        }
    }
    
    return PGSuccess;
}


int PGTest::createDynamicEnums()
{
    int format, rate;
    Error error;
    VideoMode currentVideoMode;
    FrameRate frameRate, currentFrameRate;
    Format7ImageSettings f7Settings;
    GigEImageSettings gigEImageSettings;
    unsigned int packetSize;
    float percentage;
    bool supported;
    static const char *functionName = "createDynamicEnums";
 
 
    if (pCamera_) {  
        // This is an IIDC (not GigE) camera 
        /* If the current video mode is format7 then this function creates enum strings and values 
         * for all of the valid pixel formats for the current format7 mode.
         * Otherwise it creates enum strings and values for all valid frame rates for the current video mode. */

        error = pCamera_->GetVideoModeAndFrameRate(&currentVideoMode, &currentFrameRate);
        if (checkError(error, functionName, "GetVideoModeAndFrameRate"))
            return PGError;
        printDebug( PG_TRACE_FLOW,
            "%s::%s called Camera::GetVideoModeAndFrameRate, pCamera_=%p, currentVideoMode=%d, currentFrameRate=%d\n",
            driverName, functionName, pCamera_, currentVideoMode, currentFrameRate);
        PGVideoMode = currentVideoMode;

        if (currentVideoMode == VIDEOMODE_FORMAT7) {
            error = pCamera_->GetFormat7Configuration(&f7Settings, &packetSize, &percentage);
            if (checkError(error, functionName, "GetFormat7Configuration")) 
                return PGError;
            printDebug( PG_TRACE_FLOW,
                "%s::%s called Camera::GetFormat7Configuration, pCamera_=%p, &f7Settings=%p, packetSize=%d, percentage=%f\n",
                driverName, functionName, pCamera_, &f7Settings, packetSize, percentage);
            pFormat7Info_->mode = f7Settings.mode;
            PGFormat7Mode = f7Settings.mode;
            error = pCamera_->GetFormat7Info(pFormat7Info_, &supported);
            if (checkError(error, functionName, "GetFormat7Info")) 
                return PGError;
            printDebug( PG_TRACE_FLOW,
                "%s::%s calling Camera::GetFormat7Info, pCamera_=%p, pFormat7Info_=%p, supported=%d\n",
                driverName, functionName, pCamera_, pFormat7Info_, supported);
            numValidPixelFormats_ = 0;
            for (format=0; format<(int)NUM_PIXEL_FORMATS; format++) {
                if ((pFormat7Info_->pixelFormatBitField & pixelFormatValues[format]) == pixelFormatValues[format]) {
                    numValidPixelFormats_++;
                }
            }
            PGPixelFormat = f7Settings.pixelFormat;
        } else {
            /* Format all valid frame rates for the current video mode */
            PGFrameRate = currentFrameRate;
            numValidFrameRates_ = 0;
            for (rate=0; rate<NUM_FRAMERATES; rate++) {
                frameRate = (FrameRate)rate;
                if (frameRate == FRAMERATE_FORMAT7) continue;
                error = pCamera_->GetVideoModeAndFrameRateInfo(currentVideoMode, frameRate, &supported);
                if (checkError(error, functionName, "GetVideoModeAndFrameRateInfo")) 
                    return PGError;
                printDebug( PG_TRACE_FLOW,
                    "%s::%s calling Camera::GetVideoModeAndFrameRateInfo, pCamera_=%p, currentVideoMode=%d, frameRate=%d, supported=%d\n",
                    driverName, functionName, pCamera_, currentVideoMode, frameRate, supported);
                if (supported) {
                    numValidFrameRates_++;
                } 
            }
        }
    }
    else if (pGigECamera_) {  
        // This is a GigE camera 
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling GigECamera::GetGigEImageSettings, pGigECamera_=%p, &gigEImageSettings=%p\n",
            driverName, functionName, pGigECamera_, &gigEImageSettings);
        error = pGigECamera_->GetGigEImageSettings(&gigEImageSettings);
        if (checkError(error, functionName, "GetGigEImageSettings")) 
            return PGError;
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling GigECamera::GetGigEImageSettingsInfo, pGigECamera_=%p, pGigEImageSettingsInfo_=%p\n",
            driverName, functionName, pGigECamera_, pGigEImageSettingsInfo_);
        error = pGigECamera_->GetGigEImageSettingsInfo(pGigEImageSettingsInfo_);
        if (checkError(error, functionName, "GetGigEImageSettingsInfo")) 
            return PGError;
        numValidPixelFormats_ = 0;
        for (format=0; format<(int)NUM_PIXEL_FORMATS; format++) {
            if ((pGigEImageSettingsInfo_->pixelFormatBitField & 
                     pixelFormatValues[format]) == pixelFormatValues[format]) {
                numValidPixelFormats_++;
            }
        }
        PGPixelFormat = gigEImageSettings.pixelFormat;
    }
    return PGSuccess;
}

/** Read all the propertyType settings and values from the camera.
 * This function will collect all the current values and settings from the camera,
 * and set the appropriate integer/double parameters in the param lib. If a certain propertyType
 * is not available in the given camera, this function will set all the parameters relating to that
 * propertyType to -1 or -1.0 to indicate it is not available.
 * Note the caller is responsible for calling any update callbacks if I/O interrupts
 * are to be processed after calling this function.
 * Returns int PGError or PGSuccess as an int.
 */
int PGTest::getAllProperties()
{
    PropertyInfo *pPropInfo;
    Property *pProperty;
    Error error;
    int addr;
    double dtmp;
    static const char *functionName="getAllProperties";

    /* Iterate through all of the available properties and update their values and settings  */
    for (addr=0; addr<NUM_PROPERTIES; addr++) {
        pPropInfo = allPropInfos_[addr];
        pProperty = allProperties_[addr];
        printDebug( PG_TRACE_FLOW,
            "%s::%s calling CameraBase::GetPropertyInfo, pCameraBase_=%p, pPropInfo=%p, propertyType=%d\n",
            driverName, functionName, pCameraBase_, pPropInfo, pProperty->type);
        error = pCameraBase_->GetPropertyInfo(pPropInfo);
        if (checkError(error, functionName, "GetPropertyInfo")) 
            return PGError;
        error = pCameraBase_->GetProperty(pProperty);
        if (checkError(error, functionName, "GetProperty")) 
            return PGError;
        printDebug( PG_TRACE_FLOW,
            "%s::%s called CameraBase::GetProperty, pCameraBase_=%p, pProperty=%p, pProperty->valueA=%d\n",
            driverName, functionName, pCameraBase_, pProperty, pProperty->valueA);
        printDebug( PG_TRACE_FLOW,
            "%s:%s: checking propertyType %d\n",
            driverName, functionName, addr);

        /* If the propertyType is not available in the camera, we just set
         * all the parameters to -1 to indicate this is not available to the user. */
        if (pPropInfo->present) {
            PGPropertyAvail[addr]      = 1;
            PGPropertyAutoAvail[addr]  = pPropInfo->autoSupported;
            PGPropertyManAvail[addr]   = pPropInfo->manualSupported;
            PGPropertyAbsAvail[addr]   = pPropInfo->absValSupported;
            PGPropertyAutoMode[addr]   = pProperty->autoManualMode;
            PGPropertyAbsMode[addr]    = pProperty->absControl;
            PGPropertyValue[addr]      = pProperty->valueA;
            PGPropertyValueB[addr]     = pProperty->valueB;
            PGPropertyValueMin[addr]   = pPropInfo->min;
            PGPropertyValueMax[addr]   = pPropInfo->max;
        } else {
            PGPropertyAvail[addr]     =  0;
            PGPropertyAutoAvail[addr] =  0;
            PGPropertyManAvail[addr]  =  0;
            PGPropertyAbsAvail[addr]  =  0;
            PGPropertyAutoMode[addr]  =  0;
            PGPropertyAbsMode[addr]   = -1;
            PGPropertyValue[addr]     = -1;
            PGPropertyValueMin[addr]  = -1;
            PGPropertyValueMax[addr]  = -1;
        }

        /* If the propertyType does not support 'absolute' control then we just
         * set all the absolute values to -1.0 to indicate it is not available to the user */
        if (pPropInfo->absValSupported) { 
            PGPropertyValueAbs[addr]    =  pProperty->absValue;
            PGPropertyValueAbsMin[addr] =  pPropInfo->absMin;
            PGPropertyValueAbsMax[addr] =  pPropInfo->absMax;
        } else {
            PGPropertyValueAbs[addr]    =  -1.0;
            PGPropertyValueAbsMax[addr] =  -1.0;
            PGPropertyValueAbsMin[addr] =  -1.0;
        }
    }

    /* Map a few of the AreaDetector parameters on to the camera properties */
    dtmp = PGPropertyValueAbs[SHUTTER];
    // Camera units are ms
    PGAcquireTime = dtmp/1000.;

    dtmp = PGPropertyValueAbs[FRAME_RATE];
    // Camera units are fps
    PGAcquirePeriod = 1./dtmp;

    dtmp = PGPropertyValueAbs[GAIN];
    PGGain = dtmp;

    return PGSuccess;
}

int PGTest::getAllGigEProperties()
{
    GigEProperty *pProperty;
    int addr;
    Error error;
    int itemp;
    static const char *functionName="getAllGigeEProperties";
    
    if (pGigECamera_ == NULL) return PGSuccess;

    /* Iterate through all of the available properties and update their values and settings  */
    for (addr=0; addr<NUM_GIGE_PROPERTIES; addr++) {
        pProperty = allGigEProperties_[addr];
        error = pGigECamera_->GetGigEProperty(pProperty);
        if (checkError(error, functionName, "GetGigEProperty")) 
            return PGError;
        printDebug( PG_TRACE_FLOW,
            "%s::%s called GigECamera::GetGigEProperty, pGigECamera_=%p, pProperty=%p, pProperty->propType=%d, pProperty->value=%d\n",
            driverName, functionName, pGigECamera_, pProperty, pProperty->propType, pProperty->value);

        if (pProperty->isReadable) {
            PGGigEPropertyValue[addr]    = pProperty->value;
            PGGigEPropertyValueMin[addr] = pProperty->min;
            PGGigEPropertyValueMax[addr] = pProperty->max;
        }
    }
    /* Map a few of the Point Grey parameters on to the GigE properties */
    itemp = PGGigEPropertyValue[PACKET_SIZE];
    PGPacketSizeActual = itemp;
    itemp = PGGigEPropertyValue[PACKET_DELAY];
    PGPacketDelayActual = itemp;

    return PGSuccess;
}


int PGTest::startCapture()
{
    Error error;
    static const char *functionName = "startCapture";

    /* Start the camera transmission... */
    PGNumImagesCounter = 0;
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::StartCapture, pCameraBase_=%p\n",
        driverName, functionName, pCameraBase_);
    error = pCameraBase_->StartCapture();
    if (checkError(error, functionName, "StartCapture")) 
        return PGError;
    return PGSuccess;
}


int PGTest::stopCapture()
{
    Error error;
    static const char *functionName = "stopCapture";

    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::StopCapture, pCameraBase_=%p\n",
        driverName, functionName, pCameraBase_);
    error = pCameraBase_->StopCapture();
    PGAcquire = 0;
    if (checkError(error, functionName, "StopCapture")) 
        return PGError;
    return PGSuccess;
}

int PGTest::readStatus()
{
    Error error;
    static const char *functionName = "readStatus";
    error = pCameraBase_->GetStats(pCameraStats_);
    if (checkError(error, functionName, "GetStats")) 
        return PGError;
    printDebug( PG_TRACE_FLOW,
        "%s::%s calling CameraBase::GetStats, pCameraBase_=%p, pCameraStats_=%p, pCameraStats_->temperature=%d\n",
        driverName, functionName, pCameraBase_, pCameraStats_, pCameraStats_->temperature);
    PGTemperatureActual = pCameraStats_->temperature/10. - 273.15;
    PGCorruptFrames = pCameraStats_->imageCorrupt;
    PGDriverDropped = pCameraStats_->imageDriverDropped;
    if (pCameraStats_->imageXmitFailed == 0x80000000) pCameraStats_->imageXmitFailed = 0;
    PGTransmitFailed = pCameraStats_->imageXmitFailed;
    PGDroppedFrames = pCameraStats_->imageDropped;
    return PGSuccess;
}

int main(int argc, char **argv)
{
    if (argc != 4) {
        printf("Usage: PGTest cameraId traceMask memoryChannel\n");
        return -1;
    }
    int cameraId = atoi(argv[1]);
    int traceMask = atoi(argv[2]);
    int memoryChannel = atoi(argv[3]);
    PGTest *PG = new PGTest(cameraId, traceMask, memoryChannel);
    PG->setPropertyAbsValue(FRAME_RATE,   46.0);
    PG->setPropertyAbsValue(SHUTTER,       5.0);
    PG->setPropertyValue(   AUTO_EXPOSURE, 800, propValueA);
    PG->setPropertyValue(   BRIGHTNESS,    128, propValueA);
    PG->setPropertyValue(   GAIN,            0, propValueA);
    PG->setPropertyValue(   GAMMA,        1024, propValueA);
    PG->setPropertyValue(   SHARPNESS,    1024, propValueA);
    PG->setPropertyValue(   SHARPNESS,    1024, propValueA);
    PG->PGStrobeDelay = .01;
    PG->setStrobe();
    PG->PGStrobeDuration = .02;
    PG->setStrobe();
    PG->setPropertyAbsValue(TRIGGER_DELAY, 0.0);
    PG->setPropertyAutoMode(AUTO_EXPOSURE,   1);
    PG->setPropertyAutoMode(BRIGHTNESS,      1);
    PG->PGStrobePolarity = 0;
    PG->setStrobe();
    PG->PGStrobeEnable = 1;
    PG->setStrobe();
    PG->PGMinX = 0;
    PG->setImageParams();
    PG->PGMinY = 0;
    PG->setImageParams();
    PG->PGNumExposures = 1;
    PG->setTrigger();
    PG->PGNumImages = 1;
    PG->setTrigger();
    PG->PGPacketDelay = 400;
    PG->setImageParams();
    PG->PGPacketSize = 1440;
    PG->setImageParams();
    PG->PGSizeX = 1600;
    PG->setImageParams();
    PG->PGSizeY = 1200;
    PG->setImageParams();
    PG->PGSkipFrames = 0;
    PG->setTrigger();
    PG->PGBinningMode = 1;
    PG->setImageParams();
    PG->PGFormat7Mode = 0;
    PG->setImageParams();
    PG->PGPixelFormat = 0;
    PG->setImageParams();
    PG->PGStrobeSource = 1;
    PG->setStrobe();
    PG->PGTriggerMode = 0;
    PG->setTrigger();
    PG->PGFrameRate = 0;
    PG->setFrameRate(0);
    PG->PGPixelFormat = 4194304;
    PG->setImageParams();
    PG->PGFrameRate = 0;
    PG->setFrameRate(0);
    PG->startCapture();
    PG->grabImage();
    PG->stopCapture();
    PG->disconnectCamera();
    
    return PGSuccess;
}

