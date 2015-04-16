/*
 * pointGrey.cpp
 *
 * This is a driver for PointGrey GigE, Firewire, and USB3 cameras using their FlyCapture2 SDK
 *
 * It is based on the FirewireWin areaDetector driver, which also supports Firewire DCAM drivers on Windows.
 * However, because this driver uses the PointGrey SDK it also supports USB3 cameras, which FirewireWin does not.
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

#define MAX(x,y) ((x)>(y)?(x):(y))

/* PointGrey driver specific parameters */
#define PGSerialNumberString          "PG_SERIAL_NUMBER"
#define PGFirmwareVersionString       "PG_FIRMWARE_VERSION"
#define PGSoftwareVersionString       "PG_SOFTWARE_VERSION"
#define PGPropertyAvailString         "PG_PROP_AVAIL"
#define PGPropertyOnOffAvailString    "PG_PROP_ON_OFF_AVAIL"
#define PGPropertyOnOffString         "PG_PROP_ON_OFF"
#define PGPropertyOnePushAvailString  "PG_PROP_ONE_PUSH_AVAIL"
#define PGPropertyOnePushString       "PG_PROP_ONE_PUSH"
#define PGPropertyAutoAvailString     "PG_PROP_AUTO_AVAIL"
#define PGPropertyManAvailString      "PG_PROP_MAN_AVAIL"
#define PGPropertyAutoModeString      "PG_PROP_AUTO_MODE"
#define PGPropertyAbsAvailString      "PG_PROP_ABS_AVAIL"
#define PGPropertyAbsModeString       "PG_PROP_ABS_MODE"
#define PGPropertyValueString         "PG_PROP_VAL"
#define PGPropertyValueBString        "PG_PROP_VAL_B"
#define PGPropertyValueMaxString      "PG_PROP_VAL_MAX"
#define PGPropertyValueMinString      "PG_PROP_VAL_MIN"
#define PGPropertyValueAbsString      "PG_PROP_VAL_ABS"
#define PGPropertyValueAbsMaxString   "PG_PROP_VAL_ABS_MAX"
#define PGPropertyValueAbsMinString   "PG_PROP_VAL_ABS_MIN"
#define PGGigEPropertyValueString     "PG_GIGE_PROP_VAL"
#define PGGigEPropertyValueMaxString  "PG_GIGE_PROP_VAL_MAX"
#define PGGigEPropertyValueMinString  "PG_GIGE_PROP_VAL_MIN"
#define PGVideoModeString             "PG_VIDEO_MODE"
#define PGFormat7ModeString           "PG_FORMAT7_MODE"
#define PGBinningModeString           "PG_BINNING_MODE"
#define PGFrameRateString             "PG_FRAME_RATE"
#define PGPixelFormatString           "PG_PIXEL_FORMAT"
#define PGConvertPixelFormatString    "PG_CONVERT_PIXEL_FORMAT"
#define PGTriggerSourceString         "PG_TRIGGER_SOURCE"
#define PGTriggerPolarityString       "PG_TRIGGER_POLARITY"
#define PGSoftwareTriggerString       "PG_SOFTWARE_TRIGGER"
#define PGSkipFramesString            "PG_SKIP_FRAMES"
#define PGStrobeSourceString          "PG_STROBE_SOURCE"
#define PGStrobePolarityString        "PG_STROBE_POLARITY"
#define PGStrobeEnableString          "PG_STROBE_ENABLE"
#define PGStrobeDelayString           "PG_STROBE_DELAY"
#define PGStrobeDurationString        "PG_STROBE_DURATION"
#define PGPacketSizeString            "PG_PACKET_SIZE"
#define PGPacketSizeActualString      "PG_PACKET_SIZE_ACTUAL"
#define PGMaxPacketSizeString         "PG_MAX_PACKET_SIZE"
#define PGPacketDelayString           "PG_PACKET_DELAY"
#define PGPacketDelayActualString     "PG_PACKET_DELAY_ACTUAL"
#define PGBandwidthString             "PG_BANDWIDTH"
#define PGTimeStampModeString         "PG_TIME_STAMP_MODE"
#define PGCorruptFramesString         "PG_CORRUPT_FRAMES"
#define PGDriverDroppedString         "PG_DRIVER_DROPPED"
#define PGTransmitFailedString        "PG_TRANSMIT_FAILED"
#define PGDroppedFramesString         "PG_DROPPED_FRAMES"

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

typedef enum {
    propValueA,
    propValueB
} propValue_t;

#define MAX_ENUM_STRING_SIZE 26
typedef struct {
    int value;
    char string[MAX_ENUM_STRING_SIZE];
} enumStruct_t;

static const char *videoModeStrings[NUM_VIDEOMODES] = {
    "160x120 YUV444",  
    "320x240 YUV422", 
    "640x480 YUV411", 
    "640X480 YUV422",   
    "640x480 RGB",   
    "640x480 Mono8",   
    "640x480 Mono16",
    "800x600 YUV422",  
    "800x600 RGB",    
    "800x600 Mono8",  
    "1024x768 YUV422",  
    "1024x768 RGB",  
    "1024x768 Mono8",  
    "800x600 Mono16",  
    "1024x768 Mono16",
    "1280x960 YUV422", 
    "1280x960 RGB",   
    "1280x960 Mono8", 
    "1600x1200 YUV422", 
    "1600x1200 RGB", 
    "1600x1200 Mono8", 
    "1280x960 Mono16", 
    "1600x1200 Mono16",
    "Format7"
};

static const char *frameRateStrings[NUM_FRAMERATES] = {
    "1.875",
    "3.75",
    "7.5",
    "15",
    "30",
    "60",
    "120",
    "240"
};

static const char *pixelFormatStrings[NUM_PIXEL_FORMATS] = {
    "Mono8",
    "YUV411",
    "YUV422",
    "YUV444",
    "RGB8",
    "Mono16",
    "RGB16",
    "Mono16_Signed",
    "RGB16_Signed",
    "Raw8",
    "Raw16",
    "Mono12",
    "Raw12",
    "BGR",
    "BGRU",
    "BGR16",
    "BGRU16",
    "YUV422_JPEG"
};

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

static const char *binningModeStrings[NUM_BINNING_MODES] = {
    "1x1",
    "2x2",
    "4x4",
};

static const unsigned int binningModeValues[NUM_BINNING_MODES] = {
    1,
    2,
    4,
};

static const char *convertPixelFormatStrings[NUM_CONVERT_PIXEL_FORMATS] = {
    "None",
    "Mono8",
    "Mono16",
    "RGB8",
    "RGB16",
};

static const int convertPixelFormatValues[NUM_CONVERT_PIXEL_FORMATS] = {
    0,
    PIXEL_FORMAT_MONO8,
    PIXEL_FORMAT_MONO16,
    PIXEL_FORMAT_RGB8,
    PIXEL_FORMAT_RGB16,
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


static const char *triggerModeStrings[NUM_TRIGGER_MODES] = {
    "Internal",
    "Ext. Standard",
    "Bulb",
    "Undefined",
    "Skip frames",
    "Multi-exposure",
    "Multi-exposure bulb",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Undefined",
    "Low smear",
    "Overlapped",
    "Multi-shot"
};

static const char *GPIOStrings[NUM_GPIO_PINS] = {
    "GPIO_0",
    "GPIO_1",
    "GPIO_2",
    "GPIO_3"
};

typedef enum {
   TimeStampCamera,
   TimeStampEPICS,
   TimeStampHybrid
} PGTimeStamp_t;

/** Main driver class inherited from areaDetectors ADDriver class.
 * One instance of this class will control one camera.
 */
class pointGrey : public ADDriver
{
public:
    pointGrey(const char *portName, int cameraId, int traceMask, int memoryChannel,
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize);

    /* virtual methods to override from ADDriver */
    virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                                size_t nElements, size_t *nIn);
    void report(FILE *fp, int details);
    /**< These should be private but are called from C callback functions, must be public. */
    void imageGrabTask();
    void shutdown();

protected:
    int PGSerialNumber;           /** Camera serial number (int32 read) */
    #define FIRST_PG_PARAM PGSerialNumber
    int PGFirmwareVersion;        /** Camera firmware version                         (octet read) */
    int PGSoftwareVersion;        /** Camera software version                         (octet read) */
                                  /** The following PGProperty parameters 
                                      all have addr: 0-NUM_PROPERTIES-1 */
    int PGPropertyAvail;          /** Property is available                           (int32 read) */
    int PGPropertyOnOffAvail;     /** Property on/off is available                    (int32 read) */
    int PGPropertyOnOff;          /** Property on/off                                 (int32 read) */
    int PGPropertyOnePushAvail;   /** Property one push is available                  (int32 read) */
    int PGPropertyOnePush;        /** Property one push                               (int32 read) */
    int PGPropertyAutoAvail;      /** Property auto mode available                    (int32 read) */
    int PGPropertyManAvail;       /** Property manual mode available                  (int32 read) */
    int PGPropertyAutoMode;       /** Property control mode: 0:manual or 1:automatic  (int32 read/write) */
    int PGPropertyAbsAvail;       /** Property has absolute (floating point) controls (int32 read) */
    int PGPropertyAbsMode;        /** Property raw/absolute mode: 0:raw or 1:absolute (int32 read/write) */
    int PGPropertyValue;          /** Property value                                  (int32 read/write) */
    int PGPropertyValueB;         /** Property value B                                (int32 read/write) */
    int PGPropertyValueMax;       /** Property maximum value                          (int32 read) */
    int PGPropertyValueMin;       /** Property minimum value                          (int32 read) */
    int PGPropertyValueAbs;       /** Property absolute value                         (float64 read/write) */
    int PGPropertyValueAbsMax;    /** Property absolute maximum value                 (float64 read) */
    int PGPropertyValueAbsMin;    /** Property absolute minimum value                 (float64 read) */
    int PGGigEPropertyValue;      /** GigE property value                             (int32 read/write) */
    int PGGigEPropertyValueMax;   /** GigE property maximum value                     (int32 read) */
    int PGGigEPropertyValueMin;   /** GigE property minimum value                     (int32 read) */
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
    int PGStrobeDelay;            /** Strobe delay                                    (float64 write/read) */
    int PGStrobeDuration;         /** Strobe duration                                 (float64 write/read) */
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

private:
    /* Local methods to this class */
    asynStatus grabImage();
    asynStatus startCapture();
    asynStatus stopCapture();

    inline asynStatus checkError(Error error, const char *functionName, const char *message);
    asynStatus connectCamera();
    asynStatus disconnectCamera();
    asynStatus readStatus();

    /* camera property control functions */
    asynStatus setPropertyValue(PropertyType propType, int value, propValue_t valType);
    asynStatus setGigEPropertyValue(GigEPropertyType propType, int value);
    asynStatus setPropertyAbsValue(PropertyType propType, epicsFloat64 value);
    asynStatus setPropertyAutoMode(PropertyType propType, int value);
    asynStatus setPropertyOnOff(PropertyType propType, int onOff);
    asynStatus setPropertyOnePush(PropertyType propType);
    asynStatus setVideoMode(int mode);
    asynStatus setFrameRate(int rate);
    asynStatus setVideoModeAndFrameRate(int mode, int frameRate);
    asynStatus setImageParams();
    asynStatus setFormat7Params();
    asynStatus setGigEImageParams();
    asynStatus createStaticEnums();
    asynStatus createDynamicEnums();
    asynStatus getAllProperties();
    asynStatus getAllGigEProperties();
    int getPixelFormatIndex(PixelFormat pixelFormat);
    asynStatus setTrigger();
    asynStatus softwareTrigger();
    asynStatus setStrobe();

    /* Data */
    int cameraId_;
    int memoryChannel_;
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
    enumStruct_t videoModeEnums_          [NUM_VIDEOMODES];
    enumStruct_t format7ModeEnums_        [NUM_MODES];
    enumStruct_t binningModeEnums_        [NUM_BINNING_MODES];
    enumStruct_t frameRateEnums_          [NUM_FRAMERATES];
    enumStruct_t pixelFormatEnums_        [NUM_PIXEL_FORMATS];
    enumStruct_t convertPixelFormatEnums_ [NUM_PIXEL_FORMATS];
    enumStruct_t triggerModeEnums_        [NUM_TRIGGER_MODES];
    enumStruct_t triggerSourceEnums_      [NUM_GPIO_PINS];
    enumStruct_t strobeSourceEnums_       [NUM_GPIO_PINS];
    int exiting_;
    epicsEventId startEventId_;
    NDArray *pRaw_;
};

/** Number of asynPortDriver parameters this driver supports. */
#define NUM_PG_PARAMS ((int)(&LAST_PG_PARAM - &FIRST_PG_PARAM + 1))

/** Configuration function to configure one camera.
 *
 * This function need to be called once for each camera to be used by the IOC. A call to this
 * function instanciates one object from the pointGrey class.
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId The camera index or serial number.
 * \param[in] traceMask The initial value of the asynTraceMask.  
 *            If set to 0 or 1 then asynTraceMask will be set to ASYN_TRACE_ERROR.
 *            If set to 0x21 (ASYN_TRACE_WARNING | ASYN_TRACE_ERROR) then each call to the
 *            FlyCap2 library will be traced including during initialization.
 * \param[in] memoryChannel  The camera memory channel (non-volatile memory containing camera parameters) 
 *            to load during initialization.  If 0 no memory channel is loaded.
 *            If >=1 thenRestoreFromMemoryChannel(memoryChannel-1) is called.  
 *            Set memoryChannel to 1 to work around a bug in the Linux GigE driver in R2.0.
 * \param[in] maxBuffers Maxiumum number of NDArray objects (image buffers) this driver is allowed to allocate.
 *            This driver requires 2 buffers, and each queue element in a plugin can require one buffer
 *            which will all need to be added up in this parameter. 0=unlimited.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *            and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB). 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
extern "C" int pointGreyConfig(const char *portName, int cameraId, int traceMask, int memoryChannel, 
                               int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new pointGrey( portName, cameraId, traceMask, memoryChannel, maxBuffers, maxMemory, priority, stackSize);
    return asynSuccess;
}

static void c_shutdown(void *arg)
{
  pointGrey *p = (pointGrey *)arg;
  p->shutdown();
}

static void imageGrabTaskC(void *drvPvt)
{
    pointGrey *pPvt = (pointGrey *)drvPvt;

    pPvt->imageGrabTask();
}

/** Constructor for the pointGrey class
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId The camera index or serial number.
 * \param[in] traceMask The initial value of the asynTraceMask.  
 *            If set to 0 or 1 then asynTraceMask will be set to ASYN_TRACE_ERROR.
 *            If set to 0x21 (ASYN_TRACE_WARNING | ASYN_TRACE_ERROR) then each call to the
 *            FlyCap2 library will be traced including during initialization.
 * \param[in] memoryChannel  The camera memory channel (non-volatile memory containing camera parameters) 
 *            to load during initialization.  If 0 no memory channel is loaded.
 *            If >=1 thenRestoreFromMemoryChannel(memoryChannel-1) is called.  
 *            Set memoryChannel to 1 to work around a bug in the Linux GigE driver in R2.0.
 * \param[in] maxBuffers Maxiumum number of NDArray objects (image buffers) this driver is allowed to allocate.
 *            This driver requires 2 buffers, and each queue element in a plugin can require one buffer
 *            which will all need to be added up in this parameter. 0=unlimited.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *            and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB). 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
pointGrey::pointGrey(const char *portName, int cameraId, int traceMask, int memoryChannel,
                    int maxBuffers, size_t maxMemory, int priority, int stackSize )
    : ADDriver(portName, MAX_ADDR, NUM_PG_PARAMS, maxBuffers, maxMemory,
            asynEnumMask, asynEnumMask,
            ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, priority, stackSize),
    cameraId_(cameraId), memoryChannel_(memoryChannel), pGuid_(0), exiting_(0), pRaw_(NULL)
{
    static const char *functionName = "pointGrey";
    int i;
    Error error;
    PropertyType propType;
    asynStatus status;
    
    if (traceMask == 0) traceMask = ASYN_TRACE_ERROR;
    pasynTrace->setTraceMask(pasynUserSelf, traceMask);

    createParam(PGSerialNumberString,           asynParamInt32,   &PGSerialNumber);
    createParam(PGFirmwareVersionString,        asynParamOctet,   &PGFirmwareVersion);
    createParam(PGSoftwareVersionString,        asynParamOctet,   &PGSoftwareVersion);
    createParam(PGPropertyAvailString,          asynParamInt32,   &PGPropertyAvail);
    createParam(PGPropertyOnOffAvailString,     asynParamInt32,   &PGPropertyOnOffAvail);
    createParam(PGPropertyOnOffString,          asynParamInt32,   &PGPropertyOnOff);
    createParam(PGPropertyOnePushAvailString,   asynParamInt32,   &PGPropertyOnePushAvail);
    createParam(PGPropertyOnePushString,        asynParamInt32,   &PGPropertyOnePush);
    createParam(PGPropertyAutoAvailString,      asynParamInt32,   &PGPropertyAutoAvail);
    createParam(PGPropertyManAvailString,       asynParamInt32,   &PGPropertyManAvail);
    createParam(PGPropertyAutoModeString,       asynParamInt32,   &PGPropertyAutoMode);
    createParam(PGPropertyAbsAvailString,       asynParamInt32,   &PGPropertyAbsAvail);
    createParam(PGPropertyAbsModeString,        asynParamInt32,   &PGPropertyAbsMode);
    createParam(PGPropertyValueString,          asynParamInt32,   &PGPropertyValue);
    createParam(PGPropertyValueBString,         asynParamInt32,   &PGPropertyValueB);
    createParam(PGPropertyValueMaxString,       asynParamInt32,   &PGPropertyValueMax);
    createParam(PGPropertyValueMinString,       asynParamInt32,   &PGPropertyValueMin);
    createParam(PGPropertyValueAbsString,       asynParamFloat64, &PGPropertyValueAbs);
    createParam(PGPropertyValueAbsMaxString,    asynParamFloat64, &PGPropertyValueAbsMax);
    createParam(PGPropertyValueAbsMinString,    asynParamFloat64, &PGPropertyValueAbsMin);
    createParam(PGGigEPropertyValueString,      asynParamInt32,   &PGGigEPropertyValue);
    createParam(PGGigEPropertyValueMaxString,   asynParamInt32,   &PGGigEPropertyValueMax);
    createParam(PGGigEPropertyValueMinString,   asynParamInt32,   &PGGigEPropertyValueMin);
    createParam(PGVideoModeString,              asynParamInt32,   &PGVideoMode);
    createParam(PGFormat7ModeString,            asynParamInt32,   &PGFormat7Mode);
    createParam(PGBinningModeString,            asynParamInt32,   &PGBinningMode);
    createParam(PGFrameRateString,              asynParamInt32,   &PGFrameRate);
    createParam(PGPixelFormatString,            asynParamInt32,   &PGPixelFormat);
    createParam(PGConvertPixelFormatString,     asynParamInt32,   &PGConvertPixelFormat);
    createParam(PGTriggerSourceString,          asynParamInt32,   &PGTriggerSource);
    createParam(PGTriggerPolarityString,        asynParamInt32,   &PGTriggerPolarity);
    createParam(PGSoftwareTriggerString,        asynParamInt32,   &PGSoftwareTrigger);
    createParam(PGSkipFramesString,             asynParamInt32,   &PGSkipFrames);
    createParam(PGStrobeSourceString,           asynParamInt32,   &PGStrobeSource);
    createParam(PGStrobePolarityString,         asynParamInt32,   &PGStrobePolarity);
    createParam(PGStrobeEnableString,           asynParamInt32,   &PGStrobeEnable);
    createParam(PGStrobeDelayString,            asynParamFloat64, &PGStrobeDelay);
    createParam(PGStrobeDurationString,         asynParamFloat64, &PGStrobeDuration);
    createParam(PGPacketSizeString,             asynParamInt32,   &PGPacketSize);
    createParam(PGPacketSizeActualString,       asynParamInt32,   &PGPacketSizeActual);
    createParam(PGMaxPacketSizeString,          asynParamInt32,   &PGMaxPacketSize);
    createParam(PGPacketDelayString,            asynParamInt32,   &PGPacketDelay);
    createParam(PGPacketDelayActualString,      asynParamInt32,   &PGPacketDelayActual);
    createParam(PGBandwidthString,              asynParamFloat64, &PGBandwidth);
    createParam(PGTimeStampModeString,          asynParamInt32,   &PGTimeStampMode);
    createParam(PGCorruptFramesString,          asynParamInt32,   &PGCorruptFrames);
    createParam(PGDriverDroppedString,          asynParamInt32,   &PGDriverDropped);
    createParam(PGTransmitFailedString,         asynParamInt32,   &PGTransmitFailed);
    createParam(PGDroppedFramesString,          asynParamInt32,   &PGDroppedFrames);

    /* Set initial values of some parameters */
    setIntegerParam(NDDataType, NDUInt8);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(ADMinX, 0);
    setIntegerParam(ADMinY, 0);
    setIntegerParam(PGVideoMode, 0);
    setIntegerParam(PGFormat7Mode, 0);
    setIntegerParam(PGFrameRate, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");
    setIntegerParam(PGTriggerSource, 0);
    setIntegerParam(PGStrobeSource, 1);
    setIntegerParam(PGBinningMode, 1);
    
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
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        // Call report() to get a list of available cameras
        report(stdout, 1);
        return;
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

//    if (traceMask > ASYN_TRACE_ERROR) report(stdout, 1);

    createStaticEnums();
    createDynamicEnums();

    numValidFrameRates_ = 2;
    strcpy(frameRateEnums_[0].string, "Undefined1");
    frameRateEnums_[0].value = 0;
    strcpy(frameRateEnums_[1].string, "Undefined2");
    frameRateEnums_[1].value = 1;
    numValidPixelFormats_ = 2;
    strcpy(pixelFormatEnums_[0].string, "Undefined1");
    pixelFormatEnums_[0].value = 0;
    strcpy(pixelFormatEnums_[1].string, "Undefined2");
    pixelFormatEnums_[1].value = 1;
    // Get and set maximum packet size and default packet delay for GigE cameras
    if (pGigECamera_) {
        unsigned int packetSize;
        error = pGigECamera_->DiscoverGigEPacketSize(&packetSize);
        if (checkError(error, functionName, "DiscoverGigEPacketSize")) {
            // There was an error getting the packet size, use 1440 as default;
            packetSize = 1440;
        }
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s called DiscoverGigEPacketSize, pGigECamera_=%p, packetSize=%d\n",
            driverName, functionName, pGigECamera_, packetSize);
        setIntegerParam(PGMaxPacketSize, packetSize);
        setIntegerParam(PGPacketDelay, DEFAULT_PACKET_DELAY);
        setGigEPropertyValue(PACKET_SIZE, packetSize);
        setGigEPropertyValue(PACKET_DELAY, DEFAULT_PACKET_DELAY);
    }

    startEventId_ = epicsEventCreate(epicsEventEmpty);

    /* launch image read task */
    epicsThreadCreate("PointGreyImageTask", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      imageGrabTaskC, this);

    /* shutdown on exit */
    epicsAtExit(c_shutdown, this);
    return;
}

inline asynStatus pointGrey::checkError(Error error, const char *functionName, const char *PGRFunction)
{
    if (error != PGRERROR_OK) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: ERROR calling %s Type=%d Description=%s\n",
            driverName, functionName, PGRFunction, error.GetType(), error.GetDescription());
        return asynError;
    }
    return asynSuccess;
}

void pointGrey::shutdown(void)
{
    exiting_ = 1;
    if (pGuid_) {
        disconnectCamera();
        delete pCameraBase_;
    }
}

asynStatus pointGrey::connectCamera(void)
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
    if (checkError(error, functionName, "GetNumOfCameras")) return asynError;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s called BusManager::GetNumOfCameras, pBusMgr_=%p, numCameras=%d\n",
        driverName, functionName, pBusMgr_, numCameras);
    
    if (numCameras <= 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: no cameras found\n",
            driverName, functionName);
    }

    if (cameraId_ == 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling BusManager::GetCameraFromIndex, pBusMgr_=%p, pGuid_=%p\n",
            driverName, functionName, pBusMgr_, pGuid_);
        error = pBusMgr_->GetCameraFromIndex(0, pGuid_);
        if (checkError(error, functionName, "GetCameraFromIndex")) return asynError;
    } else { 
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling BusManager::GetCameraFromSerialNumber, pBusMgr_=%p, cameraId_=%d, pGuid_=%p\n",
            driverName, functionName, pBusMgr_, cameraId_, pGuid_);
        error = pBusMgr_->GetCameraFromSerialNumber(cameraId_, pGuid_);
        if (checkError(error, functionName, "GetCameraFromSerialNumber")) return asynError;
    }
    error = pBusMgr_->GetInterfaceTypeFromGuid(pGuid_, &interfaceType);
    if (checkError(error, functionName, "GetInterfaceTypeFromGuid")) return asynError;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s called BusManager::GetInterfaceTypeFromGuid, pBusMgr_=%p, interfaceType=%d\n",
        driverName, functionName, pBusMgr_, interfaceType);
    
    // Create appropriate camera object
    if (interfaceType == INTERFACE_GIGE) {
        pCameraBase_ = new GigECamera;
        pCamera_ = NULL;
        pGigECamera_ = dynamic_cast<GigECamera*>(pCameraBase_);
        if (pGigECamera_ == NULL) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s error casting camera to GigECamera\n",
                driverName, functionName);
            return asynError;
        }
    } else {
        pCameraBase_ = new Camera;
        pGigECamera_ = NULL;
        pCamera_ = dynamic_cast<Camera*>(pCameraBase_);
        if (pCamera_ == NULL) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s error casting camera to Camera\n",
                driverName, functionName);
            return asynError;
        }   
    }

    // Connect to camera
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::Connect, pGuid_=%p\n",
        driverName, functionName, pGuid_);
    error = pCameraBase_->Connect(pGuid_);
    if (checkError(error, functionName, "Connect")) return asynError;

    // Restore from memory channel if it was specified
    if (memoryChannel_ > 0) {
        unsigned int numMemoryChannels;
        error = pCameraBase_->GetMemoryChannelInfo(&numMemoryChannels);
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s called GetMemoryChannelInfo, pCameraBase_=%p, numMemoryChannels=%u\n",
            driverName, functionName, pCameraBase_, numMemoryChannels);
        if (memoryChannel_ > (int) numMemoryChannels) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s memory channel=%d invalid, numMemoryChannels=%d\n", 
                driverName, functionName, memoryChannel_, numMemoryChannels);
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                "%s::%s calling RestoreFromMemoryChannel, pCameraBase_=%p, memoryChannel=%d\n",
                driverName, functionName, pCameraBase_, memoryChannel_-1);
            error = pCameraBase_->RestoreFromMemoryChannel(memoryChannel_-1);
            checkError(error, functionName, "RestoreFromMemoryChannel");
        }
    }
    
    // Get the camera information
    error = pCameraBase_->GetCameraInfo(pCameraInfo_);
    if (checkError(error, functionName, "GetCameraInfo")) return asynError;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s called CameraBase::GetCameraInfo, pCameraInfo_=%p, pCameraInfo_->serialNumber=%d\n",
        driverName, functionName, pCameraInfo_, pCameraInfo_->serialNumber);
    setIntegerParam(PGSerialNumber, pCameraInfo_->serialNumber);
    setStringParam(ADManufacturer, pCameraInfo_->vendorName);
    setStringParam(ADModel, pCameraInfo_->modelName);
    setStringParam(PGFirmwareVersion, pCameraInfo_->firmwareVersion);
    
    Utilities::GetLibraryVersion(&version);
    sprintf(tempString, "%d.%d.%d", version.major, version.minor, version.type);
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s called Utilities::GetLibraryVersion, version=%s\n",
        driverName, functionName, tempString);
    setStringParam(PGSoftwareVersion, tempString);
    
    // Get and set the embedded image info
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::GetEmbeddedImageInfo, &embeddedInfo=%p\n",
        driverName, functionName, &embeddedInfo);
    error = pCameraBase_->GetEmbeddedImageInfo(&embeddedInfo);
    if (checkError(error, functionName, "GetEmbeddedImageInfo")) return asynError;
    // Force the timestamp and frame counter information to be on
    embeddedInfo.timestamp.onOff = true;
    embeddedInfo.frameCounter.onOff = true;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::SetEmbeddedImageInfo, &embeddedInfo=%p\n",
        driverName, functionName, &embeddedInfo);
    error = pCameraBase_->SetEmbeddedImageInfo(&embeddedInfo);
    if (checkError(error, functionName, "SetEmbeddedImageInfo")) return asynError;
    
    return asynSuccess;
}

asynStatus pointGrey::disconnectCamera(void) 
{
    Error error;
    static const char *functionName = "disconnectCamera";

    if (pCameraBase_->IsConnected()) {
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling CameraBase::Disconnect, pCameraBase_=%p\n",
            driverName, functionName, pCameraBase_);
        error = pCameraBase_->Disconnect();
        if (checkError(error, functionName, "Disconnect")) return asynError;
    }
    printf("%s:%s disconnect camera OK\n", driverName, functionName);
    return asynSuccess;
}


/** Task to grab images off the camera and send them up to areaDetector
 *
 */
void pointGrey::imageGrabTask()
{
    asynStatus status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;
    int acquire;
    static const char *functionName = "imageGrabTask";

    lock();

    while (1) {
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire) {
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();

            /* Wait for a signal that tells this thread that the transmission
             * has started and we can start asking for image buffers...     */
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s waiting for acquire to start\n", 
                driverName, functionName);
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            unlock();
            epicsEventWait(startEventId_);
            lock();
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s started!\n", 
                driverName, functionName);
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADAcquire, 1);
        }

        /* Get the current time */
        epicsTimeGetCurrent(&startTime);
        /* We are now waiting for an image  */
        setIntegerParam(ADStatus, ADStatusWaiting);
        /* Call the callbacks to update any changes */
        callParamCallbacks();

        status = grabImage();
        if (status == asynError) {
            /* remember to release the NDArray back to the pool now
             * that we are not using it (we didn't get an image...) */
            if (pRaw_) pRaw_->release();
            pRaw_ = NULL;
            continue;
        }

        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        if (arrayCallbacks) {
            /* Call the NDArray callback */
            /* Must release the lock here, or we can get into a deadlock, because we can
             * block on the plugin lock, and the plugin can be calling us */
            unlock();
            doCallbacksGenericPointer(pRaw_, NDArrayData, 0);
            lock();
        }
        /* Release the NDArray buffer now that we are done with it.
         * After the callback just above we don't need it anymore */
        pRaw_->release();
        pRaw_ = NULL;

        /* See if acquisition is done if we are in single or multiple mode */
        if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
            status = stopCapture();
        }
        callParamCallbacks();
    }
}

asynStatus pointGrey::grabImage()
{
    asynStatus status = asynSuccess;
    Error error;
    unsigned int nRows, nCols, stride;
    PixelFormat pixelFormat;
    BayerTileFormat bayerFormat;
    NDDataType_t dataType;
    NDColorMode_t colorMode;
    Image *pPGImage;
    ImageMetadata metaData;
    TimeStamp timeStamp;
    int convertPixelFormat;
    int numColors;
    size_t dims[3];
    int pixelSize;
    size_t dataSize, dataSizePG;
    double bandwidth;
    double frameRate;
    void *pData;
    int nDims;
    int timeStampMode;
    static const char *functionName = "grabImage";

    /* unlock the driver while we wait for a new image to be ready */
    unlock();
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::RetrieveBuffer, pCameraBase_=%p, pPGRawImage_=%p\n",
        driverName, functionName, pCameraBase_, pPGRawImage_);
    error = pCameraBase_->RetrieveBuffer(pPGRawImage_);
    lock();
    if (error != PGRERROR_OK) {
        if (error == PGRERROR_ISOCH_NOT_STARTED) {
            // This is an expected error if acquisition was stopped externally
            return asynError;
        } 
        checkError(error, functionName, "RetrieveBuffer");
        if (error == PGRERROR_IMAGE_CONSISTENCY_ERROR) {
            // For now we ignore this error
        } else {
            //  Any other error we turn off acquisition and return an error
            setIntegerParam(ADAcquire, 0);
            return asynError;
        }
    }
    pPGRawImage_->GetDimensions(&nRows, &nCols, &stride, &pixelFormat, &bayerFormat);    
    metaData = pPGRawImage_->GetMetadata();    
    timeStamp = pPGRawImage_->GetTimeStamp();    
    pPGImage = pPGRawImage_;
    // Calculate bandwidth
    dataSizePG = pPGRawImage_->GetReceivedDataSize();
    getDoubleParam(FRAME_RATE, PGPropertyValueAbs, &frameRate);
    bandwidth = frameRate * dataSizePG / (1024 * 1024);
    setDoubleParam(PGBandwidth, bandwidth);

    // If the incoming pixel format is raw[8,12,16] or mono12 and convertPixelFormat is non-zero then convert
    // the pixel format of the image
    getIntegerParam(PGConvertPixelFormat, &convertPixelFormat);
    if (((pixelFormat == PIXEL_FORMAT_RAW8)   ||
         (pixelFormat == PIXEL_FORMAT_RAW12)  ||
         (pixelFormat == PIXEL_FORMAT_MONO12) ||
         (pixelFormat == PIXEL_FORMAT_RAW16)) &&
          convertPixelFormat != 0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling Image::Convert, pPGRawImage_=%p, convertPixelFormat=%d, pPGConvertedImage_=%p\n",
            driverName, functionName, pPGRawImage_, convertPixelFormat, pPGConvertedImage_);
        error = pPGRawImage_->Convert((PixelFormat)convertPixelFormat, pPGConvertedImage_);
        if (checkError(error, functionName, "Convert") == 0)
            pPGImage = pPGConvertedImage_;
    }
    
    pPGImage->GetDimensions(&nRows, &nCols, &stride, &pixelFormat, &bayerFormat);
    switch (pixelFormat) {
        case PIXEL_FORMAT_MONO8:
        case PIXEL_FORMAT_RAW8:
            dataType = NDUInt8;
            colorMode = NDColorModeMono;
            numColors = 1;
            pixelSize = 1;
            break;

        case PIXEL_FORMAT_RGB8:
            dataType = NDUInt8;
            colorMode = NDColorModeRGB1;
            numColors = 3;
            pixelSize = 1;
            break;

        case PIXEL_FORMAT_MONO16:
        case PIXEL_FORMAT_RAW16:
            dataType = NDUInt16;
            colorMode = NDColorModeMono;
            numColors = 1;
            pixelSize = 2;
            break;

        case PIXEL_FORMAT_S_MONO16:
            dataType = NDInt16;
            colorMode = NDColorModeMono;
            numColors = 1;
            pixelSize = 2;
            break;

        case PIXEL_FORMAT_RGB16:
            dataType = NDUInt16;
            colorMode = NDColorModeRGB1;
            numColors = 3;
            pixelSize = 2;
            break;

        case PIXEL_FORMAT_S_RGB16:
            dataType = NDInt16;
            colorMode = NDColorModeRGB1;
            numColors = 3;
            pixelSize = 2;
            break;

        default:
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unsupported pixel format=0x%x\n",
                driverName, functionName, pixelFormat);
            return asynError;
    }

    if (numColors == 1) {
        nDims = 2;
        dims[0] = nCols;
        dims[1] = nRows;
    } else {
        nDims = 3;
        dims[0] = 3;
        dims[1] = nCols;
        dims[2] = nRows;
    }
    dataSize = dims[0] * dims[1] * pixelSize;
    if (nDims == 3) dataSize *= dims[2];
    dataSizePG = pPGImage->GetDataSize();
    // Note, we should be testing for equality here.  However, there appears to be a bug in the
    // SDK when images are converted.  When converting from raw8 to mono8, for example, the
    // size returned by GetDataSize is the size of an RGB8 image, not a mono8 image.
    if (dataSize > dataSizePG) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: data size mismatch: calculated=%lu, reported=%lu\n",
            driverName, functionName, (long)dataSize, (long)dataSizePG);
        //return asynError;
    }
    setIntegerParam(NDArraySizeX, nCols);
    setIntegerParam(NDArraySizeY, nRows);
    setIntegerParam(NDArraySize, (int)dataSize);
    setIntegerParam(NDDataType,dataType);
    if (nDims == 3) {
        colorMode = NDColorModeRGB1;
    } else {
        /* If the color mode is currently set to Bayer leave it alone */
        getIntegerParam(NDColorMode, (int *)&colorMode);
        if (colorMode != NDColorModeBayer) colorMode = NDColorModeMono;
    }
    setIntegerParam(NDColorMode, colorMode);

    pRaw_ = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
    if (!pRaw_) {
        /* If we didn't get a valid buffer from the NDArrayPool we must abort
         * the acquisition as we have nowhere to dump the data...       */
        setIntegerParam(ADStatus, ADStatusAborting);
        callParamCallbacks();
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s [%s] ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
            driverName, functionName, portName);
        setIntegerParam(ADAcquire, 0);
        return(asynError);
    }
    pData = pPGImage->GetData();
    memcpy(pRaw_->pData, pData, dataSize);

    /* Put the frame number into the buffer */
    pRaw_->uniqueId = metaData.embeddedFrameCounter;
    getIntegerParam(PGTimeStampMode, &timeStampMode);
    updateTimeStamp(&pRaw_->epicsTS);
    /* Set the timestamps in the buffer */
    switch (timeStampMode) {
        case TimeStampCamera:
            // Some Point Grey cameras return seconds and microseconds, others cycleSeconds, etc. fields
            if (timeStamp.seconds != 0) {
                pRaw_->timeStamp = (double)timeStamp.seconds + 
                                   (double)timeStamp.microSeconds / 1.e6;
            } else {
                pRaw_->timeStamp = (double)timeStamp.cycleSeconds + 
                                   (double)timeStamp.cycleCount / 8000. + 
                                   (double)timeStamp.cycleOffset / 8000. / 3072.;
            }
            break;
        case TimeStampEPICS:
            pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;
            break;
        case TimeStampHybrid:
            // For now we just use EPICS time
            pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch + pRaw_->epicsTS.nsec/1e9;
            break;
    }
    
    /* Get any attributes that have been defined for this driver */        
    getAttributes(pRaw_->pAttributeList);
    
    /* Change the status to be readout... */
    setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();

    pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);

    return status;
}


/** Sets an int32 parameter.
  * \param[in] pasynUser asynUser structure that contains the function code in pasynUser->reason. 
  * \param[in] value The value for this parameter 
  *
  * Takes action if the function code requires it.  ADAcquire, ADSizeX, and many other
  * function codes make calls to the Firewire library from this function. */
asynStatus pointGrey::writeInt32( asynUser *pasynUser, epicsInt32 value)
{
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    int addr;
    PropertyType propType;
    static const char *functionName = "writeInt32";

    pasynManager->getAddr(pasynUser, &addr);
    if (addr < 0) addr=0;
    propType = (PropertyType) addr;

    /* Set the value in the parameter library.  This may change later but that's OK */
    status = setIntegerParam(addr, function, value);

    if (function == ADAcquire) {
        if (value) {
            /* start acquisition */
            status = startCapture();
        } else {
            status = stopCapture();
        }

    } else if ( (function == ADSizeX)       ||
                (function == ADSizeY)       ||
                (function == ADMinX)        ||
                (function == ADMinY)        ||
                (function == PGFormat7Mode) ||
                (function == PGPixelFormat) ||
                (function == PGBinningMode) ||
                (function == PGPacketSize)  ||
                (function == PGPacketDelay)) {
        status = setImageParams();

    } else if (function == PGPropertyValue) {
        status = setPropertyValue(propType, value, propValueA);

    } else if (function == PGPropertyValueB) {
        status = setPropertyValue(propType, value, propValueB);

    } else if (function == PGPropertyAutoMode) {
        status = setPropertyAutoMode(propType, value);

    } else if (function == PGPropertyOnOff) {
        status = setPropertyOnOff(propType, value);

    } else if (function == PGPropertyOnePush) {
        status = setPropertyOnePush(propType);

    } else if (function == PGVideoMode) {
        status = setVideoMode(value);

    } else if (function == PGFrameRate) {
        status = setFrameRate(value);

    } else if ((function == ADTriggerMode)  || 
               (function == ADNumImages)    ||
               (function == ADNumExposures) ||
               (function == PGSkipFrames)) {
        status = setTrigger();
        
    } else if (function == PGSoftwareTrigger) {
        status = softwareTrigger();

    } else if ((function == PGStrobeSource) || 
               (function == PGStrobeEnable) ||
               (function == PGStrobePolarity)) {
        status = setStrobe();
                
    } else if (function == ADReadStatus) {
        status = readStatus();

    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PG_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, 
        "%s::%s function=%d, addr=%d, value=%d, status=%d\n",
        driverName, functionName, function, addr, value, status);
            
    callParamCallbacks(addr);
    return status;
}

/** Sets an float64 parameter.
  * \param[in] pasynUser asynUser structure that contains the function code in pasynUser->reason. 
  * \param[in] value The value for this parameter 
  *
  * Takes action if the function code requires it.  The PGPropertyValueAbs
  * function code makes calls to the Firewire library from this function. */
asynStatus pointGrey::writeFloat64( asynUser *pasynUser, epicsFloat64 value)
{
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    int addr;
    PropertyType propertyType;
    static const char *functionName = "writeFloat64";
    
    pasynManager->getAddr(pasynUser, &addr);
    if (addr < 0) addr=0;
    propertyType = (PropertyType)addr;

    /* Set the value in the parameter library.  This may change later but that's OK */
    status = setDoubleParam(addr, function, value);

    if (function == PGPropertyValueAbs) {
        status = setPropertyAbsValue(propertyType, value);
    
    } else if (function == ADAcquireTime) {
        propertyType = SHUTTER;
        // Camera units are ms
        status = setPropertyAbsValue(propertyType, value*1000.);
    
    } else if (function == ADGain) {
        propertyType = GAIN;
        status = setPropertyAbsValue(propertyType, value);
            
    } else if (function == ADAcquirePeriod) {
        propertyType = FRAME_RATE;
        // Camera units are fps
        status = setPropertyAbsValue(propertyType, 1./value);
    
    } else if ((function == PGStrobeDelay)  || 
               (function == PGStrobeDuration)) {
        status = setStrobe();
        
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PG_PARAM) status = ADDriver::writeFloat64(pasynUser, value);
    }

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s::%s function=%d, addr=%d, value=%f, status=%d\n",
        driverName, functionName, function, addr, value, status);
    callParamCallbacks(addr);
    return status;
}


asynStatus pointGrey::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                               size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    enumStruct_t *pEnum;
    int numEnums;
    int i;

    if (function == PGVideoMode) {
        pEnum = videoModeEnums_;
        numEnums = numValidVideoModes_;
    } else if (function == PGFormat7Mode) {
        pEnum = format7ModeEnums_;
        numEnums = numValidFormat7Modes_;
    } else if (function == PGBinningMode) {
        pEnum = binningModeEnums_;
        numEnums = numValidBinningModes_;
    } else if (function == PGFrameRate) {
        pEnum = frameRateEnums_;
        numEnums = numValidFrameRates_;
    } else if (function == PGPixelFormat) {
        pEnum = pixelFormatEnums_;
        numEnums = numValidPixelFormats_;
    } else if (function == PGConvertPixelFormat) {
        pEnum = convertPixelFormatEnums_;
        numEnums = numValidConvertPixelFormats_;
    } else if (function == ADTriggerMode) {
        pEnum = triggerModeEnums_;
        numEnums = numValidTriggerModes_;
    } else if (function == PGTriggerSource) {
        pEnum = triggerSourceEnums_;
        numEnums = numValidTriggerSources_;
    } else if (function == PGStrobeSource) {
        pEnum = strobeSourceEnums_;
        numEnums = numValidStrobeSources_;
    } else {
        *nIn = 0;
        return asynError;
    }

    for (i=0; ((i<numEnums) && (i<(int)nElements)); i++) {
        if (strings[i]) free(strings[i]);
        strings[i] = epicsStrDup(pEnum->string);
        values[i] = pEnum->value;
        severities[i] = 0;
        pEnum++;
    }
    *nIn = i;
    return asynSuccess;   
}

asynStatus pointGrey::setPropertyAutoMode(PropertyType propType, int value)
{
    Error error;
    Property *pProperty = allProperties_[propType];
    PropertyInfo *pPropInfo = allPropInfos_[propType];
    static const char *functionName = "setPropertyAutoMode";
    
    /* First check if the propertyType is valid for this camera */
    if (!pProperty->present) return asynError;

    /* Check if the desired mode is even supported by the camera on this propertyType */
    if (value == 0) {
        if (!pPropInfo->manualSupported) return asynError;
    } else {
        if (!pPropInfo->autoSupported) return asynError;
    }

    /* Send the propertyType mode to the camera */
    pProperty->autoManualMode = value ? true : false;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::SetProperty, pCameraBase_=%p, pProperty=%p, pProperty->type=%d, pProperty->autoManualMode=%d\n",
        driverName, functionName, pCameraBase_, pProperty, pProperty->type, pProperty->autoManualMode);
    error = pCameraBase_->SetProperty(pProperty);
    if (checkError(error, functionName, "SetProperty")) 
        return asynError;
    /* Update all properties to see if any settings have changed */
    getAllProperties();
    return asynSuccess;
}


asynStatus pointGrey::setPropertyOnOff(PropertyType propType, int onOff)
{
    Error error;
    Property *pProperty = allProperties_[propType];
    PropertyInfo *pPropInfo = allPropInfos_[propType];
    static const char *functionName = "setPropertyAutoMode";
    
    /* First check if the propertyType is valid for this camera */
    if (!pProperty->present) return asynError;

    /* Check if onOff is supported by the camera on this propertyType */
    if (!pPropInfo->onOffSupported) return asynError;

    /* Send the onOff mode to the camera */
    pProperty->onOff = onOff ? true : false;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::SetProperty, pCameraBase_=%p, pProperty=%p, pProperty->type=%d, pProperty->onOff=%d\n",
        driverName, functionName, pCameraBase_, pProperty, pProperty->type, pProperty->onOff);
    error = pCameraBase_->SetProperty(pProperty);
    if (checkError(error, functionName, "SetProperty")) 
        return asynError;
    /* Update all properties to see if any settings have changed */
    getAllProperties();
    return asynSuccess;
}


asynStatus pointGrey::setPropertyOnePush(PropertyType propType)
{
    Error error;
    Property *pProperty = allProperties_[propType];
    PropertyInfo *pPropInfo = allPropInfos_[propType];
    static const char *functionName = "setPropertyAutoMode";
    
    /* First check if the propertyType is valid for this camera */
    if (!pProperty->present) return asynError;

    /* Check if onePush is supported by the camera on this propertyType */
    if (!pPropInfo->onePushSupported) return asynError;

    /* Send the onePush to the camera */
    pProperty->onePush = true;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::SetProperty, pCameraBase_=%p, pProperty=%p, pProperty->type=%d, pProperty->onePush=%d\n",
        driverName, functionName, pCameraBase_, pProperty, pProperty->type, pProperty->onePush);
    error = pCameraBase_->SetProperty(pProperty);
    /* Reset onePush flag */
    pProperty->onePush = false;
    if (checkError(error, functionName, "SetProperty")) 
        return asynError;
    /* Update all properties to see if any settings have changed */
    getAllProperties();
    return asynSuccess;
}



asynStatus pointGrey::setPropertyValue(PropertyType propType, int value, propValue_t valType)
{
    Error error;
    Property *pProperty = allProperties_[propType];
    PropertyInfo *pPropInfo = allPropInfos_[propType];
    static const char *functionName = "setPropertyValue";

    /* First check if the propertyType is valid for this camera */
    if (!pProperty->present) return asynError;

    /* Enable control for this propertyType */
    pProperty->onOff = true;

    /* Disable absolute mode control for this propertyType */
    pProperty->absControl = false;

    /* Disable automatic mode control for this propertyType */
    pProperty->autoManualMode = false;

    /* Check the value is within the expected boundaries */
    if (value < (int)pPropInfo->min || value > (int)pPropInfo->max) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s error setting propertyType %s, value %d is out of range [%d..%d]\n",
            driverName, functionName, propertyTypeStrings[propType], value, pPropInfo->min, pPropInfo->max);
        return asynError;
    }

    if (valType == propValueA) {
        pProperty->valueA = value;
    } else {
        pProperty->valueB = value;
    }
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::SetProperty, pCameraBase_=%p, pProperty=%p, pProperty->type=%d, pProperty->valueA=%d\n",
        driverName, functionName, pCameraBase_, pProperty, pProperty->type, pProperty->valueA);
    error = pCameraBase_->SetProperty(pProperty);
    if (checkError(error, functionName, "SetProperty")) 
        return asynError;

    /* Update all properties to see if any settings have changed */
    getAllProperties();
    return asynSuccess;
}


asynStatus pointGrey::setGigEPropertyValue(GigEPropertyType propType, int value)
{
    Error error;
    GigEProperty *pProperty = allGigEProperties_[propType];
    static const char *functionName = "setGigEPropertyValue";

    if (pGigECamera_ == NULL) return asynSuccess;
    
    /* First check if the propertyType is writeable for this camera */
    if (!pProperty->isWritable) return asynError;

    /* Check the value is within the expected boundaries */
    if (value < (int)pProperty->min || value > (int)pProperty->max) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s error setting propertyType %s, value %d is out of range [%d..%d]\n",
            driverName, functionName, gigEPropertyTypeStrings[propType], value, pProperty->min, pProperty->max);
        return asynError;
    }

    pProperty->value = value;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling GigECamera::SetGigEProperty, pGigECamera_=%p, pProperty=%p, pProperty->propType=%d, pProperty->value=%d\n",
        driverName, functionName, pGigECamera_, pProperty, pProperty->propType, pProperty->value);
    error = pGigECamera_->SetGigEProperty(pProperty);
    if (checkError(error, functionName, "SetGigeEProperty")) 
        return asynError;

    /* Update all properties to see if any settings have changed */
    getAllProperties();
    getAllGigEProperties();
    return asynSuccess;
}


asynStatus pointGrey::setPropertyAbsValue(PropertyType propType, epicsFloat64 value)
{
    Error error;
    Property *pProperty = allProperties_[propType];
    PropertyInfo *pPropInfo = allPropInfos_[propType];
    static const char *functionName = "setPropertyAbsValue";

    /* First check if the propertyType is valid for this camera */
    if (!pProperty->present) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s error setting propertyType %s: property is not supported by this camera\n",
            driverName, functionName, propertyTypeStrings[propType]);
        return asynError;
    }

    /* Disable automatic mode control for this propertyType */
    pProperty->autoManualMode = false;

   /* Check if the specific propertyType supports absolute values */
    if (!pPropInfo->absValSupported) { 
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s error setting propertyType %s: No absolute control for this propertyType\n",
            driverName, functionName, propertyTypeStrings[propType]);
        return asynError;
    }

    /* Enable control for this propertyType */
    pProperty->onOff = true;

     /* Enable absolute mode control for this propertyType */
    pProperty->absControl = true;

    /* Check the value is within the expected boundaries */
    if (value < pPropInfo->absMin || value > pPropInfo->absMax) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s error setting propertyType %s, value %.5f is out of range [%.3f..%.3f]\n",
            driverName, functionName, propertyTypeStrings[propType], value, pPropInfo->absMin, pPropInfo->absMax);
        return asynError;
    }

    /* Set the propertyType value in the camera */
    pProperty->absValue = (float)value;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::SetProperty, pCameraBase_=%p, pProperty=%p, pProperty->type=%d, pProperty->absValue=%f\n",
        driverName, functionName, pCameraBase_, pProperty, pProperty->type, pProperty->absValue);
    error = pCameraBase_->SetProperty(pProperty);
    if (checkError(error, functionName, "SetProperty")) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s error setting propertyType %s, value %.5f\n",
            driverName, functionName, propertyTypeStrings[propType], value);      
        return asynError;
    }

    /* Update all properties to see if any settings have changed */
    getAllProperties();
    return asynSuccess;
}

asynStatus pointGrey::setVideoMode(int mode)
{
    int frameRate;
    /* Get the frame rate */
    getIntegerParam(PGFrameRate, &frameRate);
    return setVideoModeAndFrameRate(mode, frameRate);
}
 
asynStatus pointGrey::setFrameRate(int frameRate)
{
    int videoMode;
    /* Get the video mode */
    getIntegerParam(PGVideoMode, &videoMode);
    return setVideoModeAndFrameRate(videoMode, frameRate);
}
 
asynStatus pointGrey::setVideoModeAndFrameRate(int videoModeIn, int frameRateIn)
{
    asynStatus status = asynSuccess;
    Error error;
    bool resumeAcquire;
    FrameRate frameRate = (FrameRate)frameRateIn;
    VideoMode videoMode = (VideoMode)videoModeIn;
    bool supported;
    static const char *functionName = "setVideoModeAndFrameRate";

    if (pCamera_ == NULL) return asynError;

    // VIDEOMODE_FORMAT7 must be treated differently
    if (videoMode == VIDEOMODE_FORMAT7) {
        setImageParams();
    } else {
        /* Must stop capture before changing the video mode */
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling Camera::StopCapture, pCamera_=%p\n",
            driverName, functionName, pCamera_);
        error = pCamera_->StopCapture();
        resumeAcquire = (error == PGRERROR_OK);
        /* Attempt to write the video mode to camera */
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling Camera::SetVideoModeAndFrameRate, pCamera_=%p, videoMode=%d, frameRate=%d\n",
            driverName, functionName, pCamera_, videoMode, frameRate);
        error = pCamera_->SetVideoModeAndFrameRate(videoMode, frameRate);
        checkError(error, functionName, "SetVideoModeAndFrameRate");
        if (resumeAcquire) {
            asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                "%s::%s calling Camera::StartCapture, pCamera_=%p\n",
                driverName, functionName, pCamera_);
            error = pCamera_->StartCapture();
            checkError(error, functionName, "StartCapture");
        }
        error = pCamera_->GetVideoModeAndFrameRateInfo(videoMode, frameRate, &supported);
        if (checkError(error, functionName, "GetVideoModeAndFrameRateInfo")) 
            return asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s called Camera::GetVideoModeAndFrameRateInfo, pCamera_=%p, videoMode=%d, frameRate=%d, supported=%d\n",
            driverName, functionName, pCamera_, videoMode, frameRate, supported);
        if (!supported) {
             asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
                "%s::%s camera does not support mode %d with framerate %d\n",
                driverName, functionName, videoMode, frameRate);
            status = asynError;
            return asynError;
        }
        /* When the video mode changes the supported values of frame rate change */
        createDynamicEnums();
        /* When the video mode changes the available properties can also change */
        getAllProperties();
    }

    return status;
}

int pointGrey::getPixelFormatIndex(PixelFormat pixelFormat)
{
    int i;
    // Find the pixel format index corresponding to the actual pixelFormat
    for(i=0; i<(int)NUM_PIXEL_FORMATS; i++) {
        if (pixelFormatValues[i] == pixelFormat) return i;
    }
    return -1;
}

asynStatus pointGrey::setImageParams()
{
    static const char *functionName = "setImageParams";
    
    if (pCamera_) 
        return setFormat7Params();
    
    else if (pGigECamera_) 
        return setGigEImageParams();
    
    else
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
           "%s::%s unknown camera type\n",
           driverName, functionName);
    return asynError;
}


asynStatus pointGrey::setFormat7Params()
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
    int itemp;
    float percentage;
    unsigned int packetSize;
    unsigned int packetSizeActual;
    int sizeX, sizeY, minX, minY;
    unsigned short hsMax, vsMax, hsUnit, vsUnit;
    unsigned short hpMax, vpMax, hpUnit, vpUnit;
    static const char *functionName = "setFormat7Params";

    if (!pCamera_) return asynError;
    
    /* Get the current video mode from EPICS.  It may have just been changed */
    getIntegerParam(PGVideoMode, &videoMode);
    /* If not format 7 then silently exit */
    if (videoMode != VIDEOMODE_FORMAT7) return asynSuccess;
    
    getIntegerParam(PGFormat7Mode, &f7Mode);
    /* Get the format7 info */
    pFormat7Info_->mode = (Mode)f7Mode;
    error = pCamera_->GetFormat7Info(pFormat7Info_, &supported);
    if (checkError(error, functionName, "GetFormat7Info")) 
        return asynError;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s called Camera::GetFormat7Info, pCamera_=%p, pFormat7Info_=%p, supported=%d\n",
        driverName, functionName, pCamera_, pFormat7Info_, supported);
    if (!supported) {
         asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s::%s camera does not support mode Format7 mode %d\n",
            driverName, functionName, f7Mode);
       return asynError;
    }
    
    getIntegerParam(ADSizeX, &sizeX);
    getIntegerParam(ADSizeY, &sizeY);
    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADMinY, &minY);
    getIntegerParam(PGPixelFormat, &itemp);
    pixelFormat = (PixelFormat)itemp;

    /* Set the size limits */
    hsMax = pFormat7Info_->maxWidth;
    vsMax = pFormat7Info_->maxHeight;
    setIntegerParam(ADSizeX, hsMax);
    setIntegerParam(ADSizeY, vsMax);
    /* Set the size units (minimum increment) */
    hsUnit = pFormat7Info_->imageHStepSize;
    vsUnit = pFormat7Info_->imageVStepSize;
    /* Set the offset units (minimum increment) */
    hpUnit = pFormat7Info_->offsetHStepSize;
    vpUnit = pFormat7Info_->offsetVStepSize;
    
    // This logic probably needs work!!!
    hpMax = hsMax;
    vpMax = vsMax;
 
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, 
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
 
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling Camera::ValidateFormat7Settings\n"
        "  pCamera_=%p, &f7Settings=%p, &f7SettingsValid=%p, &f7PacketInfo=%p\n"
        "  f7Settings: mode=%d, offsetX=%d, offsetY=%d, width=%d, height=%d, pixelFormat=0x%x\n",
        driverName, functionName, pCamera_, &f7Settings, &f7SettingsValid, &f7PacketInfo,
        f7Settings.mode, f7Settings.offsetX, f7Settings.offsetY, f7Settings.width, f7Settings.height, f7Settings.pixelFormat);
    error = pCamera_->ValidateFormat7Settings(&f7Settings, &f7SettingsValid, &f7PacketInfo);
    if (checkError(error, functionName, "ValidateFormat7Settings")) 
        return asynError;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s Camera::ValidateFormat7Settings returned f7SettingsValid=%d\n",
        driverName, functionName, f7SettingsValid);
    if (!f7SettingsValid) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s ERROR ValidateFormat7Settings returned false\n",
            driverName, functionName);
        return asynError;
    }    

    /* Attempt to write the parameters to camera */
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, 
        "%s::%s setting format 7 parameters sizeX=%d, sizeY=%d, minX=%d, minY=%d, pixelFormat=0x%x\n",
        driverName, functionName, sizeX, sizeY, minX, minY, pixelFormat);

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
        "%s:%s bytes per packet: min=%d, max=%d, recommended=%d, actually setting=%d\n", 
        driverName, functionName, f7PacketInfo.unitBytesPerPacket, f7PacketInfo.maxBytesPerPacket, 
        f7PacketInfo.recommendedBytesPerPacket, f7PacketInfo.recommendedBytesPerPacket);

    /* Must stop acquisition before changing the video mode */
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling Camera::StopCapture, pCamera_=%p\n",
        driverName, functionName, pCamera_);
    error = pCamera_->StopCapture();
    resumeAcquire = (error == PGRERROR_OK);
    getIntegerParam(PGPacketSize, &itemp);
    packetSize = itemp;
    setIntegerParam(PGMaxPacketSize, pFormat7Info_->maxPacketSize);
    if (packetSize <= 0) {
        packetSize = f7PacketInfo.recommendedBytesPerPacket;
    } else {
        // Packet size must be a multiple of unitBytesPerPacket
        packetSize = (packetSize/f7PacketInfo.unitBytesPerPacket) * f7PacketInfo.unitBytesPerPacket;
        if (packetSize < pFormat7Info_->minPacketSize) packetSize = pFormat7Info_->minPacketSize;
        if (packetSize > pFormat7Info_->maxPacketSize) packetSize = pFormat7Info_->maxPacketSize;
    }
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling Camera::SetFormat7Configuration\n"
        "  pCamera_=%p, &f7Settings=%p, packetSize=%d\n"
        "  f7Settings: mode=%d, offsetX=%d, offsetY=%d, width=%d, height=%d, pixelFormat=0x%x\n",
        driverName, functionName, pCamera_, &f7Settings, packetSize,
        f7Settings.mode, f7Settings.offsetX, f7Settings.offsetY, f7Settings.width, f7Settings.height, f7Settings.pixelFormat);
    error = pCamera_->SetFormat7Configuration(&f7Settings, packetSize);
    checkError(error, functionName, "SetFormat7Configuration");
    if (resumeAcquire) {
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling Camera::StartCapture, pCamera_=%p\n",
            driverName, functionName, pCamera_);
        error = pCamera_->StartCapture();
        checkError(error, functionName, "StartCapture");
    }

    /* Read back the actual values */
    error = pCamera_->GetFormat7Configuration(&f7Settings, &packetSizeActual, &percentage);
    if (checkError(error, functionName, "GetFormat7Configuration")) 
        return asynError;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s called Camera::GetFormat7Configuration\n"
        "  pCamera_=%p, &f7Settings=%p, packetSizeActual=%d, percentage=%f\n"
        "  f7Settings: mode=%d, offsetX=%d, offsetY=%d, width=%d, height=%d, pixelFormat=0x%x\n",
        driverName, functionName, pCamera_, &f7Settings, packetSizeActual, percentage,
        f7Settings.mode, f7Settings.offsetX, f7Settings.offsetY, f7Settings.width, f7Settings.height, f7Settings.pixelFormat);
    setIntegerParam(ADMinX,        f7Settings.offsetX);
    setIntegerParam(ADMinY,        f7Settings.offsetY);
    setIntegerParam(ADSizeX,       f7Settings.width);
    setIntegerParam(ADSizeY,       f7Settings.height);
    setIntegerParam(PGPixelFormat, f7Settings.pixelFormat);
    setIntegerParam(PGPacketSizeActual, packetSizeActual);
    callParamCallbacks();
    
    /* When the format7 mode changes the supported values of pixel format changes */
    createDynamicEnums();
    /* When the format7 mode changes the available properties can also change */
    getAllProperties();

    return asynSuccess;
}

asynStatus pointGrey::setGigEImageParams()
{
    Error error;
    GigEImageSettings gigESettings;
    PixelFormat pixelFormat;
    bool resumeAcquire;
    Mode gigEMode;
    int itemp;
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

    if (!pGigECamera_) return asynError;

    /* Must stop acquisition before changing these settings */
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling GigECamera::StopCapture, pGigECamera_=%p\n",
        driverName, functionName, pGigECamera_);
    error = pGigECamera_->StopCapture();
    resumeAcquire = (error == PGRERROR_OK);

    // Set the GigE imaging mode    
    getIntegerParam(PGFormat7Mode, &itemp);
    gigEMode = (Mode)itemp;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling GigECamera::SetGigEImagingMode, pGigECamera_=%p, gigEMode=%d\n",
        driverName, functionName, pGigECamera_, gigEMode);
    error = pGigECamera_->SetGigEImagingMode(gigEMode);
    if (checkError(error, functionName, "SetGigEImagingMode")) 
        goto cleanup;

    /* Get the GigE image settings info */
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling GigECamera::GetGigEImageSettingsInfo, pGigECamera_=%p, pGigEImageSettingsInfo_=%p\n",
        driverName, functionName, pGigECamera_, pGigEImageSettingsInfo_);
    error = pGigECamera_->GetGigEImageSettingsInfo(pGigEImageSettingsInfo_);
    if (checkError(error, functionName, "GetGigEImageSettingsInfo")) 
        goto cleanup;
    
    getIntegerParam(ADSizeX, &sizeX);
    getIntegerParam(ADSizeY, &sizeY);
    getIntegerParam(ADMinX, &minX);
    getIntegerParam(ADMinY, &minY);
    getIntegerParam(PGPixelFormat, &itemp);
    pixelFormat = (PixelFormat)itemp;

    /* Set the size limits */
    hsMax = pGigEImageSettingsInfo_->maxWidth;
    vsMax = pGigEImageSettingsInfo_->maxHeight;
    setIntegerParam(ADSizeX, hsMax);
    setIntegerParam(ADSizeY, vsMax);
    /* Set the size units (minimum increment) */
    hsUnit = pGigEImageSettingsInfo_->imageHStepSize;
    vsUnit = pGigEImageSettingsInfo_->imageVStepSize;
    /* Set the offset units (minimum increment) */
    hpUnit = pGigEImageSettingsInfo_->offsetHStepSize;
    vpUnit = pGigEImageSettingsInfo_->offsetVStepSize;
    
    // This logic probably needs work!!!
    hpMax = hsMax;
    vpMax = vsMax;
 
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, 
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
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, 
        "%s::%s setting GigE parameters width=%d, height=%d, offsetX=%d, offsetY=%d, pixelFormat=0x%x\n",
        driverName, functionName, 
        gigESettings.width, gigESettings.height, 
        gigESettings.offsetX, gigESettings.offsetY, gigESettings.pixelFormat);

    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling GigECamera::SetGigEImageSettings, pGigECamera_=%p, &gigESettings=%p\n",
        driverName, functionName, pGigECamera_, &gigESettings);
    error = pGigECamera_->SetGigEImageSettings(&gigESettings);
    if (checkError(error, functionName, "SetGigEImageSettings"))
        goto cleanup;

    // Set the packet size and delay
    getIntegerParam(PACKET_SIZE, PGGigEPropertyValueMin, &minPacketSize);
    getIntegerParam(PGMaxPacketSize, &maxPacketSize);
    getIntegerParam(PGPacketSize, &packetSize);
    if (packetSize <= 0) packetSize = maxPacketSize;
    if (packetSize < minPacketSize) packetSize = minPacketSize;
    if (packetSize > maxPacketSize) packetSize = maxPacketSize;
    setGigEPropertyValue(PACKET_SIZE, packetSize);
    setIntegerParam(PGPacketSizeActual, packetSize);

    getIntegerParam(PGPacketDelay, &packetDelay);
    getIntegerParam(PACKET_DELAY, PGGigEPropertyValueMin, &minPacketDelay);
    getIntegerParam(PACKET_DELAY, PGGigEPropertyValueMax, &maxPacketDelay);
    if (packetDelay < minPacketDelay) packetDelay = minPacketDelay;
    if (packetDelay > maxPacketDelay) packetDelay = maxPacketDelay;
    setGigEPropertyValue(PACKET_DELAY, packetDelay);

    // Set the binning
    getIntegerParam(PGBinningMode, &binningMode);
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling GigECamera::SetGigEImageBinningSettings, pGigECamera_=%p, binX=%d, binY=%d\n",
        driverName, functionName, pGigECamera_, binningMode, binningMode);
    error = pGigECamera_->SetGigEImageBinningSettings(binningMode, 
                                                      binningMode);
    if (checkError(error, functionName, "SetGigEImageBinningSettings")) 
        goto cleanup;

    /* Read back the actual values */
    error = pGigECamera_->GetGigEImagingMode(&gigEMode);
    if (checkError(error, functionName, "GetGigEImagingMode")) 
        goto cleanup;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s called GigECamera::GetGigEImagingMode, pGigECamera_=%p, gigEMode=%d\n",
        driverName, functionName, pGigECamera_, gigEMode);
    setIntegerParam(PGFormat7Mode, gigEMode);

    error = pGigECamera_->GetGigEImageBinningSettings(&binX, &binY); 
    if (checkError(error, functionName, "GetGigEImageBinningSettings")) 
        goto cleanup;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s called GigECamera::GetGigEImageBinningSettings, pGigECamera_=%p, binX=%d, binY=%d\n",
        driverName, functionName, pGigECamera_, binX, binY);
    setIntegerParam(PGBinningMode, binX);
    setIntegerParam(ADBinX, binX);
    setIntegerParam(ADBinY, binY);

    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling GigECamera::GetGigEImageSettings, pGigECamera_=%p, &gigESettings=%p\n",
        driverName, functionName, pGigECamera_, &gigESettings);
    error = pGigECamera_->GetGigEImageSettings(&gigESettings);
    if (checkError(error, functionName, "GetGigEImageSettings")) 
        goto cleanup;

    setIntegerParam(ADMinX,        gigESettings.offsetX);
    setIntegerParam(ADMinY,        gigESettings.offsetY);
    setIntegerParam(ADSizeX,       gigESettings.width);
    setIntegerParam(ADSizeY,       gigESettings.height);
    setIntegerParam(PGPixelFormat, gigESettings.pixelFormat);
    callParamCallbacks();
    
    /* When the GigE mode changes the supported values of pixel format changes */
    createDynamicEnums();
    /* When the GigE mode changes the available properties can also change */
    getAllProperties();
    getAllGigEProperties();

cleanup:
    if (resumeAcquire) {
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling GigECamera::StartCapture, pGigECamera_=%p\n",
            driverName, functionName, pGigECamera_);
        error = pGigECamera_->StartCapture();
        checkError(error, functionName, "StartCapture");
    }

    if (error != PGRERROR_OK) return asynError;
    return asynSuccess;
}


asynStatus pointGrey::setTrigger()
{
    int numImages;
    int numExposures;
    int triggerMode;
    int triggerPolarity;
    int triggerSource;
    int skipFrames;
    Error error;
    static const char *functionName = "setTrigger";
    
    getIntegerParam(ADTriggerMode, &triggerMode);
    getIntegerParam(PGTriggerPolarity, &triggerPolarity);
    getIntegerParam(PGTriggerSource, &triggerSource);
    error = pCameraBase_->GetTriggerMode(pTriggerMode_);
    if (checkError(error, functionName, "GetTriggerMode")) 
        return asynError;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s called CameraBase::GetTriggerMode, pCameraBase_=%p, pTriggerMode_=%p\n",
        driverName, functionName, pCameraBase_, pTriggerMode_);
    if (triggerMode == ADTriggerInternal) {
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
                getIntegerParam(PGSkipFrames, &skipFrames);
                pTriggerMode_->parameter = skipFrames;
                break;
            case 4:
            case 5:
                getIntegerParam(ADNumExposures, &numExposures);
                pTriggerMode_->parameter = numExposures;
                break;
            case 15:
                getIntegerParam(ADNumImages, &numImages);
                pTriggerMode_->parameter = numImages;
                break;
        }
    }
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::SetTriggerMode, pCameraBase_=%p, pTriggerMode_=%p\n"
        "onOff=%d, polarity=%d, source=%d, mode=%d, parameter=%d\n",
        driverName, functionName, pCameraBase_, pTriggerMode_,
        pTriggerMode_->onOff, pTriggerMode_->polarity, pTriggerMode_->source, 
        pTriggerMode_->mode, pTriggerMode_->parameter);
    error = pCameraBase_->SetTriggerMode(pTriggerMode_);
    if (checkError(error, functionName, "SetTriggerMode")) 
        return asynError;
    /* When the trigger mode changes the properties can also change */
    getAllProperties();
    return asynSuccess;
}

asynStatus pointGrey::softwareTrigger()
{
    Error error;
    static const char *functionName = "softwareTrigger";
    
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::FireSoftwareTrigger, pCameraBase_=%p\n",
        driverName, functionName, pCameraBase_);
    error = pCameraBase_->FireSoftwareTrigger();
    if (checkError(error, functionName, "FireSoftwareTrigger")) 
        return asynError;
    return asynSuccess;
}

asynStatus pointGrey::setStrobe()
{
    int polarity;
    int source;
    int enable;
    double delay;
    double duration;
    Error error;
    static const char *functionName = "setStrobe";

    getIntegerParam(PGStrobeSource,   &source);
    getIntegerParam(PGStrobeEnable,   &enable);
    getIntegerParam(PGStrobePolarity, &polarity);
    getDoubleParam(PGStrobeDelay,     &delay);
    getDoubleParam(PGStrobeDuration,  &duration);
    pStrobeControl_->source   = source;
    pStrobeControl_->onOff    = enable ? true : false;
    pStrobeControl_->polarity = polarity;
    pStrobeControl_->delay    = (float)(delay*1000.);
    pStrobeControl_->duration = (float)(duration*1000.);
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::SetStrobe, pCameraBase_=%p, pStrobeControl_=%p\n"
        "  source=%d, onOff=%d, polarity=%d, delay=%f, duration=%f\n",
        driverName, functionName, pCameraBase_, pStrobeControl_,
        pStrobeControl_->source, pStrobeControl_->onOff, pStrobeControl_->polarity, 
        pStrobeControl_->delay, pStrobeControl_->duration);
    error = pCameraBase_->SetStrobe(pStrobeControl_);
    if (checkError(error, functionName, "SetStrobe")) 
        return asynError;
    return asynSuccess;
}

asynStatus pointGrey::createStaticEnums()
{
    /* This function creates enum strings and values for all enums that are fixed for a given camera.
     * It is only called once at startup */
    int mode, rate, shift, pin, format;
    Error error;
    VideoMode videoMode;
    FrameRate frameRate;
    enumStruct_t *pEnum;
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
                    return asynError;
                asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                    "%s::%s called Camera::GetVideoModeAndFrameRateInfo, pCamera_=%p, videoMode=%d, frameRate%d, supported=%d\n",
                    driverName, functionName, pCamera_, videoMode, frameRate, supported);
                if (supported) modeSupported = true;
            }                  
        }
        if (modeSupported) {
            pEnum = videoModeEnums_ + numValidVideoModes_;
            strcpy(pEnum->string, videoModeStrings[videoMode]);
            pEnum->value = videoMode;
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
                return asynError;
            asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                "%s::%s called Camera::GetFormat7Info, pCamera_=%p, pFormat7Info_=%p, supported=%d\n",
                driverName, functionName, pCamera_, pFormat7Info_, supported);
            if (supported) {
                pEnum = format7ModeEnums_ + numValidFormat7Modes_;
                sprintf(pEnum->string, "%d (%dx%d)", mode, pFormat7Info_->maxWidth, pFormat7Info_->maxHeight);
                pEnum->value = mode;
                numValidFormat7Modes_++;
                if (numValidFormat7Modes_ == 1) {
                    // We assume that the lowest supported mode is the full chip size
                    setIntegerParam(ADMaxSizeX, pFormat7Info_->maxWidth);
                    setIntegerParam(ADMaxSizeY, pFormat7Info_->maxHeight);
                    setIntegerParam(ADSizeX,    pFormat7Info_->maxWidth);
                    setIntegerParam(ADSizeY,    pFormat7Info_->maxHeight);
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
            return asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s called GigECamera::GetGigEImagingMode, pGigECamera_=%p, currentMode=%d\n",
            driverName, functionName, pGigECamera_, currentMode);
        for (mode=0; mode<NUM_MODES; mode++) {
            error = pGigECamera_->QueryGigEImagingMode((Mode)mode, &supported);
            if (checkError(error, functionName, "QueryGigEImagingMode")) 
                return asynError;
            asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                "%s::%s called GigECamera::QueryGigEImagingMode, pGigECamera_=%p, mode=%d, supported=%d\n",
                driverName, functionName, pGigECamera_, mode, supported);
            if (supported) {
                asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                    "%s::%s calling GigECamera::SetGigEImagingMode, pGigECamera_=%p, mode=%d\n",
                    driverName, functionName, pGigECamera_, mode);
                error = pGigECamera_->SetGigEImagingMode((Mode)mode);
                if (checkError(error, functionName, "SetGigEImagingMode"))
                    return asynError;
                asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                    "%s::%s calling GigECamera::GetGigEImageSettingsInfo, pGigECamera_=%p, pGigEImageSettingsInfo_=%p\n",
                    driverName, functionName, pGigECamera_, pGigEImageSettingsInfo_);
                error = pGigECamera_->GetGigEImageSettingsInfo(pGigEImageSettingsInfo_);
                if (checkError(error, functionName, "GetGigEImageSettingsInfo")) 
                    return asynError;
                pEnum = format7ModeEnums_ + numValidFormat7Modes_;
                sprintf(pEnum->string, "%d (%dx%d)", mode, pGigEImageSettingsInfo_->maxWidth, 
                                                           pGigEImageSettingsInfo_->maxHeight);
                pEnum->value = mode;
                numValidFormat7Modes_++;
                if (numValidFormat7Modes_ == 1) {
                    // We assume that the lowest supported mode is the full chip size
                    setIntegerParam(ADMaxSizeX, pGigEImageSettingsInfo_->maxWidth);
                    setIntegerParam(ADMaxSizeY, pGigEImageSettingsInfo_->maxHeight);
                    setIntegerParam(ADSizeX,    pGigEImageSettingsInfo_->maxWidth);
                    setIntegerParam(ADSizeY,    pGigEImageSettingsInfo_->maxHeight);
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
        pEnum = binningModeEnums_ + mode;
        strcpy(pEnum->string, binningModeStrings[mode]);
        pEnum->value = binningModeValues[mode];
    }

    /* Convert pixel format enums */
    numValidConvertPixelFormats_ = NUM_CONVERT_PIXEL_FORMATS;
    for (format=0; format<numValidConvertPixelFormats_; format++) {
        pEnum = convertPixelFormatEnums_ + format;
        strcpy(pEnum->string, convertPixelFormatStrings[format]);
        pEnum->value = convertPixelFormatValues[format];
    }

    /* Trigger mode enums */
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::GetTriggerModeInfo, pCameraBase_=%p, pTriggerModeInfo_=%p\n",
        driverName, functionName, pCameraBase_, pTriggerModeInfo_);
    error = pCameraBase_->GetTriggerModeInfo(pTriggerModeInfo_);
    if (checkError(error, functionName, "GetTriggerModeInfo")) 
        return asynError;
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
            pEnum = triggerModeEnums_ + numValidTriggerModes_;
            strcpy(pEnum->string, triggerModeStrings[mode]);
            pEnum->value = mode;
            numValidTriggerModes_++;
        }    
    }

    /* Trigger source enums */
    numValidTriggerSources_ = 0; 
    for (pin=0; pin<NUM_GPIO_PINS; pin++) {
        shift = NUM_GPIO_PINS - pin - 1;
        supported = ((pTriggerModeInfo_->sourceMask >> shift) & 0x1) == 1;
        if (supported) {
            pEnum = triggerSourceEnums_ + numValidTriggerSources_;
            strcpy(pEnum->string, GPIOStrings[pin]);
            pEnum->value = pin;
            numValidTriggerSources_++;
        }    
    }

    /* Strobe source enums */
    numValidStrobeSources_ = 0; 
    for (pin=0; pin<NUM_GPIO_PINS; pin++) {
        pStrobeInfo_->source = pin;
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling CameraBase::GetStrobeInfo, pCameraBase_=%p, pStrobeInfo_=%p\n",
            driverName, functionName, pCameraBase_, pStrobeInfo_);
        error = pCameraBase_->GetStrobeInfo(pStrobeInfo_);
        if (checkError(error, functionName, "GetStrobeInfo")) 
            return asynError;
        if (pStrobeInfo_->present) {
            pEnum = strobeSourceEnums_ + numValidStrobeSources_;
            strcpy(pEnum->string, GPIOStrings[pin]);
            pEnum->value = pin;
            numValidStrobeSources_++;
        }
    }
    
    return asynSuccess;
}


asynStatus pointGrey::createDynamicEnums()
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
    enumStruct_t *pEnum;
    int i;
    char *enumStrings[NUM_PIXEL_FORMATS];
    int enumValues[NUM_PIXEL_FORMATS];
    int enumSeverities[NUM_PIXEL_FORMATS];
    static const char *functionName = "createDynamicEnums";
 
 
    if (pCamera_) {  
        // This is an IIDC (not GigE) camera 
        /* If the current video mode is format7 then this function creates enum strings and values 
         * for all of the valid pixel formats for the current format7 mode.
         * Otherwise it creates enum strings and values for all valid frame rates for the current video mode. */

        error = pCamera_->GetVideoModeAndFrameRate(&currentVideoMode, &currentFrameRate);
        if (checkError(error, functionName, "GetVideoModeAndFrameRate"))
            return asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s called Camera::GetVideoModeAndFrameRate, pCamera_=%p, currentVideoMode=%d, currentFrameRate=%d\n",
            driverName, functionName, pCamera_, currentVideoMode, currentFrameRate);
        setIntegerParam(PGVideoMode, currentVideoMode);

        if (currentVideoMode == VIDEOMODE_FORMAT7) {
            error = pCamera_->GetFormat7Configuration(&f7Settings, &packetSize, &percentage);
            if (checkError(error, functionName, "GetFormat7Configuration")) 
                return asynError;
            asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                "%s::%s called Camera::GetFormat7Configuration, pCamera_=%p, &f7Settings=%p, packetSize=%d, percentage=%f\n",
                driverName, functionName, pCamera_, &f7Settings, packetSize, percentage);
            pFormat7Info_->mode = f7Settings.mode;
            setIntegerParam(PGFormat7Mode, f7Settings.mode);
            error = pCamera_->GetFormat7Info(pFormat7Info_, &supported);
            if (checkError(error, functionName, "GetFormat7Info")) 
                return asynError;
            asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                "%s::%s calling Camera::GetFormat7Info, pCamera_=%p, pFormat7Info_=%p, supported=%d\n",
                driverName, functionName, pCamera_, pFormat7Info_, supported);
            numValidPixelFormats_ = 0;
            for (format=0; format<(int)NUM_PIXEL_FORMATS; format++) {
                if ((pFormat7Info_->pixelFormatBitField & pixelFormatValues[format]) == pixelFormatValues[format]) {
                    pEnum = pixelFormatEnums_ + numValidPixelFormats_;
                    strcpy(pEnum->string, pixelFormatStrings[format]);
                    pEnum->value = pixelFormatValues[format];
                    numValidPixelFormats_++;
                }
            }
            setIntegerParam(PGPixelFormat, f7Settings.pixelFormat);
            for (i=0; i<numValidPixelFormats_; i++) {
              enumStrings[i] = pixelFormatEnums_[i].string;
              enumValues[i] =  pixelFormatEnums_[i].value;
              enumSeverities[i] = 0;
            }
            doCallbacksEnum(enumStrings, enumValues, enumSeverities, 
                            numValidPixelFormats_, PGPixelFormat, 0);
        } else {
            /* Format all valid frame rates for the current video mode */
            setIntegerParam(PGFrameRate, currentFrameRate);
            numValidFrameRates_ = 0;
            for (rate=0; rate<NUM_FRAMERATES; rate++) {
                frameRate = (FrameRate)rate;
                if (frameRate == FRAMERATE_FORMAT7) continue;
                error = pCamera_->GetVideoModeAndFrameRateInfo(currentVideoMode, frameRate, &supported);
                if (checkError(error, functionName, "GetVideoModeAndFrameRateInfo")) 
                    return asynError;
                asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                    "%s::%s calling Camera::GetVideoModeAndFrameRateInfo, pCamera_=%p, currentVideoMode=%d, frameRate=%d, supported=%d\n",
                    driverName, functionName, pCamera_, currentVideoMode, frameRate, supported);
                if (supported) {
                    pEnum = frameRateEnums_ + numValidFrameRates_;
                    strcpy(pEnum->string, frameRateStrings[rate]);
                    pEnum->value = rate;
                    numValidFrameRates_++;
                } 
            }
            for (i=0; i<numValidFrameRates_; i++) {
              enumStrings[i] = frameRateEnums_[i].string;
              enumValues[i] =  frameRateEnums_[i].value;
              enumSeverities[i] = 0;
            }
            doCallbacksEnum(enumStrings, enumValues, enumSeverities, 
                            numValidFrameRates_, PGFrameRate, 0);
        }
    }
    else if (pGigECamera_) {  
        // This is a GigE camera 
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling GigECamera::GetGigEImageSettings, pGigECamera_=%p, &gigEImageSettings=%p\n",
            driverName, functionName, pGigECamera_, &gigEImageSettings);
        error = pGigECamera_->GetGigEImageSettings(&gigEImageSettings);
        if (checkError(error, functionName, "GetGigEImageSettings")) 
            return asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling GigECamera::GetGigEImageSettingsInfo, pGigECamera_=%p, pGigEImageSettingsInfo_=%p\n",
            driverName, functionName, pGigECamera_, pGigEImageSettingsInfo_);
        error = pGigECamera_->GetGigEImageSettingsInfo(pGigEImageSettingsInfo_);
        if (checkError(error, functionName, "GetGigEImageSettingsInfo")) 
            return asynError;
        numValidPixelFormats_ = 0;
        for (format=0; format<(int)NUM_PIXEL_FORMATS; format++) {
            if ((pGigEImageSettingsInfo_->pixelFormatBitField & 
                     pixelFormatValues[format]) == pixelFormatValues[format]) {
                pEnum = pixelFormatEnums_ + numValidPixelFormats_;
                strcpy(pEnum->string, pixelFormatStrings[format]);
                pEnum->value = pixelFormatValues[format];
                numValidPixelFormats_++;
            }
        }
        setIntegerParam(PGPixelFormat, gigEImageSettings.pixelFormat);
        for (i=0; i<numValidPixelFormats_; i++) {
          enumStrings[i] = pixelFormatEnums_[i].string;
          enumValues[i] =  pixelFormatEnums_[i].value;
          enumSeverities[i] = 0;
        }
        doCallbacksEnum(enumStrings, enumValues, enumSeverities, 
                        numValidPixelFormats_, PGPixelFormat, 0);
    }
    return asynSuccess;
}

/** Read all the propertyType settings and values from the camera.
 * This function will collect all the current values and settings from the camera,
 * and set the appropriate integer/double parameters in the param lib. If a certain propertyType
 * is not available in the given camera, this function will set all the parameters relating to that
 * propertyType to -1 or -1.0 to indicate it is not available.
 * Note the caller is responsible for calling any update callbacks if I/O interrupts
 * are to be processed after calling this function.
 * Returns asynStatus asynError or asynSuccess as an int.
 */
asynStatus pointGrey::getAllProperties()
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
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s calling CameraBase::GetPropertyInfo, pCameraBase_=%p, pPropInfo=%p, propertyType=%d\n",
            driverName, functionName, pCameraBase_, pPropInfo, pProperty->type);
        error = pCameraBase_->GetPropertyInfo(pPropInfo);
        if (checkError(error, functionName, "GetPropertyInfo")) 
            return asynError;
        error = pCameraBase_->GetProperty(pProperty);
        if (checkError(error, functionName, "GetProperty")) 
            return asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s called CameraBase::GetProperty, pCameraBase_=%p, pProperty=%p, pProperty->valueA=%d\n",
            driverName, functionName, pCameraBase_, pProperty, pProperty->valueA);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: checking propertyType %d\n",
            driverName, functionName, addr);

        if (pPropInfo->present) {
            setIntegerParam(addr, PGPropertyAvail,        1);
            setIntegerParam(addr, PGPropertyOnOffAvail,   pPropInfo->onOffSupported);
            setIntegerParam(addr, PGPropertyOnePushAvail, pPropInfo->onePushSupported);
            setIntegerParam(addr, PGPropertyAutoAvail,    pPropInfo->autoSupported);
            setIntegerParam(addr, PGPropertyManAvail,     pPropInfo->manualSupported);
            setIntegerParam(addr, PGPropertyAbsAvail,     pPropInfo->absValSupported);
            setIntegerParam(addr, PGPropertyOnOff,        pProperty->onOff);
            setIntegerParam(addr, PGPropertyAutoMode,     pProperty->autoManualMode);
            setIntegerParam(addr, PGPropertyAbsMode,      pProperty->absControl);
            setIntegerParam(addr, PGPropertyValue,        pProperty->valueA);
            setIntegerParam(addr, PGPropertyValueB,       pProperty->valueB);
            setIntegerParam(addr, PGPropertyValueMin,     pPropInfo->min);
            setIntegerParam(addr, PGPropertyValueMax,     pPropInfo->max);
        } else {
            /* If the propertyType is not available in the camera, we just set
             * all the parameters to 0 or -1 to indicate this is not available to the user. */
            setIntegerParam(addr, PGPropertyAvail,        0);
            setIntegerParam(addr, PGPropertyOnOffAvail,   0);
            setIntegerParam(addr, PGPropertyOnePushAvail, 0);
            setIntegerParam(addr, PGPropertyAutoAvail,    0);
            setIntegerParam(addr, PGPropertyManAvail,     0);
            setIntegerParam(addr, PGPropertyAbsAvail,     0);
            setIntegerParam(addr, PGPropertyAutoMode,     0);
            setIntegerParam(addr, PGPropertyAbsMode,      -1);
            setIntegerParam(addr, PGPropertyValue,        -1);
            setIntegerParam(addr, PGPropertyValueMin,     -1);
            setIntegerParam(addr, PGPropertyValueMax,     -1);
        }

        /* If the propertyType does not support 'absolute' control then we just
         * set all the absolute values to -1.0 to indicate it is not available to the user */
        if (pPropInfo->absValSupported) { 
            setDoubleParam(addr,  PGPropertyValueAbs,    pProperty->absValue);
            setDoubleParam(addr,  PGPropertyValueAbsMin, pPropInfo->absMin);
            setDoubleParam(addr,  PGPropertyValueAbsMax, pPropInfo->absMax);
        } else {
            setDoubleParam(addr,  PGPropertyValueAbs,    -1.0);
            setDoubleParam(addr,  PGPropertyValueAbsMax, -1.0);
            setDoubleParam(addr,  PGPropertyValueAbsMin, -1.0);
        }
    }

    /* Map a few of the AreaDetector parameters on to the camera properties */
    getDoubleParam(SHUTTER, PGPropertyValueAbs, &dtmp);
    // Camera units are ms
    setDoubleParam(ADAcquireTime, dtmp/1000.);

    getDoubleParam(FRAME_RATE, PGPropertyValueAbs, &dtmp);
    // Camera units are fps
    setDoubleParam(ADAcquirePeriod, 1./dtmp);

    getDoubleParam(GAIN, PGPropertyValueAbs, &dtmp);
    setDoubleParam(ADGain, dtmp);

    /* Do callbacks for each propertyType */
    for (addr=0; addr<NUM_PROPERTIES; addr++) callParamCallbacks(addr);

    return asynSuccess;
}

asynStatus pointGrey::getAllGigEProperties()
{
    GigEProperty *pProperty;
    int addr;
    Error error;
    int itemp;
    static const char *functionName="getAllGigeEProperties";
    
    if (pGigECamera_ == NULL) return asynSuccess;

    /* Iterate through all of the available properties and update their values and settings  */
    for (addr=0; addr<NUM_GIGE_PROPERTIES; addr++) {
        pProperty = allGigEProperties_[addr];
        error = pGigECamera_->GetGigEProperty(pProperty);
        if (checkError(error, functionName, "GetGigEProperty")) 
            return asynError;
        asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
            "%s::%s called GigECamera::GetGigEProperty, pGigECamera_=%p, pProperty=%p, pProperty->propType=%d, pProperty->value=%d\n",
            driverName, functionName, pGigECamera_, pProperty, pProperty->propType, pProperty->value);

        if (pProperty->isReadable) {
            setIntegerParam(addr, PGGigEPropertyValue,     pProperty->value);
            setIntegerParam(addr, PGGigEPropertyValueMin,  pProperty->min);
            setIntegerParam(addr, PGGigEPropertyValueMax,  pProperty->max);
        }
    }
    /* Map a few of the Point Grey parameters on to the GigE properties */
    getIntegerParam(PACKET_SIZE, PGGigEPropertyValue, &itemp);
    setIntegerParam(PGPacketSizeActual, itemp);
    getIntegerParam(PACKET_DELAY, PGGigEPropertyValue, &itemp);
    setIntegerParam(PGPacketDelayActual, itemp);

    /* Do callbacks for each propertyType */
    for (addr=0; addr<NUM_GIGE_PROPERTIES; addr++) callParamCallbacks(addr);
    return asynSuccess;
}


asynStatus pointGrey::startCapture()
{
    Error error;
    static const char *functionName = "startCapture";

    /* Start the camera transmission... */
    setIntegerParam(ADNumImagesCounter, 0);
    setShutter(1);
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::StartCapture, pCameraBase_=%p\n",
        driverName, functionName, pCameraBase_);
    error = pCameraBase_->StartCapture();
    if (checkError(error, functionName, "StartCapture")) 
        return asynError;
    epicsEventSignal(startEventId_);
    return asynSuccess;
}


asynStatus pointGrey::stopCapture()
{
    Error error;
    static const char *functionName = "stopCapture";

    setShutter(0);
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::StopCapture, pCameraBase_=%p\n",
        driverName, functionName, pCameraBase_);
    error = pCameraBase_->StopCapture();
    setIntegerParam(ADAcquire, 0);
    if (checkError(error, functionName, "StopCapture")) 
        return asynError;
    return asynSuccess;
}

asynStatus pointGrey::readStatus()
{
    Error error;
    static const char *functionName = "readStatus";

    error = pCameraBase_->GetStats(pCameraStats_);
    if (checkError(error, functionName, "GetStats")) 
        return asynError;
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
        "%s::%s calling CameraBase::GetStats, pCameraBase_=%p, pCameraStats_=%p, pCameraStats_->temperature=%d\n",
        driverName, functionName, pCameraBase_, pCameraStats_, pCameraStats_->temperature);
    setDoubleParam(ADTemperatureActual, pCameraStats_->temperature/10. - 273.15);
    setIntegerParam(PGCorruptFrames,    pCameraStats_->imageCorrupt);
    setIntegerParam(PGDriverDropped,    pCameraStats_->imageDriverDropped);
    if (pCameraStats_->imageXmitFailed == 0x80000000) pCameraStats_->imageXmitFailed = 0;
    setIntegerParam(PGTransmitFailed,   pCameraStats_->imageXmitFailed);
    setIntegerParam(PGDroppedFrames,    pCameraStats_->imageDropped);
    callParamCallbacks();
    return asynSuccess;
}



/** Print out a report; calls ADDriver::report to get base class report as well.
  * \param[in] fp File pointer to write output to
  * \param[in] details Level of detail desired.  If >1 prints information about 
               supported video formats and modes, etc.
 */
void pointGrey::report(FILE *fp, int details)
{
    unsigned int numCameras;
    Error error;
    Camera cam;
    int mode, rate;
    int source;
    unsigned int i, j;
    asynStatus status;
    bool supported;
    int property;
    int pixelFormatIndex;
    static const char *functionName = "report";
    
    error = pBusMgr_->GetNumOfCameras(&numCameras);
    if (checkError(error, functionName, "GetNumOfCameras")) return;

    fprintf(fp, "\nNumber of cameras detected: %u\n", numCameras);

    for (i=0; i<numCameras; i++) {
        PGRGuid guid;
        CameraInfo camInfo;
        error = pBusMgr_->GetCameraFromIndex(i, &guid);
        if (checkError(error, functionName, "GetCameraFromIndex")) return;

        // Connect to camera
        error = cam.Connect(&guid);
        if (checkError(error, functionName, "Connect")) return;

        // Get the camera information
        error = cam.GetCameraInfo(&camInfo);
        if (checkError(error, functionName, "GetCameraInfo")) return;

        fprintf(fp, "\n");
        fprintf(fp, "Serial number:       %u\n", camInfo.serialNumber);
        fprintf(fp, "Camera model:        %s\n", camInfo.modelName);
        fprintf(fp, "Camera vendor:       %s\n", camInfo.vendorName);
        fprintf(fp, "Sensor:              %s\n", camInfo.sensorInfo);
        fprintf(fp, "Resolution:          %s\n", camInfo.sensorResolution);
        fprintf(fp, "Firmware version:    %s\n", camInfo.firmwareVersion);
        fprintf(fp, "Firmware build time: %s\n", camInfo.firmwareBuildTime);

        // Disconnect from camera
        error = cam.Disconnect();
        if (checkError(error, functionName, "Connect")) return;
    }
    
    // It seems like disconnected from the cam object here causes the already found camera to
    // need to be reconnected
    status = connectCamera();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error calling connectCamera()\n",
            driverName, functionName);
    }
    
    if (pCamera_) {
        getIntegerParam(PGFormat7Mode, &mode);
        pFormat7Info_->mode = (Mode)mode;
        error = pCamera_->GetFormat7Info(pFormat7Info_, &supported);
        if (checkError(error, functionName, "GetFormat7Info")) 
            return;
        fprintf(fp, "\n");
        fprintf(fp, "Currently connected IIDC camera Format7 information\n");
        fprintf(fp, "  mode:            %d\n", pFormat7Info_->mode);
        fprintf(fp, "  maxWidth:        %d\n", pFormat7Info_->maxWidth);
        fprintf(fp, "  maxHeight:       %d\n", pFormat7Info_->maxHeight);
        fprintf(fp, "  offsetHStepSize: %d\n", pFormat7Info_->offsetHStepSize);
        fprintf(fp, "  offsetVStepSize: %d\n", pFormat7Info_->offsetVStepSize);
        fprintf(fp, "  imageHStepSize:  %d\n", pFormat7Info_->imageHStepSize);
        fprintf(fp, "  imageVStepSize:  %d\n", pFormat7Info_->imageVStepSize);
        fprintf(fp, "  pixelFormatBitField       0x%x\n", pFormat7Info_->pixelFormatBitField);
        fprintf(fp, "  vendorPixelFormatBitField 0x%x\n", pFormat7Info_->vendorPixelFormatBitField);
                    
    } else if (pGigECamera_) {
        error = pGigECamera_->GetGigEImageSettingsInfo(pGigEImageSettingsInfo_);
        if (checkError(error, functionName, "GetGigEImageSettingsInfo")) 
            return;
        fprintf(fp, "\n");
        fprintf(fp, "Currently connected GigE camera image information\n");
        fprintf(fp, "  maxWidth:        %d\n", pGigEImageSettingsInfo_->maxWidth);
        fprintf(fp, "  maxHeight:       %d\n", pGigEImageSettingsInfo_->maxHeight);
        fprintf(fp, "  offsetHStepSize: %d\n", pGigEImageSettingsInfo_->offsetHStepSize);
        fprintf(fp, "  offsetVStepSize: %d\n", pGigEImageSettingsInfo_->offsetVStepSize);
        fprintf(fp, "  imageHStepSize:  %d\n", pGigEImageSettingsInfo_->imageHStepSize);
        fprintf(fp, "  imageVStepSize:  %d\n", pGigEImageSettingsInfo_->imageVStepSize);
        fprintf(fp, "  pixelFormatBitField       0x%x\n", pGigEImageSettingsInfo_->pixelFormatBitField);
        fprintf(fp, "  vendorPixelFormatBitField 0x%x\n", pGigEImageSettingsInfo_->vendorPixelFormatBitField);
    }

    if (details < 1) goto done;
    
    if (pCamera_) {
        VideoMode videoMode;
        FrameRate frameRate;
        unsigned int packetSize;
        float percentage;
        Format7ImageSettings f7Settings;
        fprintf(fp, "\nSupported IIDC video modes and rates:\n");
        for (mode=0; mode<NUM_VIDEOMODES; mode++) {
            videoMode = (VideoMode)mode;
            if (videoMode == VIDEOMODE_FORMAT7) continue;
            for (rate=0; rate<NUM_FRAMERATES; rate++) {
                frameRate = (FrameRate)rate;
                if (frameRate == FRAMERATE_FORMAT7) continue;
                error = pCamera_->GetVideoModeAndFrameRateInfo(videoMode, frameRate, &supported);
                if (checkError(error, functionName, "GetVideoModeAndFrameRateInfo")) 
                    return;
                if (supported) {
                    fprintf(fp, "    Video mode %d (%s) and frame rate %d (%s)\n", 
                    mode, videoModeStrings[mode], rate, frameRateStrings[rate]);
                }
            }
        }
        error = pCamera_->GetVideoModeAndFrameRate(&videoMode, &frameRate);
        if (checkError(error, functionName, "GetVideoModeAndFrameRate")) 
            return;
        fprintf(fp, "\nCurrent image settings\n");
        fprintf(fp, "  Mode: %d (%s)\n", videoMode, videoModeStrings[videoMode]);
        fprintf(fp, "  Rate: %d (%s)\n", frameRate, frameRateStrings[frameRate]);
        if (videoMode == VIDEOMODE_FORMAT7) {
            error = pCamera_->GetFormat7Configuration(&f7Settings, &packetSize, &percentage);
            if (checkError(error, functionName, "GetFormat7Configuration")) 
                return;
            pixelFormatIndex = getPixelFormatIndex(f7Settings.pixelFormat);     
            fprintf(fp, "  Format7 video format currently selected\n");
            fprintf(fp, "    Packet size: %d\n",    packetSize);
            fprintf(fp, "    Bandwidth %%: %f\n",   percentage);
            fprintf(fp, "           Mode: %d\n",    f7Settings.mode);
            fprintf(fp, "         Offset: %d %d\n", f7Settings.offsetX, f7Settings.offsetY);
            fprintf(fp, "           Size: %d %d\n", f7Settings.width, f7Settings.height);
            fprintf(fp, "    PixelFormat: index=%d, value=0x%x [%s]\n",
                             pixelFormatIndex, f7Settings.pixelFormat, pixelFormatStrings[pixelFormatIndex]);
        }

    } else if (pGigECamera_) {
        int packetSize;
        int packetDelay;
        int heartBeat;
        int heartBeatTimeout;
        unsigned int binX, binY;
        GigEImageSettings gigESettings;
        Mode gigEMode;
        error = pGigECamera_->GetGigEImagingMode(&gigEMode);
        if (checkError(error, functionName, "GetGigEImagingMode")) 
            return;
        fprintf(fp, "\nSupported GigE modes:\n");
        for (mode=0; mode<NUM_MODES; mode++) {
            error = pGigECamera_->QueryGigEImagingMode((Mode)mode, &supported);
            if (checkError(error, functionName, "QueryGigEImagingMode")) 
                return;
            if (supported) {
                fprintf(fp, "    GigE mode %d\n", mode);
                asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                    "%s::%s calling GigECamera::SetGigEImagingMode, pGigECamera_=%p, mode=%d\n",
                    driverName, functionName, pGigECamera_, mode);
                error = pGigECamera_->SetGigEImagingMode((Mode)mode);
                if (checkError(error, functionName, "SetGigEImagingMode"))
                    return;
                asynPrint(pasynUserSelf, ASYN_TRACE_WARNING,
                    "%s::%s calling GigECamera::GetGigEImageSettingsInfo, pGigECamera_=%p, pGigEImageSettingsInfo_=%p\n",
                    driverName, functionName, pGigECamera_, pGigEImageSettingsInfo_);
                error = pGigECamera_->GetGigEImageSettingsInfo(pGigEImageSettingsInfo_);
                if (checkError(error, functionName, "GetGigEImageSettingsInfo")) 
                    return;
                fprintf(fp, "     maxWidth:        %d\n", pGigEImageSettingsInfo_->maxWidth);
                fprintf(fp, "     maxHeight:       %d\n", pGigEImageSettingsInfo_->maxHeight);
                fprintf(fp, "     offsetHStepSize: %d\n", pGigEImageSettingsInfo_->offsetHStepSize);
                fprintf(fp, "     offsetVStepSize: %d\n", pGigEImageSettingsInfo_->offsetVStepSize);
                fprintf(fp, "     imageHStepSize:  %d\n", pGigEImageSettingsInfo_->imageHStepSize);
                fprintf(fp, "     imageVStepSize:  %d\n", pGigEImageSettingsInfo_->imageVStepSize);
                fprintf(fp, "     pixelFormatBitField       0x%x\n", pGigEImageSettingsInfo_->pixelFormatBitField);
                fprintf(fp, "     vendorPixelFormatBitField 0x%x\n", pGigEImageSettingsInfo_->vendorPixelFormatBitField);
            }    
        }
        error = pGigECamera_->SetGigEImagingMode(gigEMode);
        if (checkError(error, functionName, "SetGigEImagingMode")) 
            return;
        error = pGigECamera_->GetGigEImageSettings(&gigESettings);
        if (checkError(error, functionName, "GetGigEImageSettings")) 
            return;
        error = pGigECamera_->GetGigEImageBinningSettings(&binX, &binY);
        if (checkError(error, functionName, "GetGigEImageBinningSettings")) 
            return;
        pixelFormatIndex = getPixelFormatIndex(gigESettings.pixelFormat);
        getAllGigEProperties();
        getIntegerParam(PACKET_SIZE,       PGGigEPropertyValue, &packetSize);
        getIntegerParam(PACKET_DELAY,      PGGigEPropertyValue, &packetDelay);
        getIntegerParam(HEARTBEAT,         PGGigEPropertyValue, &heartBeat);
        getIntegerParam(HEARTBEAT_TIMEOUT, PGGigEPropertyValue, &heartBeatTimeout);
        fprintf(fp, "\n");
        fprintf(fp, "Current GigE image settings\n");
        fprintf(fp, "               Mode: %d\n",    gigEMode);
        fprintf(fp, "        Packet size: %d\n",    packetSize);
        fprintf(fp, "       Packet delay: %d\n",    packetDelay);
        fprintf(fp, "          Heartbeat: %d\n",    heartBeat);
        fprintf(fp, "  Heartbeat timeout: %d\n",    heartBeatTimeout);
        fprintf(fp, "             Offset: %d %d\n", gigESettings.offsetX, gigESettings.offsetY);
        fprintf(fp, "               Size: %d %d\n", gigESettings.width, gigESettings.height);
        fprintf(fp, "            Binning: %u %u\n", binX, binY);
        fprintf(fp, "        PixelFormat: index=%d, value=0x%x [%s]\n",
                             pixelFormatIndex, gigESettings.pixelFormat, pixelFormatStrings[pixelFormatIndex]);
    }

    /* Iterate through all of the available properties and report on them  */
    fprintf(fp, "\nSupported properties\n");
    Property *pProperty;
    PropertyInfo *pPropInfo;
    for (property=0; property<NUM_PROPERTIES; property++) {
        pProperty = allProperties_[property];
        pPropInfo = allPropInfos_[property];
        error = pCameraBase_->GetProperty(pProperty);
        if (checkError(error, functionName, "GetProperty")) 
            return;
        error = pCameraBase_->GetPropertyInfo(pPropInfo);
        if (checkError(error, functionName, "GetPropertyInfo")) 
            return;
        if (pProperty->present) {
            fprintf(fp, "Property %s \n",                       propertyTypeStrings[property]);
            fprintf(fp, "  min           = %d\n",               pPropInfo->min);
            fprintf(fp, "  max           = %d\n",               pPropInfo->max);
            fprintf(fp, "  value         = %d\n",               pProperty->valueA);
            fprintf(fp, "  hasAutoMode   = %d     status=%d\n", pPropInfo->autoSupported,    pProperty->autoManualMode);
            fprintf(fp, "  hasManualMode = %d     status=%d\n", pPropInfo->manualSupported,  !pProperty->autoManualMode);
            fprintf(fp, "  hasOnOff      = %d     status=%d\n", pPropInfo->onOffSupported,   pProperty->onOff);
            fprintf(fp, "  hasOnePush    = %d     status=%d\n", pPropInfo->onePushSupported, pProperty->onePush);
            fprintf(fp, "  hasReadout    = %d\n",               pPropInfo->readOutSupported);
            fprintf(fp, "  hasAbsControl = %d     status=%d\n", pPropInfo->absValSupported,  pProperty->absControl);
            if (pPropInfo->absValSupported) { 
                fprintf(fp, "    units        = %s    abbreviated=%s\n", pPropInfo->pUnits, pPropInfo->pUnitAbbr);
                fprintf(fp, "    min          = %f\n", pPropInfo->absMin);
                fprintf(fp, "    max          = %f\n", pPropInfo->absMax);
                fprintf(fp, "    value        = %f\n", pProperty->absValue);
            }
        }
        else {
            fprintf(fp, "Property %s is not supported\n", propertyTypeStrings[property]);
        }

    }

    error = pCameraBase_->GetTriggerMode(pTriggerMode_);
    if (checkError(error, functionName, "GetTriggerMode")) 
        return;
    fprintf(fp, "\nTrigger mode\n");
    fprintf(fp, "       Mode: %d\n", pTriggerMode_->mode);
    fprintf(fp, "      onOff: %d\n", pTriggerMode_->onOff);
    fprintf(fp, "   polarity: %d\n", pTriggerMode_->polarity);
    fprintf(fp, "     source: %d\n", pTriggerMode_->source);
    fprintf(fp, "  parameter: %d\n", pTriggerMode_->parameter);

    error = pCameraBase_->GetTriggerModeInfo(pTriggerModeInfo_);
    if (checkError(error, functionName, "GetTriggerModeInfo")) 
        return;
    fprintf(fp, "\nTrigger mode information\n");
    fprintf(fp, "                   present: %d\n",   pTriggerModeInfo_->present);
    fprintf(fp, "          readOutSupported: %d\n",   pTriggerModeInfo_->readOutSupported);
    fprintf(fp, "            onOffSupported: %d\n",   pTriggerModeInfo_->onOffSupported);
    fprintf(fp, "         polaritySupported: %d\n",   pTriggerModeInfo_->polaritySupported);
    fprintf(fp, "             valueReadable: %d\n",   pTriggerModeInfo_->valueReadable);
    fprintf(fp, "                sourceMask: 0x%x\n", pTriggerModeInfo_->sourceMask);
    fprintf(fp, "  softwareTriggerSupported: %d\n",   pTriggerModeInfo_->softwareTriggerSupported);
    fprintf(fp, "                  modeMask: 0x%x\n", pTriggerModeInfo_->modeMask);

    fprintf(fp, "\nStrobe information\n");
    source = pStrobeControl_->source;
    for (j=0; j<NUM_GPIO_PINS; j++) {
        pStrobeInfo_->source = j;
        error = pCameraBase_->GetStrobeInfo(pStrobeInfo_);
        if (checkError(error, functionName, "GetStrobeInfo")) 
            return;
        if (pStrobeInfo_->present) {
            fprintf(fp, "     Strobe source %d: present\n", j);
            fprintf(fp, "     readOutSupported: %d\n", pStrobeInfo_->readOutSupported);
            fprintf(fp, "       onOffSupported: %d\n", pStrobeInfo_->onOffSupported);
            fprintf(fp, "    polaritySupported: %d\n", pStrobeInfo_->polaritySupported);
            fprintf(fp, "             minValue: %f\n", pStrobeInfo_->minValue);
            fprintf(fp, "             maxValue: %f\n", pStrobeInfo_->maxValue);
            pStrobeControl_->source = j;
            error = pCameraBase_->GetStrobe(pStrobeControl_);
            if (checkError(error, functionName, "GetStrobe")) 
                return;
            fprintf(fp, "                onOff: %d\n", pStrobeControl_->onOff);
            fprintf(fp, "             polarity: %u\n", pStrobeControl_->polarity);
            fprintf(fp, "                delay: %f\n", pStrobeControl_->delay);
            fprintf(fp, "             duration: %f\n", pStrobeControl_->duration);
        }
    }
    pStrobeControl_->source = source;


    error = pCameraBase_->GetStats(pCameraStats_);
    if (checkError(error, functionName, "GetStats")) 
        return;
    if (pCameraStats_->imageXmitFailed == 0x80000000) pCameraStats_->imageXmitFailed = 0;
    fprintf(fp, "\nCamera statistics\n");
    fprintf(fp, "              Images dropped: %u\n", pCameraStats_->imageDropped);
    fprintf(fp, "              Images corrupt: %u\n", pCameraStats_->imageCorrupt);
    fprintf(fp, "             Transmit failed: %u\n", pCameraStats_->imageXmitFailed);
    fprintf(fp, "              Driver dropped: %u\n", pCameraStats_->imageDriverDropped);
    fprintf(fp, "        Register read failed: %u\n", pCameraStats_->regReadFailed);
    fprintf(fp, "       Register write failed: %u\n", pCameraStats_->regWriteFailed);
    fprintf(fp, "                 Port errors: %u\n", pCameraStats_->portErrors);
    fprintf(fp, "             Camera power up: %d\n", pCameraStats_->cameraPowerUp);
    fprintf(fp, "                  # voltages: %d\n", pCameraStats_->numVoltages);
    for (j=0; j<pCameraStats_->numVoltages; j++) {
        fprintf(fp, "                   voltage %d: %f\n", j, pCameraStats_->cameraVoltages[j]);
    }
    fprintf(fp, "                  # currents: %d\n", pCameraStats_->numCurrents);
    for (j=0; j<pCameraStats_->numCurrents; j++) {
        fprintf(fp, "                   current %d: %f\n", j, pCameraStats_->cameraCurrents[j]);
    }
    fprintf(fp, "             Temperature (C): %f\n", pCameraStats_->temperature/10. - 273.15);
    fprintf(fp, "   Time since initialization: %u\n", pCameraStats_->timeSinceInitialization);
    fprintf(fp, "        Time since bus reset: %u\n", pCameraStats_->timeSinceBusReset);
    fprintf(fp, "  # resend packets requested: %u\n", pCameraStats_->numResendPacketsRequested);
    fprintf(fp, "   # resend packets received: %u\n", pCameraStats_->numResendPacketsReceived);
    fprintf(fp, "                  Time stamp: %f\n", pCameraStats_->timeStamp.seconds + 
                                                      pCameraStats_->timeStamp.microSeconds/1e6);
    done:          
    ADDriver::report(fp, details);
    return;
}

static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"cameraId", iocshArgInt};
static const iocshArg configArg2 = {"traceMask", iocshArgInt};
static const iocshArg configArg3 = {"memoryChannel", iocshArgInt};
static const iocshArg configArg4 = {"maxBuffers", iocshArgInt};
static const iocshArg configArg5 = {"maxMemory", iocshArgInt};
static const iocshArg configArg6 = {"priority", iocshArgInt};
static const iocshArg configArg7 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3,
                                              &configArg4,
                                              &configArg5,
                                              &configArg6,
                                              &configArg7};
static const iocshFuncDef configpointGrey = {"pointGreyConfig", 8, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    pointGreyConfig(args[0].sval, args[1].ival, args[2].ival, 
                    args[3].ival, args[4].ival, args[5].ival,
                    args[6].ival, args[7].ival);
}


static void pointGreyRegister(void)
{
    iocshRegister(&configpointGrey, configCallFunc);
}

extern "C" {
epicsExportRegistrar(pointGreyRegister);
}

