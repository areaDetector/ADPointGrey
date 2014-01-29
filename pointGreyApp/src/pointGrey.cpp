/*
 * andor3.cpp
 *
 * This is a driver for Andor SDK3 camera (Neo/Zyla)
 *
 * Author: Phillip Sorensen
 *         Cornell University
 *
 * Co-author: Mark Rivers
 *            Univerity of Chicago
 *
 * Created: October 8, 2012
 *
 * NOTES:
 *
 * 1. Feature "CameraAcquiring" is a boolen with FALSE = 0 and TRUE = 1.  This
 *    is mapped to parameter ADStatus and relies on the fact the current
 *    definition of the mbbi/mbbo are 0 is "Idle", and 1 is "Acquiring".  No
 *    other states of ADStatus are currently used.  If default mapping is
 *    changed the mbbi/mbbo record will need to be overriden like the 
 *    trigger and image mode records. 
 *
 * 2. Memory for buffers passed to the Camera API are allocated with the
 *    posix_memalign funtion in linux.  For other platforms this may need
 *    to be changed.
 */

#include <stdio.h>
#include <string.h>
#include <wchar.h>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExit.h>

#include <atcore.h>

#include "ADDriver.h"

#include <epicsExport.h>

#define MAX_FEATURE_NAME_LEN 64

static const char *driverName = "andor3";

static int AtInitialized = 0;

/* feature types */
typedef enum {
    ATint,
    ATfloat,
    ATbool,
    ATenum,
    ATstring,
    ATcommand
} Andor3FeatureType;

typedef struct {
  class andor3*     camera;
  Andor3FeatureType type;
  int               paramIndex;
  AT_BOOL           isImplemented;
  AT_WC             featureNameWC[MAX_FEATURE_NAME_LEN];
  char              featureNameMBS[MAX_FEATURE_NAME_LEN];
  bool              exists;
} featureInfo;


class andor3 : public ADDriver {
public:
    andor3(const char *portName, int cameraId, int maxBuffers, 
           size_t maxMemory, int priority, int stackSize, int maxFrames);

    /* override ADDriver methods */ 
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                                size_t nElements, size_t *nIn);
    virtual void report(FILE *fp, int details);

    /* "private methods" that need to be called from C */
    void shutdown();
    void imageTask();
    void tempTask();
    int getFeature(int paramIndex);

protected:
    int Andor3FrameRate;
    #define FIRST_ANDOR3_PARAM Andor3FrameRate
    int Andor3PixelEncoding;
    int Andor3FullAOIControl;
    int Andor3Binning;
    int Andor3ShutterMode;
    int Andor3SoftwareTrigger;
    int Andor3SensorCooling;
    int Andor3TempControl;
    int Andor3TempStatus;
    int Andor3SerialNumber;
    int Andor3FirmwareVersion;
    int Andor3SoftwareVersion;
    int Andor3ControllerID;
    int Andor3Overlap;
    int Andor3ReadoutRate;
    int Andor3ReadoutTime;
    int Andor3TransferRate;
    int Andor3PreAmpGain;
    int Andor3NoiseFilter;
    int Andor3FanSpeed;
    #define LAST_ANDOR3_PARAM Andor3FanSpeed
private:
    int registerFeature(const AT_WC *feature, Andor3FeatureType type,
                        int paramIndex);
    int setFeature(int paramIndex);
    int getFeature(int paramIndex, AT_H handle);
    int getEnumString(int paramIndex, char *str, int len);
    int reportFeature(int paramIndex, FILE *fp, int details);
    int getAOI();
    int setAOI();
    int allocateBuffers();
    int freeBuffers();
    int connectCamera();
    int disconnectCamera();
    size_t WCSToMBS(char *mbs, const AT_WC *wcs, size_t mbsLen);
    
    featureInfo *featureInfo_;
    AT_H    handle_;
    int     id_;
    int     maxFrames_;
    AT_U8 **drvBuffers_;    
    AT_64   imageSize_;
    int     exiting_;
    epicsEventId startEvent_;
};
#define NUM_ANDOR3_PARAMS ((int)(&LAST_ANDOR3_PARAM - &FIRST_ANDOR3_PARAM + 1))

/* Andor3 driver specific parameters */
#define Andor3FrameRateString        "A3_FRAME_RATE"        /* asynFloat64  rw */
#define Andor3PixelEncodingString    "A3_PIXEL_ENCODING"    /* asynInt32    rw */
#define Andor3FullAOIControlString   "A3_FULL_AOI_CONTROL"  /* asynInt32    ro */
#define Andor3BinningString          "A3_BINNING"           /* asynInt32    rw */
#define Andor3ShutterModeString      "A3_SHUTTER_MODE"      /* asynInt32    rw */
#define Andor3SoftwareTriggerString  "A3_SOFTWARE_TRIGGER"  /* asynInt32    wo */
#define Andor3SensorCoolingString    "A3_SENSOR_COOLING"    /* asynInt32    rw */
#define Andor3TempControlString      "A3_TEMP_CONTROL"      /* asynInt32    rw */
#define Andor3TempStatusString       "A3_TEMP_STATUS"       /* asynInt32    ro */
#define Andor3SerialNumberString     "A3_SERIAL_NUMBER"     /* asynOctet    ro */
#define Andor3FirmwareVersionString  "A3_FIRMWARE_VERSION"  /* asynOctet    ro */
#define Andor3SoftwareVersionString  "A3_SOFTWARE_VERSION"  /* asynOctet    ro */
#define Andor3ControllerIDString     "A3_CONTROLLER_ID"     /* asynOctet    ro */
#define Andor3OverlapString          "A3_OVERLAP"           /* asynInt32    rw */
#define Andor3ReadoutRateString      "A3_READOUT_RATE"      /* asynInt32    rw */
#define Andor3ReadoutTimeString      "A3_READOUT_TIME"      /* asynFloat64  rw */
#define Andor3TransferRateString     "A3_TRANSFER_RATE"     /* asynFloat64  rw */
#define Andor3PreAmpGainString       "A3_PREAMP_GAIN"       /* asynInt32    rw */
#define Andor3NoiseFilterString      "A3_NOISE_FILTER"      /* asynInt32    rw */
#define Andor3FanSpeedString         "A3_FAN_SPEED"         /* asynInt32    rw */

static void c_shutdown(void *arg)
{
  andor3 *p = (andor3 *)arg;
  p->shutdown();
}

static void c_imagetask(void *arg)
{
  andor3 *p = (andor3 *)arg;
  p->imageTask();
}

static void c_temptask(void *arg)
{
  andor3 *p = (andor3 *)arg;
  p->tempTask();
}

static int AT_EXP_CONV c_getfeature(AT_H handle, const AT_WC *feature, void *context)
{
  featureInfo *info = (featureInfo *)context;
  return info->camera->getFeature(info->paramIndex);
}


void andor3::shutdown(void)
{
    exiting_ = 1;
    if(handle_) {
        disconnectCamera();
    }
    AtInitialized--;
    if(AtInitialized == 0) {
        AT_FinaliseLibrary();
    }
}

void andor3::imageTask()
{
    epicsTimeStamp imageStamp;
    int status;
    AT_U8  *image;
    int size;
    char modeString[MAX_FEATURE_NAME_LEN];
    int acquire;
    int total;
    int number;
    int count;
    int callback;

    static const char *functionName = "imageTask";

    lock();

    while(!exiting_) {

        getIntegerParam(ADAcquire, &acquire);
        if(!acquire) {
            AT_Flush(handle_);

            unlock();
            epicsEventWait(startEvent_);
            lock();
            
            AT_Flush(handle_);
            for(int x=0; x<maxFrames_; x++) {
                if(drvBuffers_[x]) {
                    status = AT_QueueBuffer(handle_, drvBuffers_[x],
                                            (int)imageSize_);
                    if(status) {
                        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                            "%s:%s: AT_QueueBuffer error: %d\n", 
                            driverName, functionName, status);
                    }
                }
            }
            AT_Command(handle_, L"AcquisitionStart");
        }
        
        unlock();
        status = AT_WaitBuffer(handle_, &image, &size, AT_INFINITE);
        lock();
        if(status != AT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: AT_WaitBuffer, error=%d\n", 
                driverName, functionName, status);
            continue;
        }
        epicsTimeGetCurrent(&imageStamp);

        getIntegerParam(ADNumImagesCounter, &number);
        number++;
        setIntegerParam(ADNumImagesCounter, number);
        getIntegerParam(NDArrayCounter, &count);
        count++;
        setIntegerParam(NDArrayCounter, count);
        callParamCallbacks();

        getIntegerParam(NDArrayCallbacks, &callback);
        if(callback) {
            NDArray *pImage;
            size_t dims[2];
            int itemp;
            char encodingString[MAX_FEATURE_NAME_LEN];
            AT_64 stride;
            int pixelSize;

            getIntegerParam(NDArraySizeX, &itemp); dims[0] = itemp;
            getIntegerParam(NDArraySizeY, &itemp); dims[1] = itemp;

            getEnumString(Andor3PixelEncoding, encodingString, sizeof(encodingString));
            if (strcmp(encodingString, "Mono32")==0) {
                pImage = pNDArrayPool->alloc(2, dims, NDUInt32, 0, NULL);
                pixelSize = 4;
                setIntegerParam(NDDataType, NDUInt32);
            } else {
                pImage = pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);
                pixelSize = 2;
                setIntegerParam(NDDataType, NDUInt16);
            }
            if(pImage) {
                pImage->uniqueId = count;
                pImage->timeStamp = 631152000 + imageStamp.secPastEpoch +
                    (imageStamp.nsec / 1.0e9);
                updateTimeStamp(&pImage->epicsTS);

                AT_GetInt(handle_, L"AOIStride", &stride);
                if ((strcmp(encodingString, "Mono12")==0) || 
                    (strcmp(encodingString, "Mono16")==0) ||
                    (strcmp(encodingString, "Mono32")==0)) {
                    AT_U8 *p;

                    p = (AT_U8 *)pImage->pData;
                    for(int x = 0; x < size; x += (int)stride) {
                        memcpy(p, image+x, dims[0]*pixelSize);
                        p += dims[0]*pixelSize;
                    }
                } else if (strcmp(encodingString, "Mono12Packed")==0) {
                    AT_U8 *enc = image;
                    unsigned short *dec = (unsigned short*)pImage->pData;

                    for(int x = 0; x < size; x += (int)stride) {
                        enc = image + x;
                        for (size_t j = 0; j < dims[0]/pixelSize; j++) {
                            *dec     = (*enc << 4) + (*(enc+1) & 0xf);
                            *(dec+1) = (*(enc+2)<<4) + ((*(enc+1) >> 4) & 0xf);
                            enc += 3;
                            dec += pixelSize;
                        }
                    }
                }

                getAttributes(pImage->pAttributeList);

                unlock();
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                lock();

                pImage->release();
            }
        }
        getEnumString(ADImageMode, modeString, sizeof(modeString));
        getIntegerParam(ADNumImages, &total);
        if((strcmp(modeString, "Fixed")==0) && number == total) {
            setShutter(0);
            AT_Command(handle_, L"AcquisitionStop");
            setIntegerParam(ADAcquire, 0);
        }
        callParamCallbacks();

        status = AT_QueueBuffer(handle_, image, size);
        if(status) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: AT_QueueBuffer 2: error=%d\n", 
                driverName, functionName, status);
        }
    }
}

void andor3::tempTask(void)
{
    int status;
    static const char *functionName = "tempTask";

    while(!exiting_) {
        status  = getFeature(ADTemperatureActual);
        status |= getFeature(Andor3SensorCooling);
        status |= getFeature(Andor3TempStatus);

        if(status && (status != AT_ERR_NOTIMPLEMENTED)) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: temperature read error = %d\n", 
                driverName, functionName, status);
        }
        epicsThreadSleep(1.0);
    }
}

int andor3::getFeature(int paramIndex)
{
    return getFeature(paramIndex, handle_);
}

int andor3::getFeature(int paramIndex, AT_H handle)
{
    static const char* functionName = "getFeature";

    int     status = 0;
    featureInfo *info = &featureInfo_[paramIndex];
    AT_WC   *featureWC = info->featureNameWC;
    AT_64   i_value;
    double  d_value;
    AT_BOOL b_value;
    int     index;
    int     length;
    AT_WC  *wide;
    char   *str;
    
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
        "%s:%s: entry, feature=%s\n",
        driverName, functionName, info->featureNameMBS);

    if (!info->isImplemented) return AT_ERR_NOTIMPLEMENTED;
    
    /* get feature value to paramIndex */
    switch(info->type) {
    case ATint:
        status = AT_GetInt(handle, featureWC, &i_value);
        if(status == AT_SUCCESS) {
            status = setIntegerParam(paramIndex, (int)i_value);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATfloat:
        status = AT_GetFloat(handle, featureWC, &d_value);
        if(status == AT_SUCCESS) {
            status = setDoubleParam(paramIndex, d_value);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATbool:
        status = AT_GetBool(handle, featureWC, &b_value);
        if(status == AT_SUCCESS) {
            status = setIntegerParam(paramIndex, (int)b_value);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATenum:
        status = AT_GetEnumIndex(handle, featureWC, &index);
        if(status == AT_SUCCESS) {
            status = setIntegerParam(paramIndex, index);
            if(status) {
                status = -1;
            }
        }
        break;
    case ATstring:
        status = AT_GetStringMaxLength(handle, featureWC, &length);
        if(status == AT_SUCCESS) {
            length++;

            wide = new AT_WC[length];
            str  = new char[length];

            status = AT_GetString(handle, featureWC, wide, length);
            if(status == AT_SUCCESS) {
                WCSToMBS(str, wide, length);
                status = setStringParam(paramIndex, str);
            }
            delete wide;
            delete str;
        }
        break;
    case ATcommand:
        // Nothing to do
        break;
    }

    /* translate features to ADBase parameters */
    if(status == AT_SUCCESS) {
        if(!strcmp(info->featureNameMBS, "FrameRate")) {
            paramIndex = ADAcquirePeriod;
            status = setDoubleParam(paramIndex, d_value ? 1.0/d_value : 0);
            if(status) {
                status = -1;
            }
        }
        if(!strncmp(info->featureNameMBS, "AOI", 3)) {
            getAOI();
        }       
    }

    /* reallocate image buffers if size changed */
        if(!strcmp(info->featureNameMBS, "ImageSizeBytes")) {
        asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
            "%s:%s: allocating buffers\n",
            driverName, functionName);
        allocateBuffers();
    }

    /* error messages */
    if(status == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to set parameter index %d\n",
            driverName, functionName, paramIndex);
        status = AT_ERR_NOTWRITABLE;
    } else if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to get feature %s (%d)\n",
            driverName, functionName, info->featureNameMBS, status);   
    }

    callParamCallbacks();
    return status;
}

int andor3::getEnumString(int paramIndex, char *str, int len)
{
    AT_WC *featureWC = featureInfo_[paramIndex].featureNameWC;
    int status = 0;
    int enumIndex;
    AT_WC enumStringWC[MAX_FEATURE_NAME_LEN];

    status |= AT_GetEnumIndex(handle_, featureWC, &enumIndex);
    status |= AT_GetEnumStringByIndex(handle_, featureWC, enumIndex, 
                                      enumStringWC, MAX_FEATURE_NAME_LEN-1);
    WCSToMBS(str, enumStringWC, len);
    return status;
} 

int andor3::setFeature(int paramIndex)
{
    static const char* functionName = "setFeature";

    featureInfo *info = &featureInfo_[paramIndex];
    AT_WC  *featureWC = info->featureNameWC;
    int    status=0;
    int    i_value;
    AT_64  i_min;
    AT_64  i_max;
    double d_value;
    double d_min;
    double d_max;

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
        "%s:%s: entry, feature=%s\n",
        driverName, functionName, info->featureNameMBS);

    /* get feature value to paramIndex */
    switch(info->type) {
    case ATint:
        status = getIntegerParam(paramIndex, &i_value);
        if(status) {
            status = -1;
        } else {
            status  = AT_GetIntMax(handle_, featureWC, &i_max);
            status |= AT_GetIntMin(handle_, featureWC, &i_min);
            if(status == AT_SUCCESS) {
                if(i_value < i_min) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s setting %s to minimum value %lld (was %d)\n",
                        driverName, functionName, info->featureNameMBS, 
                              i_min, i_value);
                    i_value = (int)i_min;
                } else if(i_value > i_max) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s setting %s to max value %lld (was %d)\n",
                        driverName, functionName, info->featureNameMBS, 
                              i_max, i_value);
                    i_value = (int)i_max;
                }
                status = AT_SetInt(handle_, featureWC, (AT_64)i_value);
            }
        }
        break;
    case ATfloat:
        status = getDoubleParam(paramIndex, &d_value);
        if(status) {
            status = -1;
        } else {
            status  = AT_GetFloatMax(handle_, featureWC, &d_max);
            status |= AT_GetFloatMin(handle_, featureWC, &d_min);
            if(status == AT_SUCCESS) {
                if(d_value < d_min) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s setting %s to minimum value %f (was %f)\n",
                          driverName, functionName, info->featureNameMBS, 
                          d_min, d_value);
                    d_value = d_min;
                } else if(d_value > d_max) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s setting %s to max value %f (was %f)\n",
                        driverName, functionName, info->featureNameMBS, 
                        d_max, d_value);
                    d_value = d_max;
                }
                status = AT_SetFloat(handle_, featureWC, d_value);
            }
        }
        break;
    case ATbool:
        status = getIntegerParam(paramIndex, &i_value);
        if(status) {
            status = -1;
        } else {
            status = AT_SetBool(handle_, featureWC, 
                                i_value ? AT_TRUE : AT_FALSE);
        }
        break;
    case ATenum:
        status = getIntegerParam(paramIndex, &i_value);
        if(status) {
            status = -1;
        } else {
            status = AT_SetEnumIndex(handle_, featureWC, i_value);
        }
        break;
    case ATstring:
        status = 2;
        break;
    case ATcommand:
        status = AT_Command(handle_, featureWC);
        break;
    }

    /* error messages */
    if(status == -1) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to get parameter index %d\n",
            driverName, functionName, paramIndex);
        status = AT_ERR_NOTWRITABLE;
    } else if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to set feature %s (%d)\n",
            driverName, functionName, info->featureNameMBS, status);   
    }

    /* read back current value if set failed */
    if(status != AT_SUCCESS) {
        getFeature(paramIndex);
    }
    return status;
}

void andor3::report(FILE *fp, int details)
{
    char str[MAX_FEATURE_NAME_LEN];
    
    getStringParam(Andor3SoftwareVersion, sizeof(str), str);
    fprintf(fp, "Andor driver, SDK version=%s\n", str);
    if (details < 1) return;

    reportFeature(ADModel, fp, details);
    reportFeature(Andor3SerialNumber, fp, details);
    reportFeature(Andor3FirmwareVersion, fp, details);
    reportFeature(Andor3ControllerID, fp, details);
    reportFeature(ADMaxSizeX, fp, details);
    reportFeature(ADMaxSizeY, fp, details);
    reportFeature(Andor3FullAOIControl, fp, details);
    reportFeature(ADSizeX, fp, details);
    reportFeature(ADSizeY, fp, details);
    reportFeature(ADMinX, fp, details);
    reportFeature(ADMinY, fp, details);
    reportFeature(Andor3Binning, fp, details);
    reportFeature(Andor3ShutterMode, fp, details);
    reportFeature(Andor3PixelEncoding, fp, details);
    reportFeature(Andor3ReadoutRate, fp, details);
    reportFeature(Andor3FrameRate, fp, details);
    reportFeature(Andor3Overlap, fp, details);
    reportFeature(Andor3PreAmpGain, fp, details);
    reportFeature(Andor3ReadoutTime, fp, details);
    reportFeature(Andor3TransferRate, fp, details);
    reportFeature(Andor3NoiseFilter, fp, details);
    reportFeature(ADImageMode, fp, details);
    reportFeature(ADAcquireTime, fp, details);
    reportFeature(ADNumExposures, fp, details);
    reportFeature(ADNumImages, fp, details);
    reportFeature(ADStatus, fp, details);
    reportFeature(ADTriggerMode, fp, details);
    reportFeature(Andor3SoftwareTrigger, fp, details);
    reportFeature(Andor3SensorCooling, fp, details);
    reportFeature(ADTemperature, fp, details);
    reportFeature(ADTemperatureActual, fp, details);
    reportFeature(Andor3TempControl, fp, details);
    reportFeature(Andor3TempStatus, fp, details);
    reportFeature(Andor3FanSpeed, fp, details);
    
    ADDriver::report(fp, details);
}    

int andor3::reportFeature(int paramIndex, FILE *fp, int details)
{
    static const char* functionName = "reportFeature";

    featureInfo *info = &featureInfo_[paramIndex];
    AT_WC   *featureWC = info->featureNameWC;
    char    *featureMBS = info->featureNameMBS;
    int     status=0;
    const char *paramName;
    int     enumIndex;

    getParamName(paramIndex, &paramName);
    
    if (!info->exists) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: parameter %s (%d) not associated with Andor feature\n",
            driverName, functionName, paramName, paramIndex);
        return -1;
    }
    if (!info->isImplemented) {
        fprintf(fp, "  %s: not implemented\n", featureMBS);
        return status;
    }
    switch(info->type) {
    case ATint: {
        AT_64 i_value, i_min, i_max;
        status |= AT_GetInt(   handle_, featureWC, &i_value);
        status |= AT_GetIntMin(handle_, featureWC, &i_min);
        status |= AT_GetIntMax(handle_, featureWC, &i_max);
        fprintf(fp, "  %s: type=Integer, range=[%lld : %lld], value=%lld\n", 
                featureMBS, i_min, i_max, i_value);
        } break;
    case ATfloat: {
        double d_value, d_min, d_max;
        status |= AT_GetFloat(   handle_, featureWC, &d_value);
        status |= AT_GetFloatMin(handle_, featureWC, &d_min);
        status |= AT_GetFloatMax(handle_, featureWC, &d_max);
        fprintf(fp, "  %s: type=Float, range=[%f : %f], value=%f\n", 
                featureMBS, d_min, d_max, d_value);
        } break;
    case ATbool: {
        AT_BOOL bool_value;
        status |= AT_GetBool(   handle_, featureWC, &bool_value);
        fprintf(fp, "  %s: type=Bool, value=%s\n", 
                featureMBS, bool_value ? "True" : "False");
        } break;
    case ATenum: {
        int i, enumCount;
        AT_BOOL isImplemented, isAvailable;
        AT_WC enumStringWC[MAX_FEATURE_NAME_LEN];
        char enumString[MAX_FEATURE_NAME_LEN];
        status |= AT_GetEnumCount(handle_, featureWC, &enumCount);
        status |= AT_GetEnumIndex(handle_, featureWC, &enumIndex);
        status |= AT_GetEnumStringByIndex(handle_, featureWC, enumIndex, 
                                          enumStringWC, MAX_FEATURE_NAME_LEN-1);
        WCSToMBS(enumString, enumStringWC, sizeof(enumString)-1);  
        fprintf(fp, "  %s: type=Enum, index=%d, value=%s\n", 
                featureMBS, enumIndex, enumString);
        fprintf(fp, "    %d enum choices:\n", enumCount);
        for (i=0; i<enumCount; i++) {
            status |= AT_IsEnumIndexImplemented(handle_, featureWC, i, &isImplemented);
            status |= AT_IsEnumIndexAvailable(handle_, featureWC, i, &isAvailable);
            status |= AT_GetEnumStringByIndex(handle_, featureWC, i, 
                                              enumStringWC, MAX_FEATURE_NAME_LEN-1);
            WCSToMBS(enumString, enumStringWC, sizeof(enumString)-1); 
            fprintf(fp, "    index=%d, Implemented=%s, Available=%s, string=%s\n",
                    i, isImplemented ? "True" : "False", 
                    isAvailable ? "True" : "False", enumString);
        }
        } break;
    case ATstring: {
        int strLen;
        status |= AT_GetStringMaxLength(handle_, featureWC, &strLen);
        strLen++;
        AT_WC *wcs = new AT_WC[strLen];
        char *mbs = new char[strLen];
        status |= AT_GetString(handle_, featureWC, wcs, strLen);
        WCSToMBS(mbs, wcs, strLen);
        fprintf(fp, "  %s: type=String, value=%s\n", 
                featureMBS, mbs);
        delete wcs;
        delete mbs;
        } break;
    case ATcommand: {
        fprintf(fp, "  %s: type=Command, implemented\n", 
                featureMBS);
        } break;
    }

    /* error messages */
    if(status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error reading values for feature=%s, parameter=%s (index=%d)\n",
            driverName, functionName, featureMBS, paramName, paramIndex);
    }
    return status;
}

int andor3::registerFeature(const AT_WC *feature, Andor3FeatureType type,
                            int paramIndex)
{
    static const char *functionName = "registerFeature";
    int  status = -1;
    char featureName[MAX_FEATURE_NAME_LEN];

    WCSToMBS(featureName, feature, MAX_FEATURE_NAME_LEN);

    featureInfo *info = &featureInfo_[paramIndex];
    info->camera = this;
    info->type = type;
    info->paramIndex = paramIndex;
    wcscpy(info->featureNameWC, feature);
    WCSToMBS(info->featureNameMBS, feature, sizeof(info->featureNameMBS));
    info->exists = true;
    
    status  = AT_IsImplemented(handle_, feature, &info->isImplemented);
    if (!info->isImplemented) return 0;
    status = AT_RegisterFeatureCallback(handle_, feature, 
                                         c_getfeature, info);
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to register feature %s (%d)\n",
            driverName, functionName, featureName, status);   
    }
    return status;
}

int andor3::setAOI()
{
    int status=0;
    int minX, sizeX, binX;
    int minY, sizeY, binY;
    int binning;
    int binValues[] = {1, 2, 3, 4, 8};
    static const char *functionName = "setAOI";

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
        "%s:%s: entry\n",
        driverName, functionName);

    status |= getIntegerParam(ADSizeX, &sizeX);
    status |= getIntegerParam(ADSizeY, &sizeY);
    status |= getIntegerParam(Andor3Binning, &binning);
    status |= getIntegerParam(ADMinX, &minX);
    status |= getIntegerParam(ADMinY, &minY);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error getting values, error=%d\n",
            driverName, functionName, status);   
        return status;
    }
    binX = binValues[binning];
    binY = binValues[binning];
    // There is a bug in the SDK.  
    // It will crash if we try to set the current AOI is not an integer multiple of the binning
    // Work around this by first setting the binning to 1x1 and the size of an integer multiple
    // of the binning
    int maxSizeX, maxSizeY;
    int width, height;
    status |= getIntegerParam(ADMaxSizeX, &maxSizeX);
    status |= getIntegerParam(ADMaxSizeY, &maxSizeY);
    width  = (maxSizeX / binX) * binX;
    height = (maxSizeY / binY) * binY;
    status |= AT_SetEnumIndex(handle_, L"AOIBinning", 0);
    status |= AT_SetInt(handle_, L"AOILeft",   1);
    status |= AT_SetInt(handle_, L"AOITop",    1);
    status |= AT_SetInt(handle_, L"AOIWidth",  width); ;
    status |= AT_SetInt(handle_, L"AOIHeight", height);
    // End of bug workaround
    status |= AT_SetEnumIndex(handle_, L"AOIBinning", binning);
    status |= AT_SetInt(handle_, L"AOIWidth",  sizeX/binX);
    status |= AT_SetInt(handle_, L"AOILeft",   minX);
    status |= AT_SetInt(handle_, L"AOIHeight", sizeY/binY);
    status |= AT_SetInt(handle_, L"AOITop",    minY);
    return status;
}

int andor3::getAOI()
{
    int status=0;
    AT_64 minX, sizeX, binX;
    AT_64 minY, sizeY, binY;
    int binning;
    int binValues[] = {1, 2, 3, 4, 8};
    static const char *functionName = "getAOI";

    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
        "%s:%s: entry\n",
        driverName, functionName);

    status |= AT_GetInt(handle_, L"AOIWidth",  &sizeX);
    status |= AT_GetInt(handle_, L"AOILeft",   &minX);
    status |= AT_GetInt(handle_, L"AOIHeight", &sizeY);
    status |= AT_GetInt(handle_, L"AOITop",    &minY);
    status |= AT_GetEnumIndex(handle_, L"AOIBinning", &binning);
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
        "%s:%s: minX=%lld, sizeX=%lld, minY=%lld, sizeY=%lld, binning=%d\n",
        driverName, functionName, minX, sizeX, minY, sizeY, binning);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error getting values, error=%d\n",
            driverName, functionName, status);   
        return status;
    }
    binX = binValues[binning];
    setIntegerParam(ADBinX, (int)binX);
    binY = binValues[binning];
    setIntegerParam(ADBinY, (int)binY);
    status |= setIntegerParam(ADSizeX, (int)(sizeX*binX));
    status |= setIntegerParam(ADMinX,  (int)minX);
    status |= setIntegerParam(ADSizeY, (int)(sizeY*binY));
    status |= setIntegerParam(ADMinY,  (int)minY);
    status |= setIntegerParam(Andor3Binning, binning);

    /* set NDArray parameters */
    status |= setIntegerParam(NDArraySizeX, (int)sizeX);
    status |= setIntegerParam(NDArraySizeY, (int)sizeY);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error setting values, error=%d\n",
            driverName, functionName, status);   
        return status;
    }
    return status;
}

int andor3::allocateBuffers(void)
{
    int   status;
    AT_64 size;

    freeBuffers();

    status = AT_GetInt(handle_, L"ImageSizeBytes", &size);
    if(status != AT_SUCCESS) {
        size = 11059200; /* 16 bit full size */
        status = AT_SUCCESS;
    }
    imageSize_ = size;

    drvBuffers_ = (AT_U8 **)calloc(maxFrames_, sizeof(AT_U8 *));
    if(drvBuffers_) {
        for(int x = 0; x < maxFrames_; x++) {
            #ifdef _WIN32
               drvBuffers_[x] = (AT_U8*)_aligned_malloc((size_t)size, 8);
            #else
                /* allocate 8 byte aligned buffer */
                if(!posix_memalign((void **)&drvBuffers_[x], 8, size)) {
                    //status |= AT_QueueBuffer(handle_,
                    //                         drvBuffers[x], (int)size);
                } else {   
                    drvBuffers_[x] = NULL;
                }
            #endif
        }
    }

    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:allocateBuffers: Failed to allocate and queue buffers\n",
            driverName);
    }   
    return status;
}

int andor3::freeBuffers()
{
    int status;

    status = AT_Flush(handle_);
    if(drvBuffers_) {
        for(int x = 0; x < maxFrames_; x++) {
            if(drvBuffers_[x]) {
                #ifdef _WIN32
                    _aligned_free(drvBuffers_[x]);
                #else
                    free(drvBuffers_[x]);
                #endif
                drvBuffers_[x] = NULL;
            }
        }
        free(drvBuffers_);
        drvBuffers_ = NULL;
    }
    return status;
}

    
int andor3::connectCamera(void)
{
    static const char *functionName = "connectCamera";

    int status = AT_SUCCESS;


    /* disconnect any connected camera first */
    disconnectCamera();

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: connecting camera %d\n",
        driverName, functionName, id_);

    /* open handle to camera */
    status = AT_Open(id_, &handle_);
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: unable to open camera %d\n",
            driverName, functionName, id_);
        return status;
    }

    allocateBuffers();
    return status;
}

int andor3::disconnectCamera(void) 
{
    static const char *functionName = "disconnectCamera";

    int status;
    int acquiring;


    if(!handle_) {
        return AT_SUCCESS;
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: disconnecting camera %d\n",
        driverName, functionName, id_);

    status = AT_GetBool(handle_, L"CameraAcquiring", &acquiring);
    if(status == AT_SUCCESS && acquiring) {
        status |= AT_Command(handle_, L"Acquisition Stop");
    }

    status |= freeBuffers();
    status |= AT_Close(handle_);

    if(status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error closing camera %d\n",
            driverName, functionName, id_);
    }

    handle_ = 0;
    return status;
}

size_t andor3::WCSToMBS(char *mbs, const AT_WC *wcs, size_t mbsLen)
{
    mbstate_t mbState = {0};
    const AT_WC *pWC = wcs;
    
    return wcsrtombs(mbs, &pWC, mbsLen, &mbState);
}  


asynStatus andor3::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], 
                            size_t nElements, size_t *nIn)
{
    int index = pasynUser->reason;
    int i;
    int enumCount;
    AT_WC enumStringWC[MAX_FEATURE_NAME_LEN];
    char enumString[MAX_FEATURE_NAME_LEN];
    AT_BOOL isImplemented;
    int status;
    static const char *functionName = "readEnum";

    *nIn = 0;
    // Get the featureInfo for this parameter, set if it exists, is implemented, and is ATenum
    featureInfo *info = &featureInfo_[index];    
    if (!info->exists) return asynError;
    if (!info->isImplemented) {
        if (strings[0]) free(strings[0]);
        if (strings[1]) free(strings[1]);
        if (info->type == ATbool) {
            strings[0] = epicsStrDup("N.A. 0");
            strings[1] = epicsStrDup("N.A. 1");
            *nIn = 2;
        } else {
            strings[0] = epicsStrDup("N.A.");
            *nIn = 1;
        }
        return asynSuccess;
    }
    if (info->type != ATenum) return asynError;
    
    status = AT_GetEnumCount(handle_, info->featureNameWC, &enumCount);
    for (i=0; ((i<enumCount) && (i<(int)nElements)); i++) {
        status |= AT_IsEnumIndexImplemented(handle_, info->featureNameWC, i, &isImplemented);
        if (!isImplemented) continue;
        if (strings[*nIn]) free(strings[*nIn]);
        status |= AT_GetEnumStringByIndex(handle_, info->featureNameWC, i, 
                                          enumStringWC, MAX_FEATURE_NAME_LEN-1);
        WCSToMBS(enumString, enumStringWC, sizeof(enumString)-1);  
        strings[*nIn] = epicsStrDup(enumString);
        values[*nIn] = i;
        severities[*nIn] = 0;
        (*nIn)++;
    }
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: error calling AT enum functions, status=%d\n",
            driverName, functionName, status);
    }
    return asynSuccess;
}



asynStatus andor3::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    static const char *functionName = "writeInt32";
    const char *paramName;

    int index = pasynUser->reason;
    int status;

    getParamName(index, &paramName);

    /* set "locked" parameters */
    if(index == NDDataType) {
        value = NDUInt16;
    } else if(index == NDColorMode) {
        value = NDColorModeMono;
    }

    /* set parameter from value */
    status = setIntegerParam(index, value);
    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: failed to write parameter %s = %d\n",
            driverName, functionName, paramName, value);
        return (asynStatus)status;
    }

    if(index == ADAcquire) {
        if(value) {
            setIntegerParam(ADNumImagesCounter, 0);
            setShutter(1);
            epicsEventSignal(startEvent_);
        } else {
            setShutter(0);
            status = AT_Command(handle_, L"AcquisitionStop");
        }
    }
    else if(index == ADImageMode) {
        status = setFeature(ADImageMode);        
    }
    else if(index == ADTriggerMode) {
        status = setFeature(ADTriggerMode);
    }
    else if(index == ADNumExposures) {
        // The SDK requires FrameCount to be desired number of frames times AccumulateCount
        int numImages;
        status  = getIntegerParam(ADNumImages, &numImages);
        status |= setIntegerParam(ADNumImages, value * numImages);
        status |= setFeature(ADNumImages);
        status |= setIntegerParam(ADNumImages, numImages);
        status |= setFeature(ADNumExposures);
    }
    else if(index == ADNumImages) {
        // The SDK requires FrameCount to be desired number of frames times AccumulateCount
        int numExposures;
        status = getIntegerParam(ADNumExposures, &numExposures);
        status |= setIntegerParam(ADNumImages, value * numExposures);
        status |= setFeature(ADNumImages);
        status |= setIntegerParam(ADNumImages, value);
    }
    else if 
       ((index == Andor3Binning) ||
        (index == ADMinX)  ||
        (index == ADSizeX) ||
        (index == ADMinY)  ||
        (index == ADSizeY)) {
        status = setAOI();
    }
    else if(index == ADReadStatus) {
        status  = getFeature(ADStatus);
        status |= getAOI();
    }
    else if(index == Andor3PixelEncoding) {
        status = setFeature(Andor3PixelEncoding);
        if(status == AT_SUCCESS) {
            status = allocateBuffers();
        }
    }
    else if(index == Andor3SensorCooling) {
        status = setFeature(Andor3SensorCooling);
    }
    else if(index == Andor3ShutterMode) {
        status = setFeature(Andor3ShutterMode);
    }
    else if(index == Andor3SoftwareTrigger) {
        if(value) {
            status = AT_Command(handle_, L"SoftwareTrigger");
            setIntegerParam(Andor3SoftwareTrigger, 0);
        }
    }
    else if(index == Andor3TempControl) {
        status = setFeature(Andor3TempControl);
    }
    else if(index == Andor3Overlap) {
        status = setFeature(Andor3Overlap);
    }
    else if(index == Andor3FanSpeed) {
        status = setFeature(Andor3FanSpeed);
    }
    else if(index == Andor3ReadoutRate) {
        status = setFeature(Andor3ReadoutRate);
    }
    else if(index == Andor3PreAmpGain) {
        status = setFeature(Andor3PreAmpGain);
    }
    else if(index == Andor3NoiseFilter) {
        status = setFeature(Andor3NoiseFilter);
    }
    else {
        if(index < FIRST_ANDOR3_PARAM) {
            status = ADDriver::writeInt32(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: error, status=%d param=%s(%d) value=%d\n",
            driverName, functionName, status, paramName, index, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: param=%s(%d) value=%d\n",
            driverName, functionName, paramName, index, value);
    }
    return (asynStatus)status;
}

asynStatus andor3::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    static const char *functionName = "writeFloat64";
    const char *paramName;

    int index = pasynUser->reason;
    int status = asynSuccess;

    getParamName(index, &paramName);

    /* set parameter from value */
    status = setDoubleParam(index, value);
    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: failed to write parameter %s = %f\n",
            driverName, functionName, paramName, value);
        return (asynStatus)status;
    }

    if(index == ADTemperature) {
        status = setFeature(ADTemperature);
    }
    else if(index == ADAcquireTime) {
        status = setFeature(ADAcquireTime);
    }
    else if(index == ADAcquirePeriod) {
        status = setDoubleParam(Andor3FrameRate, value ? 1.0/value : 0);
        if(!status) {
            status = setFeature(Andor3FrameRate);
        }
    }
    else if(index == Andor3FrameRate) {
        status = setFeature(Andor3FrameRate);
    }
    else {    
        if(index < FIRST_ANDOR3_PARAM) {
            status = ADDriver::writeFloat64(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if(status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: error, status=%d param=%s(%d) value=%f\n",
            driverName, functionName, status, paramName, index, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
            "%s:%s: param=%s(%d) value=%f\n",
            driverName, functionName, paramName, index, value);
    }
    return (asynStatus)status;
}



extern "C" int andor3Config(const char *portName, int cameraId, int maxBuffers,
                            size_t maxMemory, int priority, int stackSize,
                            int maxFrames)
{
    new andor3(portName, cameraId, maxBuffers, maxMemory, priority, stackSize,
               maxFrames);
    return(asynSuccess);
}

/** Constructor for Andor3 driver; most parameters are simply passed to
  * ADDriver::ADDriver.
  *
  * After calling the base class constructor this method creates a thread to
  * collect the images from the detector and sets reasonable default values for
  * the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  *
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] cameraId The id number of the Andor camera (see listdevices
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
  * \param[in] maxFrames The number of frame buffers to use in driver.
  */
andor3::andor3(const char *portName, int cameraId, int maxBuffers,
               size_t maxMemory, int priority, int stackSize, int maxFrames)
    : ADDriver(portName, 1, NUM_ANDOR3_PARAMS, maxBuffers, maxMemory,
               asynEnumMask, asynEnumMask, 
               ASYN_CANBLOCK,  /* ASYN_CANBLOCK=1 ASYN_MULTIDEVICE=0 */
               1,              /* autoConnect=1 */
               priority, stackSize),
      handle_(0), id_(cameraId), drvBuffers_(NULL), exiting_(0)
{
    static const char *functionName = "andor3";
    int status;

    /* set max frames */
    if(maxFrames == 0) {
        maxFrames_ = 10;
    } else {
        maxFrames_ = maxFrames;
    }
    
    /* create andor specific parameters */
    createParam(Andor3FrameRateString,        asynParamFloat64, &Andor3FrameRate);
    createParam(Andor3PixelEncodingString,    asynParamInt32,   &Andor3PixelEncoding);
    createParam(Andor3FullAOIControlString,   asynParamInt32,   &Andor3FullAOIControl);
    createParam(Andor3BinningString,          asynParamInt32,   &Andor3Binning);
    createParam(Andor3ShutterModeString,      asynParamInt32,   &Andor3ShutterMode);
    createParam(Andor3SoftwareTriggerString,  asynParamInt32,   &Andor3SoftwareTrigger);
    createParam(Andor3SensorCoolingString,    asynParamInt32,   &Andor3SensorCooling);
    createParam(Andor3TempControlString,      asynParamInt32,   &Andor3TempControl);
    createParam(Andor3TempStatusString,       asynParamInt32,   &Andor3TempStatus);
    createParam(Andor3SerialNumberString,     asynParamOctet,   &Andor3SerialNumber);
    createParam(Andor3FirmwareVersionString,  asynParamOctet,   &Andor3FirmwareVersion);
    createParam(Andor3SoftwareVersionString,  asynParamOctet,   &Andor3SoftwareVersion);
    createParam(Andor3ControllerIDString,     asynParamOctet,   &Andor3ControllerID);
    createParam(Andor3OverlapString,          asynParamInt32,   &Andor3Overlap);
    createParam(Andor3ReadoutRateString,      asynParamInt32,   &Andor3ReadoutRate);
    createParam(Andor3ReadoutTimeString,      asynParamFloat64, &Andor3ReadoutTime);
    createParam(Andor3TransferRateString,     asynParamFloat64, &Andor3TransferRate);
    createParam(Andor3PreAmpGainString,       asynParamInt32,   &Andor3PreAmpGain);
    createParam(Andor3NoiseFilterString,      asynParamInt32,   &Andor3NoiseFilter);
    createParam(Andor3FanSpeedString,         asynParamInt32,   &Andor3FanSpeed);

    featureInfo_ = (featureInfo *)calloc(LAST_ANDOR3_PARAM+1, sizeof(featureInfo));
        
    /* set read-only parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");

    /* open camera (also allocates frames) */
    status = AT_InitialiseLibrary();
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: Andor Library initialization failed (%d)\n",
            driverName, functionName, status);
        return;
    }
    AtInitialized++;

    status = connectCamera();
    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        return;
    }


    status  = setStringParam(ADManufacturer, "Andor");

    /* register features for change callback (invokes callback to set value)*/

    status |= registerFeature(L"CameraModel",              ATstring, ADModel);
    status |= registerFeature(L"SensorWidth",              ATint,    ADMaxSizeX);
    status |= registerFeature(L"SensorHeight",             ATint,    ADMaxSizeY);
    status |= registerFeature(L"SerialNumber",             ATstring, Andor3SerialNumber);
    status |= registerFeature(L"FirmwareVersion",          ATstring, Andor3FirmwareVersion);
    {
        AT_WC wcs[MAX_FEATURE_NAME_LEN];
        char  mbs[MAX_FEATURE_NAME_LEN];
        status |= AT_GetString(AT_HANDLE_SYSTEM, L"SoftwareVersion", wcs, MAX_FEATURE_NAME_LEN);
        WCSToMBS(mbs, wcs, sizeof(mbs));
        setStringParam(Andor3SoftwareVersion, mbs);
    }
    status |= getFeature(Andor3SoftwareVersion, AT_HANDLE_SYSTEM);
    status |= registerFeature(L"ControllerID",             ATstring, Andor3ControllerID);
    status |= registerFeature(L"FullAOIControl",           ATbool,   Andor3FullAOIControl);

    status  = registerFeature(L"AOIWidth",                 ATint,    ADSizeX);
    status |= registerFeature(L"AOIHeight",                ATint,    ADSizeY);
    status |= registerFeature(L"AOILeft",                  ATint,    ADMinX);
    status |= registerFeature(L"AOITop",                   ATint,    ADMinY);
    status |= registerFeature(L"AOIBinning",               ATenum,   Andor3Binning);
    status |= registerFeature(L"ImageSizeBytes",           ATint ,   NDArraySize);

    status |= registerFeature(L"SensorCooling",            ATbool,   Andor3SensorCooling);
    status |= registerFeature(L"TargetSensorTemperature",  ATfloat,  ADTemperature);
    status |= registerFeature(L"SensorTemperature",        ATfloat,  ADTemperatureActual);
    status |= registerFeature(L"TemperatureControl",       ATenum,   Andor3TempControl);
    status |= registerFeature(L"TemperatureStatus",        ATenum,   Andor3TempStatus);
    status |= registerFeature(L"FanSpeed",                 ATenum,   Andor3FanSpeed);
    
    status |= registerFeature(L"CycleMode",                ATenum,   ADImageMode);
    status |= registerFeature(L"ExposureTime",             ATfloat,  ADAcquireTime);
    status |= registerFeature(L"AccumulateCount",          ATint,    ADNumExposures);
    status |= registerFeature(L"FrameCount",               ATint,    ADNumImages);
    status |= registerFeature(L"CameraAcquiring",          ATbool,   ADStatus);

    status |= registerFeature(L"TriggerMode",              ATenum,   ADTriggerMode);
    status |= registerFeature(L"SoftwareTrigger",          ATcommand,Andor3SoftwareTrigger);

    status |= registerFeature(L"FrameRate",                ATfloat,  Andor3FrameRate);
    status |= registerFeature(L"PixelEncoding",            ATenum,   Andor3PixelEncoding);
    status |= registerFeature(L"ElectronicShutteringMode", ATenum,   Andor3ShutterMode);
    status |= registerFeature(L"Overlap",                  ATbool,   Andor3Overlap);
    status |= registerFeature(L"PixelReadoutRate",         ATenum,   Andor3ReadoutRate);
    status |= registerFeature(L"ReadoutTime",              ATfloat,  Andor3ReadoutTime);
    status |= registerFeature(L"MaxInterfaceTransferRate", ATfloat,  Andor3TransferRate);
    status |= registerFeature(L"SimplePreAmpGainControl",  ATenum,   Andor3PreAmpGain);
    status |= registerFeature(L"SpuriousNoiseFilter",      ATbool,   Andor3NoiseFilter);

    if(status != AT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: failed to register all features for camera %d\n",
            driverName, functionName, id_);
    }

    startEvent_ = epicsEventCreate(epicsEventEmpty);

    /* launch image read task */
    epicsThreadCreate("Andor3ImageTask", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_imagetask, this);

    /* launch temp read task */
    epicsThreadCreate("Andor3TempTask", 
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      c_temptask, this);

    /* shutdown on exit */
    epicsAtExit(c_shutdown, this);

    /* Hard coded values for test weekend */
    status  = AT_SetBool(handle_, L"Overlap", AT_TRUE);
    status |= AT_SetBool(handle_, L"SpuriousNoiseFilter", AT_TRUE);

    status |= AT_SetEnumIndex(handle_, L"PixelEncoding", 2);
    status |= AT_SetEnumIndex(handle_, L"SimplePreAmpGainControl", 2);
 
    status |= AT_SetEnumIndex(handle_, L"PixelReadoutRate", 3);
    
    if(status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: failed to set hard coded values: %d\n", 
            driverName, functionName, status);
    }
}



/* Code for iocsh registration */
static const iocshArg andor3ConfigArg0 = {"Port name", iocshArgString};
static const iocshArg andor3ConfigArg1 = {"CameraId", iocshArgInt};
static const iocshArg andor3ConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg andor3ConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg andor3ConfigArg4 = {"priority", iocshArgInt};
static const iocshArg andor3ConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg andor3ConfigArg6 = {"maxFrames", iocshArgInt};
static const iocshArg * const andor3ConfigArgs[] =  {&andor3ConfigArg0,
                                                     &andor3ConfigArg1,
                                                     &andor3ConfigArg2,
                                                     &andor3ConfigArg3,
                                                     &andor3ConfigArg4,
                                                     &andor3ConfigArg5,
                                                     &andor3ConfigArg6};
static const iocshFuncDef configAndor3 = {"andor3Config", 7, andor3ConfigArgs};
static void configAndor3CallFunc(const iocshArgBuf *args)
{
    andor3Config(args[0].sval, args[1].ival, args[2].ival,  args[3].ival, 
                 args[4].ival, args[5].ival, args[6].ival);
}

static void andor3Register(void)
{

    iocshRegister(&configAndor3, configAndor3CallFunc);
}

extern "C" {
epicsExportRegistrar(andor3Register);
}
