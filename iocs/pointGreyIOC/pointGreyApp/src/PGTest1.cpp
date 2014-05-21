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

#define PGSuccess 0
#define PGError -1

class PGTest
{
public:
    PGTest(int cameraId);

    inline int checkError(Error error, const char *functionName, const char *message);
    int connectCamera();

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
    int traceMask_;
};

/** Constructor for the PGTest class
 */
PGTest::PGTest(int cameraId)
    : cameraId_(cameraId)
{
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

    connectCamera();
}

inline int PGTest::checkError(Error error, const char *functionName, const char *PGRFunction)
{
    if (error != PGRERROR_OK) {
        printf("%s:%s: ERROR calling %s Type=%d Description=%s\n",
            driverName, functionName, PGRFunction, error.GetType(), error.GetDescription());
        return PGError;
    }
    return PGSuccess;
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
    printf(
        "%s::%s called BusManager::GetNumOfCameras, pBusMgr_=%p, numCameras=%d\n",
        driverName, functionName, pBusMgr_, numCameras);
    
    if (numCameras <= 0) {
        printf( 
            "%s:%s: no cameras found\n",
            driverName, functionName);
    }

    if (cameraId_ == 0) {
        printf(
            "%s::%s calling BusManager::GetCameraFromIndex, pBusMgr_=%p, pGuid_=%p\n",
            driverName, functionName, pBusMgr_, pGuid_);
        error = pBusMgr_->GetCameraFromIndex(0, pGuid_);
        if (checkError(error, functionName, "GetCameraFromIndex")) return PGError;
    } else { 
        printf(
            "%s::%s calling BusManager::GetCameraFromSerialNumber, pBusMgr_=%p, cameraId_=%d, pGuid_=%p\n",
            driverName, functionName, pBusMgr_, cameraId_, pGuid_);
        error = pBusMgr_->GetCameraFromSerialNumber(cameraId_, pGuid_);
        if (checkError(error, functionName, "GetCameraFromSerialNumber")) return PGError;
    }
    error = pBusMgr_->GetInterfaceTypeFromGuid(pGuid_, &interfaceType);
    if (checkError(error, functionName, "GetInterfaceTypeFromGuid")) return PGError;
    printf(
        "%s::%s called BusManager::GetInterfaceTypeFromGuid, pBusMgr_=%p, interfaceType=%d\n",
        driverName, functionName, pBusMgr_, interfaceType);
    
    // Create appropriate camera object
    if (interfaceType == INTERFACE_GIGE) {
        pCameraBase_ = new GigECamera;
        pCamera_ = NULL;
        pGigECamera_ = dynamic_cast<GigECamera*>(pCameraBase_);
        if (pGigECamera_ == NULL) {
            printf(
                "%s::%s error casting camera to GigECamera\n",
                driverName, functionName);
            return PGError;
        }
    } else {
        pCameraBase_ = new Camera;
        pGigECamera_ = NULL;
        pCamera_ = dynamic_cast<Camera*>(pCameraBase_);
        if (pCamera_ == NULL) {
            printf(
                "%s::%s error casting camera to Camera\n",
                driverName, functionName);
            return PGError;
        }   
    }

    // Connect to camera
    printf(
        "%s::%s calling CameraBase::Connect, pGuid_=%p\n",
        driverName, functionName, pGuid_);
    error = pCameraBase_->Connect(pGuid_);
    if (checkError(error, functionName, "Connect")) return PGError;

    // Get the camera information
    error = pCameraBase_->GetCameraInfo(pCameraInfo_);
    if (checkError(error, functionName, "GetCameraInfo")) return PGError;
    printf(
        "%s::%s called CameraBase::GetCameraInfo, pCameraInfo_=%p, pCameraInfo_->serialNumber=%d\n",
        driverName, functionName, pCameraInfo_, pCameraInfo_->serialNumber);
    
    Utilities::GetLibraryVersion(&version);
    sprintf(tempString, "%d.%d.%d", version.major, version.minor, version.type);
    printf(
        "%s::%s called Utilities::GetLibraryVersion, version=%s\n",
        driverName, functionName, tempString);
    
    // Get and set the embedded image info
    printf(
        "%s::%s calling CameraBase::GetEmbeddedImageInfo, &embeddedInfo=%p\n",
        driverName, functionName, &embeddedInfo);
    error = pCameraBase_->GetEmbeddedImageInfo(&embeddedInfo);
    if (checkError(error, functionName, "GetEmbeddedImageInfo")) return PGError;
    // Force the timestamp and frame counter information to be on
    embeddedInfo.timestamp.onOff = true;
    embeddedInfo.frameCounter.onOff = true;
    printf(
        "%s::%s calling CameraBase::SetEmbeddedImageInfo, &embeddedInfo=%p\n",
        driverName, functionName, &embeddedInfo);
    error = pCameraBase_->SetEmbeddedImageInfo(&embeddedInfo);
    if (checkError(error, functionName, "SetEmbeddedImageInfo")) return PGError;
    
    return PGSuccess;
}


int main(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: PGTest cameraId\n");
        return -1;
    }
    int cameraId = atoi(argv[1]);
    new PGTest(cameraId);
}

