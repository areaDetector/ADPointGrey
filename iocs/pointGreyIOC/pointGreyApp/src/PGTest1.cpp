/*
 * PGTest.cpp
 *
 * Test program to try to track down problem on Linux which displays flaky behavior.
 * Running this program under the GNU debugger causes a register write error
 *
 * Author: Mark Rivers
 *         Univerity of Chicago
 *
 * Created: May 21, 2014
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
#define PGError  -1

class PGTest
{
public:
    PGTest(int cameraId);

    inline int checkError(Error error, const char *functionName, const char *message);
    int connectCamera();

    /* Data */
    int          cameraId_;
    BusManager   *pBusMgr_;
    PGRGuid      *pGuid_;
    CameraInfo   *pCameraInfo_;
    GigECamera   *pGigECamera_;
};

/** Constructor for the PGTest class
 */
PGTest::PGTest(int cameraId)
    : cameraId_(cameraId)
{
    // Create camera control objects
    pBusMgr_        = new BusManager;
    pGuid_          = new PGRGuid;
    pCameraInfo_    = new CameraInfo;

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
    EmbeddedImageInfo embeddedInfo;
    InterfaceType interfaceType;
    unsigned int numCameras;
    static const char *functionName = "connectCamera";

    error = pBusMgr_->GetNumOfCameras(&numCameras);
    if (checkError(error, functionName, "GetNumOfCameras")) return PGError;
    printf("%s::%s called BusManager::GetNumOfCameras, pBusMgr_=%p, numCameras=%d\n",
        driverName, functionName, pBusMgr_, numCameras);
    
    if (numCameras <= 0) {
        printf("%s:%s: no cameras found\n",
            driverName, functionName);
        return PGError;
    }

    if (cameraId_ == 0) {
        printf("%s::%s calling BusManager::GetCameraFromIndex, pBusMgr_=%p, pGuid_=%p\n",
            driverName, functionName, pBusMgr_, pGuid_);
        error = pBusMgr_->GetCameraFromIndex(0, pGuid_);
        if (checkError(error, functionName, "GetCameraFromIndex")) return PGError;
    } else { 
        printf("%s::%s calling BusManager::GetCameraFromSerialNumber, pBusMgr_=%p, cameraId_=%d, pGuid_=%p\n",
            driverName, functionName, pBusMgr_, cameraId_, pGuid_);
        error = pBusMgr_->GetCameraFromSerialNumber(cameraId_, pGuid_);
        if (checkError(error, functionName, "GetCameraFromSerialNumber")) return PGError;
    }
    error = pBusMgr_->GetInterfaceTypeFromGuid(pGuid_, &interfaceType);
    if (checkError(error, functionName, "GetInterfaceTypeFromGuid")) return PGError;
    printf("%s::%s called BusManager::GetInterfaceTypeFromGuid, pBusMgr_=%p, interfaceType=%d\n",
        driverName, functionName, pBusMgr_, interfaceType);
    
    // Create camera object
    if (interfaceType == INTERFACE_GIGE) {
        pGigECamera_ = new GigECamera;
    } else {
        printf("%s::%s: camera is not GigE camera\n",
            driverName, functionName);
        return PGError;   
    }

    // Connect to camera
    printf("%s::%s calling GigECamera::Connect, pGuid_=%p\n",
        driverName, functionName, pGuid_);
    error = pGigECamera_->Connect(pGuid_);
    if (checkError(error, functionName, "Connect")) return PGError;
    
    // Disable heartbeat timeout
    GigEProperty heartbeat;
    heartbeat.propType = HEARTBEAT;
    heartbeat.value = 0;
    error = pGigECamera_->SetGigEProperty(&heartbeat);
    if (checkError(error, functionName, "SetGigEProperty")) return PGError;
    printf("%s::%s called GigECamera::SetGigEProperty(HEARTBEAT)\n",
        driverName, functionName);

    // Get the camera information
    error = pGigECamera_->GetCameraInfo(pCameraInfo_);
    if (checkError(error, functionName, "GetCameraInfo")) return PGError;
    printf("%s::%s called GigECamera::GetCameraInfo, pCameraInfo_=%p, pCameraInfo_->serialNumber=%d\n",
        driverName, functionName, pCameraInfo_, pCameraInfo_->serialNumber);
    
    // Get and set the embedded image info
    printf("%s::%s calling GigECamera::GetEmbeddedImageInfo, &embeddedInfo=%p\n",
        driverName, functionName, &embeddedInfo);
    error = pGigECamera_->GetEmbeddedImageInfo(&embeddedInfo);
    if (checkError(error, functionName, "GetEmbeddedImageInfo")) return PGError;
    // Force the timestamp and frame counter information to be on
    embeddedInfo.timestamp.onOff = true;
    embeddedInfo.frameCounter.onOff = true;
    printf("%s::%s calling GigECamera::SetEmbeddedImageInfo, &embeddedInfo=%p\n",
        driverName, functionName, &embeddedInfo);
    error = pGigECamera_->SetEmbeddedImageInfo(&embeddedInfo);
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
    Error error;
    PGTest *pPGTest1 = new PGTest(cameraId);
    printf("%s::%s calling GigECamera::Disconnect\n",
        driverName, "main");
    error = pPGTest1->pGigECamera_->Disconnect();
    if (pPGTest1->checkError(error, "main", "Disconnect")) return -1;
    return 0;
}

