#ifndef DEVICE_INFO_SVMP_H_4PN2JCQE
#define DEVICE_INFO_SVMP_H_4PN2JCQE

#include "../video_capture_impl.h"
#include "../device_info_impl.h"
#include "svmp_common.h"
namespace webrtc
{
namespace videocapturemodule
{
class DeviceInfoSVMP: public DeviceInfoImpl
{
public:
    DeviceInfoSVMP(const int32_t id);
    virtual ~DeviceInfoSVMP();
    virtual uint32_t NumberOfDevices();
    virtual int32_t GetDeviceName(
        uint32_t deviceNumber,
        char* deviceNameUTF8,
        uint32_t deviceNameLength,
        char* deviceUniqueIdUTF8,
        uint32_t deviceUniqueIdUTF8Length,
        char* productUniqueIdUTF8=0,
        uint32_t productUniqueIdUTF8Length=0);
    /*
    * Fills the membervariable _captureCapabilities with capabilites for the given device name.
    */
    virtual int32_t CreateCapabilityMap (const char* deviceUniqueIdUTF8);
    virtual int32_t DisplayCaptureSettingsDialogBox(
        const char* /*deviceUniqueIdUTF8*/,
        const char* /*dialogTitleUTF8*/,
        void* /*parentWindow*/,
        uint32_t /*positionX*/,
        uint32_t /*positionY*/) { return -1;}
    int32_t FillCapabilityMap(int fd);
    int32_t Init();
    //struct fbdata  fb_data;
private:

    bool IsDeviceNameMatches(const char* name, const char* deviceUniqueIdUTF8);
};
} // namespace videocapturemodule
} // namespace webrtc

#endif /* end of include guard: DEVICE_INFO_SVMP_H_4PN2JCQE */
