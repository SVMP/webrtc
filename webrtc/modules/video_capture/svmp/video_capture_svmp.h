/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef VIDEO_CAPTURE_SVMP_H_5YT21RQO
#define VIDEO_CAPTURE_SVMP_H_5YT21RQO



#include "common_types.h"
#include "../video_capture_impl.h"

// Android stuff
#include <gui/DisplayEventReceiver.h>
#include <utils/Looper.h>

namespace android
{

class MyEventReceiver:  public DisplayEventReceiver {
	public:
	MyEventReceiver();
	~MyEventReceiver();
	void setCapture(void *myobj);
	static bool doCapture();

	private:
	static void *obj;

};


}

namespace webrtc
{
class CriticalSectionWrapper;
class ThreadWrapper;
namespace videocapturemodule
{
class VideoCaptureModuleV4L2: public VideoCaptureImpl
{
public:
    VideoCaptureModuleV4L2(int32_t id);
    virtual ~VideoCaptureModuleV4L2();
    virtual int32_t Init(const char* deviceUniqueId);
    virtual int32_t StartCapture(const VideoCaptureCapability& capability);
    virtual int32_t StopCapture();
    virtual bool CaptureStarted();
    virtual int32_t CaptureSettings(VideoCaptureCapability& settings);
    bool CaptureProcessAsync();
    bool CaptureProcess();

private:
    enum {kNoOfV4L2Bufffers=4};

    static bool CaptureThread(void*);
    static bool CaptureThreadAsync(void*);


    bool AllocateVideoBuffers();
    bool DeAllocateVideoBuffers();

    ThreadWrapper* _captureThread;
    CriticalSectionWrapper* _captureCritSect;

    int32_t _deviceId;
    int32_t _deviceFd;
    struct fbdata _fbdata;

    int32_t _buffersAllocatedByDevice;
    int32_t _currentWidth;
    int32_t _currentHeight;
    int32_t _currentFrameRate;
    bool _captureStarted;
    RawVideoType _captureVideoType;
    struct Buffer
    {
        void *start;
        size_t length;
    };
    Buffer *_pool;
};
} // namespace videocapturemodule
} // namespace webrtc

#endif /* end of include guard: VIDEO_CAPTURE_SVMP_H_5YT21RQO */
