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

#define LOG_TAG "video_capture_svmp"

#include <android/log.h>
#include <android/native_window.h>

// uncomment this for debugging
//#define DEBUG_WEBRTC_VSYNC

#ifdef DEBUG_WEBRTC_VSYNC
#define DEBUG_VSYNC_PRINT 1
#else
#define DEBUG_VSYNC_PRINT 0
#endif

#define LOGG(...) \
 do { if(DEBUG_VSYNC_PRINT) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__); } while (0)



#include "../video_capture_impl.h"


// Android stuff
#ifndef HAVE_PTHREADS
#define HAVE_PTHREADS
#endif
#include <gui/DisplayEventReceiver.h>
#include <utils/Looper.h>
#include <utils/Thread.h>



namespace android {

    class myLooperCallback : public LooperCallback {
    public:

        myLooperCallback() {
        }

        ~myLooperCallback() {
            LOGG("!!myLooperCallback destructor");
        }
        virtual int handleEvent(int fd, int events, void *data);

    };

    class MyEventReceiver : public DisplayEventReceiver {
    public:

        MyEventReceiver() {
        }

        virtual ~MyEventReceiver() {
            LOGG("!!myEventReceiver destructor");
        }

        void setCapture(void *myobj);
        static bool doCapture();

    private:
        static void *obj;

    };

    class LooperHelper {
    public:

        LooperHelper() {
            _setup = false;
            _capturing = false;
        }

        ~LooperHelper() {
            LOGG("LooperHelper destructor");
        }
        sp<Looper> mLooper;
        sp<myLooperCallback> receiver;
        MyEventReceiver *myDisplayEvent;

        void Setup(void *myobj); //{                      
        bool loop();

        /*
        void StopPolling(){
            _capturing = false;
        } */

        void TearDown();

    private:
        int _fd;
        bool _capturing;
        bool _setup;

    };
}

namespace webrtc {
    class CriticalSectionWrapper;
    class ThreadWrapper;
    namespace videocapturemodule {

        class VideoCaptureModuleV4L2 : public VideoCaptureImpl {
        public:
            VideoCaptureModuleV4L2(int32_t id);
            virtual ~VideoCaptureModuleV4L2();
            virtual int32_t Init(const char* deviceUniqueId);
            virtual int32_t StartCapture(const VideoCaptureCapability& capability);
            virtual int32_t StopCapture();
            virtual bool CaptureStarted();
            virtual int32_t CaptureSettings(VideoCaptureCapability& settings);            
            bool CaptureProcess();


        private:

            enum {
                kNoOfV4L2Bufffers = 4
            };

            static bool CaptureThread(void*);            
            void CleanupCapture();
            android::LooperHelper* Loop;           

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

            struct Buffer {
                void *start;
                size_t length;
            };
            Buffer *_pool;
        };
    } // namespace videocapturemodule
} // namespace webrtc

#endif /* end of include guard: VIDEO_CAPTURE_SVMP_H_5YT21RQO */
