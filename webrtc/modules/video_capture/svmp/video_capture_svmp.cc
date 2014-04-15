/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#define LOG_TAG "fbstream_webrtc"

#include "video_capture_android.h"

#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <stdio.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/types.h>

#include <iostream>
#include <new>

#include "ref_count.h"
#include "trace.h"
#include "thread_wrapper.h"
#include "critical_section_wrapper.h"
#include "svmp_common.h"
#include "video_capture_svmp.h"

// Android stuff
#include <gui/DisplayEventReceiver.h>
#include <utils/Looper.h>
using namespace android;

namespace android {


    // static variables;

    void *MyEventReceiver::obj;

    MyEventReceiver::MyEventReceiver() {
    }

    MyEventReceiver::~MyEventReceiver() {
    }

    void MyEventReceiver::setCapture(void *myobj) {
        obj = myobj;
    }

    bool MyEventReceiver::doCapture() {
        return static_cast<webrtc::videocapturemodule::VideoCaptureModuleV4L2*> (obj)->CaptureProcess();
    }

}

namespace webrtc {

    namespace videocapturemodule {

        // static variables;
        android::LooperHelper VideoCaptureModuleV4L2::Loop;

        VideoCaptureModule* VideoCaptureImpl::Create(const int32_t id,
                const char* deviceUniqueId) {
            RefCountImpl<videocapturemodule::VideoCaptureModuleV4L2>* implementation =
                    new RefCountImpl<videocapturemodule::VideoCaptureModuleV4L2>(id);

            if (!implementation || implementation->Init(deviceUniqueId) != 0) {
                delete implementation;
                implementation = NULL;
            }

            return implementation;
        }

        VideoCaptureModuleV4L2::VideoCaptureModuleV4L2(const int32_t id)
        : VideoCaptureImpl(id),
        _captureThread(NULL),
        _captureCritSect(CriticalSectionWrapper::CreateCriticalSection()),
        _deviceId(-1),
        _deviceFd(-1),
        _buffersAllocatedByDevice(-1),
        _currentWidth(-1),
        _currentHeight(-1),
        _currentFrameRate(-1),
        _captureStarted(false),
        _captureVideoType(kVideoRGB24), // default to virtual frame buffer
        _pool(NULL)
 {

        }

        // hard code to a single device

        int32_t VideoCaptureModuleV4L2::Init(const char* deviceUniqueIdUTF8) {
            char device[32];
            bool found = false;
            int fd;
            sprintf(device, "/dev/graphics/fb0");
            if ((fd = open(device, O_RDONLY)) != -1) {

                if (ioctl(fd, FBIOGET_VSCREENINFO, &_fbdata.var) == 0) {
                    _fbdata.len = _fbdata.var.xres_virtual * _fbdata.var.yres_virtual * (_fbdata.var.bits_per_pixel / 8);
                    found = true;
                }

            }
            close(fd);

            if (!found) {
                WEBRTC_TRACE(webrtc::kTraceError, webrtc::kTraceVideoCapture, _id, "no matching device found");
                return -1;
            }
            _deviceId = 0; //store the device id
            return 0;

        }

        VideoCaptureModuleV4L2::~VideoCaptureModuleV4L2() {
            StopCapture();
            if (_captureCritSect) {
                delete _captureCritSect;
            }
            if (_deviceFd != -1)
                close(_deviceFd);
        }

        int32_t VideoCaptureModuleV4L2::StartCapture(
                const VideoCaptureCapability& capability) {
            if (_captureStarted) {
                if (capability.width == _currentWidth &&
                        capability.height == _currentHeight &&
                        _captureVideoType == capability.rawType) {
                    return 0;
                } else {
                    StopCapture();
                }
            }

            CriticalSectionScoped cs(_captureCritSect);
            //first open /dev/video device
            char device[20];

            sprintf(device, "/dev/graphics/fb%d", _deviceId);
            if ((_deviceFd = open(device, O_RDWR | O_NONBLOCK, 0)) < 0) {
                WEBRTC_TRACE(webrtc::kTraceError, webrtc::kTraceVideoCapture, _id,
                        "error in opening %s errono = %d", device, errno);
                return -1;
            }
            struct fbdata fb_data;
            memset(&fb_data, 0, sizeof (struct fbdata));

            if (ioctl(_deviceFd, FBIOGET_VSCREENINFO, &fb_data.var) < 0) {
                //fb_data.len = fb_data.var.xres_virtual * fb_data.var.yres_virtual * ( fb_data.var.bits_per_pixel / 8 );
                WEBRTC_TRACE(webrtc::kTraceError, webrtc::kTraceVideoCapture, _id,
                        "error in fbdev ioctel %s errono = %d", device, errno);
                return -1;
            }
            const int nFormats = 1;
            unsigned int fmts[nFormats];
            fmts[0] = V4L2_PIX_FMT_RGB565;

            struct v4l2_fmtdesc fmt;
            int fmtsIdx = 0;
            memset(&fmt, 0, sizeof (fmt));
            fmt.index = 0;
            fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            WEBRTC_TRACE(webrtc::kTraceInfo, webrtc::kTraceVideoCapture, _id,
                    "Video Capture enumerates supported image formats:");
            // hardcode to RGB24
            fmt.pixelformat = fmts[0];


            WEBRTC_TRACE(webrtc::kTraceInfo, webrtc::kTraceVideoCapture, _id,
                    "We prefer format %c%c%c%c",
                    fmts[fmtsIdx] & 0xFF, (fmts[fmtsIdx] >> 8) & 0xFF,
                    (fmts[fmtsIdx] >> 16) & 0xFF, (fmts[fmtsIdx] >> 24) & 0xFF);


            struct v4l2_format video_fmt;
            memset(&video_fmt, 0, sizeof (struct v4l2_format));
            video_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            video_fmt.fmt.pix.sizeimage = 0;
            //video_fmt.fmt.pix.width = capability.width;
            video_fmt.fmt.pix.width = fb_data.var.xres;
            video_fmt.fmt.pix.height = fb_data.var.yres;
            video_fmt.fmt.pix.pixelformat = fmts[fmtsIdx];

            if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
                _captureVideoType = kVideoYUY2;
            else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
                _captureVideoType = kVideoI420;
            else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG ||
                    video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_JPEG)
                _captureVideoType = kVideoMJPEG;
            else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) // used in SVMP
                _captureVideoType = kVideoRGB24;
            else if (video_fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565) // to correspond to fbset settings RGB565
                _captureVideoType = kVideoRGB565;


            // initialize current width and height
            _currentWidth = video_fmt.fmt.pix.width;
            _currentHeight = video_fmt.fmt.pix.height;
            _captureDelay = 120;

            // Trying to set frame rate, before check driver capability.
            bool driver_framerate_support = true;
            struct v4l2_streamparm streamparms;
            memset(&streamparms, 0, sizeof (streamparms));
            streamparms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            driver_framerate_support = false;


            // If driver doesn't support framerate control, need to hardcode.
            // Hardcoding the value based on the frame size.
            if (!driver_framerate_support) {
                if (_currentWidth >= 800 && _captureVideoType != kVideoMJPEG) {
                    _currentFrameRate = 15;
                } else {
                    _currentFrameRate = 30;
                }
            }
            WEBRTC_TRACE(webrtc::kTraceError, webrtc::kTraceVideoCapture, _id, "Framerate: setting framerate to %d", _currentFrameRate);


            if (!AllocateVideoBuffers()) {
                WEBRTC_TRACE(webrtc::kTraceError, webrtc::kTraceVideoCapture, _id,
                        "failed to allocate video capture buffers");
                return -1;
            }

            //start capture thread;
            if (!_captureThread) {
                _captureThread = ThreadWrapper::CreateThread(
                        VideoCaptureModuleV4L2::CaptureThreadAsync, this, kHighPriority);
                unsigned int id;
                _captureThread->Start(id);

            }

            _captureStarted = true;
            return 0;


        }

        int32_t VideoCaptureModuleV4L2::StopCapture() {

            if (_captureThread) {
                // Make sure the capture thread stop 
                // Cleanup VSYNC polling thread first
                CleanupCapture();
                _captureThread->SetNotAlive();
                if (_captureThread->Stop()) {
                    delete _captureThread;
                    _captureThread = NULL;
                } else {
                    // Couldn't stop the thread, leak instead of crash.
                    WEBRTC_TRACE(webrtc::kTraceError, webrtc::kTraceVideoCapture, -1,
                            "%s: could not stop capture thread", __FUNCTION__);
                    assert(false);
                }
            }

            CriticalSectionScoped cs(_captureCritSect);
            if (_captureStarted) {
                _captureStarted = false;
                _captureThread = NULL;

                DeAllocateVideoBuffers();
                close(_deviceFd);
                _deviceFd = -1;
            }

            return 0;
        }

        //critical section protected by the caller

        // Need to update for SVMP code.
        // Basically hard code to a single device.
        // and mmap to existing device.

        bool VideoCaptureModuleV4L2::AllocateVideoBuffers() {
            _buffersAllocatedByDevice = 1;
            _pool = new Buffer[1];

            _pool[0].start = (unsigned char *) mmap(NULL, _fbdata.len, PROT_READ, MAP_SHARED, _deviceFd, 0);
            _pool[0].length = _fbdata.len;

            if (MAP_FAILED == _pool[0].start) {
                munmap(_pool[0].start, _pool[0].length);

                return false;
            }

            return true;
        }

        bool VideoCaptureModuleV4L2::DeAllocateVideoBuffers() {
            // unmap buffers
            for (int i = 0; i < _buffersAllocatedByDevice; i++)
                munmap(_pool[i].start, _pool[i].length);

            delete[] _pool;

            // turn off stream
            enum v4l2_buf_type type;
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(_deviceFd, VIDIOC_STREAMOFF, &type) < 0) {
                WEBRTC_TRACE(webrtc::kTraceError, webrtc::kTraceVideoCapture, _id,
                        "VIDIOC_STREAMOFF error. errno: %d", errno);
            }

            return true;
        }

        bool VideoCaptureModuleV4L2::CaptureStarted() {
            return _captureStarted;
        }

        bool VideoCaptureModuleV4L2::CaptureThread(void* obj) {
            return static_cast<VideoCaptureModuleV4L2*> (obj)->CaptureProcess();
        }

        bool VideoCaptureModuleV4L2::CaptureThreadAsync(void* obj) {
            return static_cast<VideoCaptureModuleV4L2*> (obj)->CaptureProcessAsync();
        }

        int receiver(int fd, int events, void* data) {
            // retrieve handle to videocapturemodule
            MyEventReceiver* q = (MyEventReceiver*) data;

            ssize_t n;
            MyEventReceiver::Event buffer[1];

            while ((n = q->getEvents(buffer, 1)) > 0) {
                for (int i = 0; i < n; i++) {
                    if (buffer[i].header.type == MyEventReceiver::DISPLAY_EVENT_VSYNC) {
                        q->doCapture();
                    }
                }
            }
            if (n < 0) {
                printf("error reading events (%s)\n", strerror(-n));
            }

            return 1;
        }

        void VideoCaptureModuleV4L2::CleanupCapture() {
            // shutdown VSYNC polling thread
            Loop.TearDown();

        }

        bool VideoCaptureModuleV4L2::CaptureProcessAsync() {
            android::MyEventReceiver myDisplayEvent;

            myDisplayEvent.setCapture(this);

            Loop.Setup(myDisplayEvent.getFd());
            Loop.mLooper->addFd(myDisplayEvent.getFd(), 0, ALOOPER_EVENT_INPUT, receiver,
                    &myDisplayEvent);

            // set rate to 30 hz ( skip every other event )
            myDisplayEvent.setVsyncRate(2);

            do {
                //printf("about to poll...\n");
                int32_t ret = Loop.mLooper->pollOnce(-1);
                switch (ret) {
                    case ALOOPER_POLL_WAKE:
                        //("ALOOPER_POLL_WAKE\n");
                        break;
                    case ALOOPER_POLL_CALLBACK:
                        //("ALOOPER_POLL_CALLBACK\n");
                        break;
                    case ALOOPER_POLL_TIMEOUT:
                        printf("ALOOPER_POLL_TIMEOUT\n");
                        break;
                        // Looper is killed so exit 
                    case ALOOPER_POLL_ERROR:
                        printf("ALOOPER_POLL_ERROR\n");
                        goto end;
                        break;
                    default:
                        printf("ugh? poll returned %d\n", ret);
                        break;
                }
            } while (1);
end:
            return true;
        }

        bool VideoCaptureModuleV4L2::CaptureProcess() {
            int retVal = 0;
            fd_set rSet;
            struct timeval timeout;


            _captureCritSect->Enter();

            FD_ZERO(&rSet);
            FD_SET(_deviceFd, &rSet);
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            retVal = select(_deviceFd + 1, &rSet, NULL, NULL, &timeout);
            if (retVal < 0 && errno != EINTR) // continue if interrupted
            {
                // select failed
                _captureCritSect->Leave();
                return false;
            } else if (retVal == 0) {
                // select timed out
                _captureCritSect->Leave();
                return true;
            } else if (!FD_ISSET(_deviceFd, &rSet)) {
                // not event on camera handle
                _captureCritSect->Leave();
                return true;
            }

            if (_captureStarted) {
                struct v4l2_buffer buf;
                memset(&buf, 0, sizeof (struct v4l2_buffer));
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;

                VideoCaptureCapability frameInfo;
                frameInfo.width = _currentWidth;
                frameInfo.height = _currentHeight;
                frameInfo.rawType = _captureVideoType;
                buf.bytesused = _currentWidth * _currentHeight * 2;
                // convert to to I420 if needed
                IncomingFrame((unsigned char*) _pool[0].start, buf.bytesused, frameInfo);

            }
            _captureCritSect->Leave();
            usleep(0);
            return true;
        }

        int32_t VideoCaptureModuleV4L2::CaptureSettings(VideoCaptureCapability& settings) {
            settings.width = _currentWidth;
            settings.height = _currentHeight;
            settings.maxFPS = _currentFrameRate;
            settings.rawType = _captureVideoType;

            return 0;
        }
    } // namespace videocapturemodule
} // namespace webrtc
