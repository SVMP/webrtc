#!/bin/bash
#gclient sync --nohooks
export PATH=$PATH:../depot_tools
export WEBRTC_SKIP_RESOURCES_DOWNLOAD=1
. build/android/envsetup.sh --target-arch=x86
gclient runhooks

#ninja -C out/Debug -j 4  All
ninja -C out/Debug -j 4 svmp-fbstream-webrtc
cp out/Debug/svmp-fbstream-webrtc out/Debug/svmp-fbstream-webrtc-dbg
strip out/Debug/svmp-fbstream-webrtc
ninja -C out/Release -j 4 svmp-fbstream-webrtc
ninja -C out/Debug -j 4 libjingle_peerconnection_so
ninja -C out/Debug -j 4 libjingle_peerconnection_jar
ninja -C out/Release -j 4 libjingle_peerconnection_so
ninja -C out/Release -j 4 libjingle_peerconnection_jar

cp out/Debug/svmp-fbstream-webrtc out/Debug/svmp-fbstream-webrtc-dbg
strip out/Debug/svmp-fbstream-webrtc
strip out/Release/svmp-fbstream-webrtc
# This is what needs to go to AOSP build.
strip out/Release/libjingle_peerconnection_so
