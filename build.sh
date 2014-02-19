#!/bin/bash
#gclient sync --nohooks
source build_functions.sh

config() {
	export PATH=$PATH:../depot_tools
	export WEBRTC_SKIP_RESOURCES_DOWNLOAD=1
	export GYP_DEFINES="build_svmp_video_capture=1"
	setconfig 1
	. build/android/envsetup.sh --target-arch=x86
	gclient runhooks
}

if [ "$1" = "config" ] ; then
	echo running config
	config;
fi
echo starting build

#ninja -C out/Debug -j 4  All
#ninja -C out/Debug -j 4 svmp-fbstream-webrtc
#cp out/Debug/svmp-fbstream-webrtc out/Debug/svmp-fbstream-webrtc-dbg
#strip out/Debug/svmp-fbstream-webrtc
#ninja -C out/Release -j 4 svmp-fbstream-webrtc
ninja -C out/Debug -j 4 libjingle_peerconnection_so
#ninja -C out/Debug -j 4 libjingle_peerconnection_jar
#ninja -C out/Release -j 4 libjingle_peerconnection_so
#ninja -C out/Release -j 4 libjingle_peerconnection_jar

#cp out/Debug/svmp-fbstream-webrtc out/Debug/svmp-fbstream-webrtc-dbg
#strip out/Debug/svmp-fbstream-webrtc
#strip out/Release/svmp-fbstream-webrtc
# This is what needs to go to AOSP build.
#strip out/Release/libjingle_peerconnection_so.so
cp out/Debug/libjingle_peerconnection_so.so out/Debug/libjingle_peerconnection_so_dbg.so
strip out/Debug/libjingle_peerconnection_so.so
