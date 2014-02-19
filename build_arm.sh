#!/bin/bash
source build_functions.sh
export PATH=$PATH:../depot_tools
export WEBRTC_SKIP_RESOURCES_DOWNLOAD=1
setconfig 0
. build/android/envsetup.sh 
GYP_GENERATOR_OUTPUT="out_arm" gclient runhooks --force

ninja -C out_arm/out/Debug -j 4 libjingle_peerconnection_so
ninja -C out_arm/out/Debug -j 4 libjingle_peerconnection_jar
ninja -C out_arm/out/Release -j 4 libjingle_peerconnection_so
ninja -C out_arm/out/Release -j 4 libjingle_peerconnection_jar
#ninja -C out_arm/out/Debug -j 4  AppRTCDemo

strip out/Release/libjingle_peerconnection_so
