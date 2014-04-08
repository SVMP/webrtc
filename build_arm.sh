#!/bin/bash
source build_functions.sh
export PATH=$PATH:../depot_tools
export WEBRTC_SKIP_RESOURCES_DOWNLOAD=1
STRIP=`pwd`/third_party/android_tools/ndk/toolchains/arm-linux-androideabi-4.4.3/prebuilt/linux-x86_64/arm-linux-androideabi/bin/strip

echo prepping for ARM build..
cp mitrebuild/common-arm.gypi build/common.gypi


setconfig 0
. build/android/envsetup.sh 
GYP_GENERATOR_OUTPUT="out_arm" gclient runhooks --force

ninja -C out_arm/out/Debug -j 4 libjingle_peerconnection_so
ninja -C out_arm/out/Debug -j 4 libjingle_peerconnection_jar
ninja -C out_arm/out/Release -j 4 libjingle_peerconnection_so
ninja -C out_arm/out/Release -j 4 libjingle_peerconnection_jar
#ninja -C out_arm/out/Debug -j 4  AppRTCDemo

$STRIP out_arm/out/Debug/libjingle_peerconnection_so.so
$STRIP out_arm/out/Release/libjingle_peerconnection_so.so
