#!/bin/bash
#gclient sync --nohooks
#export PATH=$PATH:../depot_tools
export PATH=$PATH:/home/apyles/android-sdk-linux/platform-tools:/home/apyles/depot_tools:/usr/local/bin
export WEBRTC_SKIP_RESOURCES_DOWNLOAD=1
. build/android/envsetup.sh 
GYP_GENERATOR_OUTPUT="out_arm" gclient runhooks --force

ninja -C out_arm/out/Debug -j 4  AppRTCDemo
#ninja -C out/Release -j 4  All
