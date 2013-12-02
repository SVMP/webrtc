#!/bin/bash
#gclient sync --nohooks
#export PATH=$PATH:../depot_tools
export PATH=$PATH:/home/apyles/android-sdk-linux/platform-tools:/home/apyles/depot_tools:/usr/local/bin
export WEBRTC_SKIP_RESOURCES_DOWNLOAD=1
#. build/android/envsetup.sh --target-arch=x86
GYP_GENERATOR_OUTPUT="out_linux" gclient runhooks --force

ninja $1 -C out_linux/out/Debug -j 4  peerconnection_server
ninja $1 -C out_linux/out/Debug -j 4   peerconnection_client
ninja $1 -C out_linux/out/Debug -j 4   vie_auto_test
echo java stuff building:
ninja -C out_linux/out/Debug -j 4 libjingle_peerconnection_so
ninja -C out_linux/out/Debug -j 4 libjingle_peerconnection_jar
ninja -C out_linux/out/Release -j 4 libjingle_peerconnection_so
ninja -C out_linux/out/Release -j 4 libjingle_peerconnection_jar
#ninja -C out/Release -j 4  All
