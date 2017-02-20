#!/bin/bash

TARGET=opticalFlow
adb root
adb remount
adb shell rm /data/$TARGET
adb push /media/euler/codebase/sdk0801/out/target/product/rk3288/symbols/system/bin/$TARGET /data/
adb shell /data/$TARGET
