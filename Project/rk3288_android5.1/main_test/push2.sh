adb root
adb remount
adb shell rm /data/acantha
adb push /media/euler/codebase/sdk0801/out/target/product/rk3288/symbols/system/bin/acantha /data/
adb shell /data/acantha wlan0 -rx