adb root
adb remount
adb shell rm /data/acantha
adb push ../../../../../out/target/product/rk3288/symbols/system/bin/acantha /data/
adb shell /data/acantha wlan0 -arx