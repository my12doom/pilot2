adb root
adb remount
adb shell rm /data/acanthaCopy
adb push /media/euler/codebase/sdk0801/out/target/product/rk3288/symbols/system/bin/acanthaCopy /data/
adb shell /data/acanthaCopy
