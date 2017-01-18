adb -s F8QWG6LRRY shell rm /data/YAP_camera_test
adb -s F8QWG6LRRY push /media/euler/codebase/sdk0801/out/target/product/rk3288/obj/EXECUTABLES/YAP_camera_test_intermediates/YAP_camera_test /data/
adb -s F8QWG6LRRY shell /data/YAP_camera_test