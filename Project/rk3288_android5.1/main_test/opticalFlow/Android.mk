
LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

#STL for C++ compile
STL_PATH=/media/euler/codebase/sdk0801/prebuilts/ndk/9/sources/cxx-stl/gnu-libstdc++/4.9/libs/armeabi-v7a
STL_PATH2=/media/euler/codebase/sdk0801/prebuilts/ndk/9/sources/cxx-stl/EH/stlport/libs/armeabi-v7a
OPENCV_LIBS_PATH=system/eulerPilotProject/euler_pilot/modules/OpenCV-android-sdk/sdk/native/libs/armeabi-v7a
OPENCV_3RD_LIBS_PATH=system/eulerPilotProject/euler_pilot/modules/OpenCV-android-sdk/sdk/native/3rdparty/libs/armeabi-v7a

#local pilot source file
#Iv4l2.cpp 
LOCAL_SRC_FILES := \
	opticalFlow.cpp \
	checkCapablities.cpp \
	cameraCtrl.cpp \
	OV7740Control.cpp\
	../../../../HAL/sensors/AFlow.cpp \
    ../../../../HAL/sensors/ACamera.cpp \
    ../../../../HAL/sensors/AVideo.cpp \
	../../../../BSP/boards/rk3288UAV/ARCOUT.cpp \
	../../../../HAL/rk32885.1/ASPI.cpp \
	../../../../HAL/rk32885.1/AGpio.cpp \
	../../../../HAL/rk32885.1/ASysTimer.cpp \
	../../../../HAL/rk32885.1/AUIOTimer.cpp \
	../../../../HAL/rk32885.1/AUART.cpp \
	../../../../HAL/rk32885.1/AStorage.cpp \
	../../../../HAL/rk32885.1/ATimer.cpp \
	../../../../HAL/rk32885.1/ACriticalSection.cpp \
	../../../../HAL/rk32885.1/AIMUFIFO.cpp \
	../../../../HAL/rk32885.1/AI2C.cpp \
	../../../../modules/utils/param.cpp \
	../../../../modules/utils/space.cpp \
	../../../../modules/utils/gauss_newton.cpp \
	../../../../modules/utils/vector.cpp \
	../../../../modules/utils/console.cpp \
	../../../../modules/utils/ymodem.cpp \
	../../../../modules/utils/SEGGER_RTT.c \
	../../../../modules/utils/log_android.cpp \
	../../../../modules/Algorithm/pos_controll.cpp \
	../../../../modules/Algorithm/pos_controll_old.cpp \
	../../../../modules/Algorithm/pos_estimator.cpp \
	../../../../modules/Algorithm/pos_estimator2.cpp \
	../../../../modules/Algorithm/attitude_controller.cpp \
	../../../../modules/Algorithm/ahrs.cpp \
	../../../../modules/Algorithm/motion_detector.cpp \
	../../../../modules/Algorithm/ekf_estimator.cpp \
	../../../../modules/Algorithm/altitude_controller.cpp \
	../../../../modules/Algorithm/altitude_estimator.cpp \
	../../../../modules/Algorithm/altitude_estimator2.cpp \
	../../../../modules/Algorithm/of_controller2.cpp \
	../../../../modules/Algorithm/battery_estimator.cpp \
	../../../../modules/Algorithm/mag_calibration.cpp \
	../../../../modules/Algorithm/ekf_lib/src/init_ekf_matrix.c \
	../../../../modules/Algorithm/ekf_lib/src/INS_SetState.c \
	../../../../modules/Algorithm/ekf_lib/src/LinearFG.c \
	../../../../modules/Algorithm/ekf_lib/src/RungeKutta.c \
	../../../../modules/Algorithm/ekf_lib/src/INS_CovariancePrediction.c \
	../../../../modules/Algorithm/ekf_lib/src/INS_Correction.c \
	../../../../modules/Algorithm/ekf_lib/src/quaternion_to_euler.c \
	../../../../modules/Algorithm/ekf_lib/src/ned2body.c \
	../../../../modules/Algorithm/ekf_lib/src/body2ned.c \
	../../../../modules/Algorithm/ekf_lib/src/f.c \
	../../../../modules/Algorithm/ekf_lib/src/normlise_quaternion.c \
	../../../../modules/Algorithm/ekf_lib/src/LinearizeH.c \
	../../../../modules/Algorithm/ekf_lib/src/inv.c \
	../../../../modules/Algorithm/ekf_lib/src/h.c \
	../../../../modules/Algorithm/ekf_lib/src/rt_nonfinite.c \
	../../../../modules/main/mode_basic.cpp \
	../../../../modules/main/mode_althold.cpp \
	../../../../modules/main/mode_of_loiter.cpp \
	../../../../modules/main/mode_poshold.cpp \
	../../../../modules/main/mode_RTL.cpp \
	../../../../modules/math/matrix.cpp \
	../../../../modules/math/LowPassFilter2p.cpp \
	../../../../HAL/Resources.cpp \
	../../../../HAL/sensors/MPU6000.cpp \
	../../../../HAL/sensors/MS5611_SPI.cpp \
	../../../../HAL/sensors/HMC5983SPI.cpp \
	../../../../HAL/sensors/UartUbloxNMEAGPS.cpp \
	../../../../HAL/sensors/EBusIn.cpp  \
    ../../../../HAL/sensors/ASonar.cpp \
	../libyuv/source/compare.cpp \
	../libyuv/source/scale.cpp \
	../libyuv/source/scale_neon.cpp \
	../libyuv/source/scale_common.cpp \
	../libyuv/source/scale_any.cpp \
	../libyuv/source/convert.cpp \
	../libyuv/source/row_neon.cpp \
	../libyuv/source/cpu_id.cpp \
	../libyuv/source/planar_functions.cpp \
	../libyuv/source/row_any.cpp \
	../libyuv/source/row_common.cpp \
	../encoder.cpp \
	
LOCAL_LDFLAGS	+= \
	 -Wl,--start-group \
	$(OPENCV_LIBS_PATH)/libopencv_highgui.a \
	$(OPENCV_LIBS_PATH)/libopencv_calib3d.a\
	$(OPENCV_LIBS_PATH)/libopencv_core.a\
	$(OPENCV_LIBS_PATH)/libopencv_features2d.a\
	$(OPENCV_LIBS_PATH)/libopencv_flann.a\
	$(OPENCV_LIBS_PATH)/libopencv_highgui.a\
	$(OPENCV_LIBS_PATH)/libopencv_imgcodecs.a\
	$(OPENCV_LIBS_PATH)/libopencv_imgproc.a\
	$(OPENCV_LIBS_PATH)/libopencv_core.a \
	$(OPENCV_LIBS_PATH)/libopencv_ml.a\
	$(OPENCV_LIBS_PATH)/libopencv_objdetect.a\
	$(OPENCV_LIBS_PATH)/libopencv_photo.a\
	$(OPENCV_LIBS_PATH)/libopencv_shape.a\
	$(OPENCV_LIBS_PATH)/libopencv_stitching.a\
	$(OPENCV_LIBS_PATH)/libopencv_superres.a\
	$(OPENCV_LIBS_PATH)/libopencv_ts.a\
	$(OPENCV_LIBS_PATH)/libopencv_video.a\
	$(OPENCV_LIBS_PATH)/libopencv_videoio.a\
	$(OPENCV_LIBS_PATH)/libopencv_videostab.a \
	$(OPENCV_3RD_LIBS_PATH)/libIlmImf.a \
	$(OPENCV_3RD_LIBS_PATH)/liblibtiff.a\
	$(OPENCV_3RD_LIBS_PATH)/liblibjasper.a\
	$(OPENCV_3RD_LIBS_PATH)/liblibwebp.a \
	$(OPENCV_3RD_LIBS_PATH)/liblibjpeg.a \
	$(OPENCV_3RD_LIBS_PATH)/libtbb.a \
	$(OPENCV_3RD_LIBS_PATH)/liblibpng.a\
	$(STL_PATH)/libgnustl_static.a \
	-Wl,--end-group
		
LOCAL_C_INCLUDES :=  \
	system/eulerPilotProject/acantha \
	system/eulerPilotProject/acantha/modules \
	system/eulerPilotProject/acantha/modules/Algorithm/ekf_lib/inc \
	system/eulerPilotProject/acantha/Project/rk3288_android5.1/main_test/libyuv/include \
	system/eulerPilotProject/acantha/Project/rk3288_android5.1/ \
	frameworks/av/media/libstagefright \
	frameworks/av/media/libstagefright/include \
	frameworks/av/media/libmediaplayerservice \
	frameworks/native/include/media/openmax \
	system/eulerPilotProject/euler_pilot/modules/OpenCV-android-sdk/sdk/native/jni/include \
	external/libcxx/include \
	hardware/libhardware/include \
	system/core/libion/kernel-headers/linux \
	system/eulerPilotProject/acantha/Project/rk3288_android5.1/main_test/libyuv/include \
	system/eulerPilotProject/acantha/Project/rk3288_android5.1/ \

LOCAL_SHARED_LIBRARIES := \
	libutils \
	libbinder \
	libui \
	libgui \
	libcamera_metadata \
	libcameraservice \
	libcamera_client \
	libstagefright libmedia libstagefright_foundation\
	libion \
    libvpu \
    
LOCAL_CFLAGS := -Wno-format -Wno-unused -Wno-unused-parameter -Wfatal-errors -Wno-non-virtual-dtor
LOCAL_LDLIBS := -ldl -lm -lz -llog 
#LOCAL_RTTI_FLAG := -frtti
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:= opticalFlow
LOCAL_STATIC_LIBRARIES += \
		#libc++


include $(BUILD_EXECUTABLE)
