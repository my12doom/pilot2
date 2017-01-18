LOCAL_PATH:= $(call my-dir)
YAL_PATH:= ../../../modules/YAL
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
		$(YAL_PATH)/fec/append.cpp \
		$(YAL_PATH)/fec/frame.cpp \
		$(YAL_PATH)/fec/GFMath.cpp \
		$(YAL_PATH)/fec/oRS.cpp \
		$(YAL_PATH)/fec/reciever.cpp\
		$(YAL_PATH)/fec/sender.cpp \
		libyuv/source/compare.cpp \
		libyuv/source/scale.cpp \
		libyuv/source/scale_neon.cpp \
		libyuv/source/scale_common.cpp \
		libyuv/source/scale_any.cpp \
		libyuv/source/convert.cpp \
		libyuv/source/row_neon.cpp \
		libyuv/source/cpu_id.cpp \
		libyuv/source/planar_functions.cpp \
		libyuv/source/row_any.cpp \
		libyuv/source/row_common.cpp \
		main.cpp \
		encoder.cpp \



LOCAL_C_INCLUDES := frameworks/av/media/libstagefright \
	frameworks/av/media/libstagefright/include \
	frameworks/av/media/libmediaplayerservice \
	$(TOP)/frameworks/native/include/media/openmax \
	$(LOCAL_PATH)/../../../ \
	$(LOCAL_PATH)/../../../modules \
	$(LOCAL_PATH)/libyuv/include \


LOCAL_CFLAGS := -Wno-format -Wno-unused -Wno-unused-parameter

LOCAL_SHARED_LIBRARIES := \
	libcutils \
	libutils \
	libbinder \
	libui \
	libgui \
	libGLESv1_CM\
	libEGL \
	libhardware \
	libcamera_metadata \
	libcameraservice \
	libcamera_client \
	libstagefright libmedia libstagefright_foundation\

LOCAL_MODULE:= YAP_camera_test
  
LOCAL_MODULE_TAGS := tests

include $(BUILD_EXECUTABLE)
