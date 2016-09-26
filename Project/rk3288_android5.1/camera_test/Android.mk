LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
  
LOCAL_SRC_FILES:=main.cpp \
		fec/append.cpp \
		fec/frame.cpp \
		fec/GFMath.cpp \
		fec/oRS.cpp \
		fec/reciever.cpp\
		fec/sender.cpp

LOCAL_C_INCLUDES := frameworks/av/media/libstagefright \
	frameworks/av/media/libstagefright/include \
	frameworks/av/media/libmediaplayerservice \
	$(TOP)/frameworks/native/include/media/openmax \

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

LOCAL_MODULE:= my12doom_camera
  
LOCAL_MODULE_TAGS := tests

include $(BUILD_EXECUTABLE)
