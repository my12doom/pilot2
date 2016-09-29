# Description:
# Date: 09-08
# Version:v 0.0.0

LOCAL_PATH:= $(call my-dir)

#show parameters
$(warning "LOCAL_PATH = $(LOCAL_PATH)")

SRC = system/eulerOpenGlPro/HAL
EXEDIR=/media/euler/newvolume/3288sdk/sdk0801/system/eulerOpenGlPro/eulerspace-euler_pilot/binary_release/rkPilotEXE
PRODUCTDIR=/media/euler/newvolume/3288sdk/sdk0801/out/target/product/rk3288/obj/EXECUTABLES/sensorTest_intermediates/LINKED
include $(CLEAR_VARS)

#local pilot source file
LOCAL_SRC_FILES := \
		testMain.cpp \
        ../../HAL/AInterface/ASPI.cpp \
        ../../HAL/AInterface/AGpio.cpp \
        ../../HAL/Sensors/MPU6000.cpp \
        ../../modules/Protocol/data_protocol.cpp
        
LOCAL_C_INCLUDES :=  \
			system/eulerOpenGlPro \
            system/eulerOpenGlPro/modules \
            kernel/include/uapi \
            system/eulerOpenGlPro/HAL/Android
#LOCAL_CFLAGS += -Wno-multichar

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:= sensorTest
$(shell cp $(PRODUCTDIR)/$(LOCAL_MODULE) $(EXEDIR))
include $(BUILD_EXECUTABLE)
#include $(BUILD_SHARED_LIBRARY)




