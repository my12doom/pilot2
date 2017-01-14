# Copyright (C) 2010 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
LOCAL_PATH := $(call my-dir)
FEC_PATH := ../../../../modules/YAL/fec

include $(CLEAR_VARS)

LOCAL_MODULE    := fec
LOCAL_SRC_FILES := \
	$(FEC_PATH)/append.cpp \
	$(FEC_PATH)/frame.cpp \
	$(FEC_PATH)/GFMath.cpp \
	$(FEC_PATH)/oRS.cpp \
	$(FEC_PATH)/reciever.cpp \
	$(FEC_PATH)/sender.cpp \
	udp_fec.cpp

LOCAL_LDLIBS    := -llog
LOCAL_STATIC_LIBRARIES := 
LOCAL_CFLAGS := 
LOCAL_C_INCLUDES := ../../../../modules \

include $(BUILD_SHARED_LIBRARY)
