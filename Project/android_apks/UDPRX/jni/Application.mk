# armeabi/armeabi-v7a
APP_ABI := armeabi
#APP_ABI := armeabi-v7a
# default depends on AndroidManifest.xml
APP_OPTIM := release
# default is -O2
OPT_CFLAGS := -O3 -mlong-calls -fstrict-aliasing -fprefetch-loop-arrays -ffast-math
ifeq ($(APP_ABI),armeabi-v7a)
OPT_CFLAGS += -mfpu=neon -mtune=cortex-a8 -ftree-vectorize -mvectorize-with-neon-quad -mfloat-abi=softfp
else
OPT_CFLAGS += -DCMP_HAVE_VFP -mfloat-abi=softfp -mfpu=vfp -marm -march=armv6j
endif
OPT_CPPFLAGS := $(OPT_CLFAGS)
# override default
APP_CFLAGS := $(APP_CFLAGS) $(OPT_CFLAGS)
APP_CPPFLAGS := $(APP_CPPFLAGS) $(OPT_CPPFLAGS) 
# stl
APP_STL := stlport_static
