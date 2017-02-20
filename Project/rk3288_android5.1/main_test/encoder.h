#pragma once

#include <media/stagefright/MediaCodec.h>
#include <media/openmax/OMX_IVCommon.h>
#include <media/openmax/OMX_Video.h>

class android_video_encoder
{
public:
	android_video_encoder();
	~android_video_encoder();

	int init(int wdith, int height, int bitrate, float frame_rate = 25, int color_format = OMX_COLOR_FormatYUV420Planar);
	int destroy();

	int get_spspps(void *out, int max_byte_count);

	void* get_next_input_frame_pointer();
	int encode_next_frame();
	int get_encoded_frame(uint8_t **pp);

protected:
	android::sp<android::MediaCodec> codec;
	android::Vector<android::sp<android::ABuffer> > inputBuffers;
	android::Vector<android::sp<android::ABuffer> > outputBuffers;

	int input_buffer_size;

	size_t next_input_index;

	uint8_t *out_buffer;
};
