#include "encoder.h"

#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/foundation/ABuffer.h>
#include <media/ICrypto.h>
#include <media/stagefright/MediaCodecList.h>
#include <gui/Surface.h>

using namespace android;



android_video_encoder::android_video_encoder()
{
	next_input_index = -1;
	out_buffer = new uint8_t[1024*1024];
}

android_video_encoder::~android_video_encoder()
{
	delete [] out_buffer;
}

int android_video_encoder::init(int width, int height, int bitrate, float frame_rate/* = 25*/, int color_format /*= OMX_COLOR_FormatYUV420Planar*/)
{
	sp<ALooper> looper = new ALooper;
    looper->setName("codec_looper");
    looper->start();

	codec = MediaCodec::CreateByType(looper, "video/avc", true);

	printf("codec=%08x\n", codec.get());
	AString name;
	codec->getName(&name);
	printf("codec name = %s\n", name.c_str());


	sp<AMessage> format = new AMessage;
	format->setInt32("width", width);
	format->setInt32("height", height);
	format->setString("mime", "video/avc");
	format->setInt32("color-format", OMX_COLOR_FormatYUV420Planar);
	format->setInt32("bitrate", bitrate);
	format->setFloat("frame-rate", frame_rate);
	format->setInt32("i-frame-interval", 10);
	format->setInt32("profile", OMX_VIDEO_AVCProfileHigh);
	format->setInt32("level", OMX_VIDEO_AVCLevel4);
	int mbs = (((width + 15) / 16) * ((height + 15) / 16) * 10) / 100;

	format->setInt32("intra-refresh-mode", OMX_VIDEO_IntraRefreshCyclic);
    format->setInt32("intra-refresh-CIR-mbs", mbs);

	status_t err = codec->configure(format, NULL, NULL, MediaCodec::CONFIGURE_FLAG_ENCODE);
	printf("codec->configure()=%d\n", err);
	codec->start();

	codec->getInputBuffers(&inputBuffers);
	codec->getOutputBuffers(&outputBuffers);

	input_buffer_size = width * height * 3 / 2;				// TODO: fix me

	return 0;
}

int android_video_encoder::destroy()
{
	return 0;
}

int android_video_encoder::get_spspps(void *out, int max_byte_count)
{
	return 0;
}

void* android_video_encoder::get_next_input_frame_pointer()
{
	if (next_input_index != -1)
		return inputBuffers.itemAt(next_input_index)->data();

	status_t err = codec->dequeueInputBuffer(&next_input_index, -1);
	if (err != OK)
		return NULL;

	const sp<ABuffer> &dstBuffer = inputBuffers.itemAt(next_input_index);
	dstBuffer->setRange(0, input_buffer_size);

	return dstBuffer->data();
}

int android_video_encoder::encode_next_frame()
{
	if (next_input_index == -1)
		return -1;				// call get_next_input_frame_pointer() to get a pointer and fill it first,

	const sp<ABuffer> &dstBuffer = inputBuffers.itemAt(next_input_index);

	status_t err = codec->queueInputBuffer(next_input_index, 0, dstBuffer->size(), 0, 0);

	next_input_index = -1;

	return err;
}

int android_video_encoder::get_encoded_frame(uint8_t **pp)
{
	if (!pp)
		return -1;

	size_t mIndex;
	size_t mOffset;
	size_t mSize;
	int64_t mPresentationTimeUs;
	uint32_t mFlags;

	status_t err = codec->dequeueOutputBuffer(&mIndex, &mOffset, &mSize, &mPresentationTimeUs, &mFlags, 1);

	if (err == OK)
	{
		const sp<ABuffer> &outBuffer = outputBuffers.itemAt(mIndex);

		//printf("outBuffer = %d, %d\n", outBuffer->offset(), outBuffer->size());
		memcpy(out_buffer, outBuffer->data(), outBuffer->size());

		codec->releaseOutputBuffer(mIndex);

		*pp = out_buffer;
		return outBuffer->size();
	}

	return 0;
}
