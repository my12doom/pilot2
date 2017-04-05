// x264client.cpp : 定义控制台应用程序的入口点。
//

#include <assert.h>
#ifndef UINT64_C
#define UINT64_C unsigned __int64
#endif
extern "C"
{
#include <stdint.h>
#include <inttypes.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#include <Windows.h>
#include "resource.h"
#include <tchar.h>
#pragma comment(lib,"ws2_32.lib")

#pragma comment(lib, "libavutil.a")
#pragma comment(lib, "libswscale.a")
//#pragma comment(lib, "libavformat.a")
#pragma comment(lib, "libavcodec.a")
#pragma comment(lib, "libgcc.a")
#pragma comment(lib, "libmingwex.a")
#pragma comment(lib, "libiconv.a")
#pragma comment(lib, "libswresample.a")

#include <HAL/SIL_WIN32/Win32UDP.h>
#include <YAL/fec/reciever.h>
#include <YAL/fec/sender.h>

using namespace SIL_WIN32;
using namespace std;

Win32UDP_RX udp;
Win32UDP_TX udp_tx("192.168.0.197", 0xbbb);


HWND wnd;
int width = 1280;
int height = 720;

void DoEvents()
{
	MSG msg;
	BOOL result;

	while ( ::PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE ) )
	{
		result = ::GetMessage(&msg, NULL, 0, 0);
		if (result == 0) // WM_QUIT
		{                
			::PostQuitMessage(msg.wParam);
			break;
		}
		else if (result == -1)
		{
			// Handle errors/exit application, etc.
		}
		else 
		{
			::TranslateMessage(&msg);
			:: DispatchMessage(&msg);
		}
	}
}


class cb : public IFrameReciever
{
public:
	cb()
	{
		InitializeCriticalSection(&_cs);
	}
	~cb()
	{
		DeleteCriticalSection(&_cs);
	}
	int handle_event()
	{
		return 0;
	}
	int handle_frame(const frame &f)
	{
		// 		printf("frame.v=%d, %d/%d bytes\n", f.integrality, *(int*)f.payload, f.payload_size);
		frame * _frame = clone_frame(&f);

		EnterCriticalSection(&_cs);

		frames.push_back(_frame);

		LeaveCriticalSection(&_cs);
		return 0;
	}
	frame * get_frame()
	{
		EnterCriticalSection(&_cs);

		if (frames.size() == 0)
		{
			LeaveCriticalSection(&_cs);
			return NULL;
		}

		frame * f = frames[0];
		frames.erase(frames.begin());

		LeaveCriticalSection(&_cs);

		return f;
	}
	CRITICAL_SECTION _cs;
	vector<frame*> frames;
};

cb * frame_cache = NULL;
reciever *rec;
FrameSender sender;

AVFrame *RGB32Frame = NULL;
SwsContext * sws_ctx = NULL;
void show_picture(AVFrame *frame)
{
	if (RGB32Frame == NULL)
	{
		sws_ctx = sws_getCachedContext(NULL, frame->width, frame->height, AV_PIX_FMT_YUV420P,
			width, height, AV_PIX_FMT_BGRA, SWS_BILINEAR, NULL, NULL, NULL);

		RGB32Frame = av_frame_alloc();
		RGB32Frame->data[0] = new uint8_t[width*height*4];
		memset(RGB32Frame->data[0], 0, width*height*4);
		RGB32Frame->width = width;
		RGB32Frame->height = height;
		RGB32Frame->linesize[0] = width*4;
	}

	// convert
	sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height, RGB32Frame->data, RGB32Frame->linesize);

	// draw
	RECT rect;
	HWND canvas = ::GetDlgItem(wnd, IDC_S);
	::GetWindowRect(canvas, &rect);

	HDC hdc = ::GetDC(canvas);
	HDC memDC = CreateCompatibleDC(hdc);
	HBITMAP bitmap = CreateCompatibleBitmap(hdc, width, height);
	HGDIOBJ obj = SelectObject(memDC, bitmap);


	SetBitmapBits(bitmap, width * height *4, RGB32Frame->data[0]);

	BitBlt(hdc, 0, 0, width, height, memDC, 0, 0, SRCCOPY);

	DeleteObject(obj);
	DeleteObject(bitmap);
	DeleteDC(memDC);
	::ReleaseDC(wnd, hdc);

	DoEvents();
// 	Sleep(33);
}

int file_size = -1;
LPBYTE pfile;
int file_pos = 0;
int skip = 0;

int init_file()
{
	// 创建文件对象
	HANDLE hFile = CreateFile(L"Z:\\out.h264", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hFile == INVALID_HANDLE_VALUE)
		return -1;

	// 创建文件映射对象
	HANDLE hFileMap = CreateFileMapping(hFile, NULL, PAGE_READWRITE, 0, 0, NULL);
	if (hFileMap == NULL || hFileMap == INVALID_HANDLE_VALUE)
	{
		CloseHandle(hFile);
		return -2;
	}

	file_size = GetFileSize(hFile, NULL);

	// 关闭文件对象
	CloseHandle(hFile);

	// 映射视图
	pfile = (LPBYTE)MapViewOfFile(hFileMap,FILE_MAP_ALL_ACCESS, 0, 0, file_size);

	if (pfile == NULL)
		return -3;

	return 0;
}

int init_udp()
{
	frame_cache = new cb();
	rec = new reciever(frame_cache);
	sender.set_block_device(&udp_tx);

	return 0;
}
int get_h264_frame_from_file(void *buf)
{
	int next_pos = file_pos+1;

	while(next_pos < file_size - 4)
	{
		if (*(int*)(pfile+next_pos) == 0x01000000)
			break;
		else
			next_pos++;
	}

	if (next_pos == file_size - 4)
	{
		file_pos = 0;
		return get_h264_frame_from_file(buf);
	}

	int size = next_pos - file_pos;
	memcpy(buf, pfile+file_pos, size);
	file_pos += size;

	return size;

	if (skip == 49)
	{
		skip = 0;

		unsigned char *p = (unsigned char *)buf;
		for(int i=0; i<size; i++)
			if ( rand() < 10)
				p[i] ^=  0xff;

		return 0;
	}
	else
	{
		skip ++;
		return size;
	}
}

int get_h264_frame(void *buf)
{
	char pkt[4096] = {0};
	while(udp.available())
	{
		int s = udp.read(pkt, sizeof(pkt));
		if ( s> 0)
			rec->put_packet(pkt, s);
		else if (s < 0)
			break;
	}

	// UDP
	frame * f = frame_cache->get_frame();
	if (!f)
	{
		//Sleep(1);

		// test
		static uint8_t *frame = new uint8_t[1024*1024];
		int frame_size = get_h264_frame_from_file(frame+4);
		memcpy(frame, &frame_size, 4);

		sender.send_frame(frame, frame_size+4);

		return 0;
	}

	if (!f->integrality)
	{
		release_frame(f);
		printf("bad frame\n");
		return 0;
	}

	uint32_t size = *(int*)f->payload;
	if (size > f->payload_size-4)
	{
		release_frame(f);
		return 0;
	}

	printf("good frame %d bytes\n", size);
	memcpy(buf, (char*)f->payload+4, size);
	release_frame(f);

	return size;
}

DWORD WINAPI video_decode_example(LPVOID p)
{
    AVCodec *codec;
    AVCodecContext *c= NULL;
    int frame, got_picture, len;
    AVFrame *picture;
	#define INBUF_SIZE 4096000
    uint8_t *inbuf = new uint8_t[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
    AVPacket avpkt;

	avcodec_register_all();
    av_init_packet(&avpkt);

    /* set end of buffer to 0 (this ensures that no overreading happens for damaged mpeg streams) */
    memset(inbuf + INBUF_SIZE, 0, FF_INPUT_BUFFER_PADDING_SIZE);

    printf("Video decoding\n");

    /* find the mpeg1 video decoder */
    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        fprintf(stderr, "codec not found\n");
        exit(1);
    }

    c = avcodec_alloc_context3(codec);
    picture= av_frame_alloc();

    if(codec->capabilities&CODEC_CAP_TRUNCATED)
        c->flags|= CODEC_FLAG_TRUNCATED; /* we do not send complete frames */

	c->thread_count = 4;

    /* For some codecs, such as msmpeg4 and mpeg4, width and height
       MUST be initialized there because this information is not
       available in the bitstream. */

    /* open it */
    if (avcodec_open2(c, codec, NULL) < 0) {
        fprintf(stderr, "could not open codec\n");
        exit(1);
    }

    /* the codec gives us the frame size, in samples */


    frame = 0;
	int frame_read = 0;
    for(;;) {
        avpkt.size = get_h264_frame(inbuf);
        if (avpkt.size < 0)
            break;
		frame_read ++;
		if (avpkt.size == 0)
			continue;

        /* NOTE1: some codecs are stream based (mpegvideo, mpegaudio)
           and this is the only method to use them because you cannot
           know the compressed data size before analysing it.

           BUT some other codecs (msmpeg4, mpeg4) are inherently frame
           based, so you must call them with all the data for one
           frame exactly. You must also initialize 'width' and
           'height' before initializing them. */

        /* NOTE2: some codecs allow the raw parameters (frame size,
           sample rate) to be changed at any frame. We handle this, so
           you should also take care of it */

        /* here, we use a stream based decoder (mpeg1video), so we
           feed decoder and see if it could decode a frame */
        avpkt.data = inbuf;
        while (avpkt.size > 0) {
            len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
            if (len < 0) {
                fprintf(stderr, "Error while decoding frame %d\n", frame);
                break;
            }
            if (got_picture) {
                printf("saving frame %3d/%3d\n", frame, frame_read);
                fflush(stdout);

                /* the picture is allocated by the decoder. no need to
                   free it */
				show_picture(picture);
				frame++;
            }
            avpkt.size -= len;
            avpkt.data += len;
        }
    }

    /* some codecs, such as MPEG, transmit the I and P frame with a
       latency of one frame. You must do the following to have a
       chance to get the last frame of the video */
    avpkt.data = NULL;
    avpkt.size = 0;
    len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
    if (got_picture) {
        printf("saving last frame %3d\n", frame);
        fflush(stdout);

        /* the picture is allocated by the decoder. no need to
           free it */
		show_picture(picture);
        frame++;
    }

    avcodec_close(c);
    av_free(c);
    av_free(picture);

	return 0;
}

INT_PTR CALLBACK about_window_proc( HWND hDlg, UINT msg, WPARAM wParam, LPARAM lParam )
{
	switch( msg ) 
	{
	case WM_COMMAND:
		file_pos = (unsigned int)(rand()&0xff) * file_size / 255;
		skip = true;
		break;

	case WM_INITDIALOG:
		{
			CreateThread(NULL, NULL, video_decode_example, NULL, 0, 0);
			SetWindowPos(GetDlgItem(hDlg, IDC_S), NULL, 0, 0, width, height, SWP_NOMOVE);
			wnd = hDlg;
		}
		break;

	case WM_CLOSE:
		EndDialog(hDlg, 0);
		break;

	default:
		return FALSE;
	}

	return TRUE; // Handled message
}

typedef struct _pkt
{
	uint8_t data[400];
} pkt;


#include <vector>
#include <list>

using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	const int msg_size = 23;
	const int PAR = 100;

	uint8_t data[msg_size];
	uint8_t codeword[msg_size+PAR];
	rsEncoder enc;
	rsDecoder dec;
	enc.init(PAR);
	dec.init(PAR);

	int t = GetTickCount();
	for(int i=0; i<msg_size; i++)
	{
		data[i] = i;
		codeword[i] = i;
	}

	printf("%d\n", GetTickCount()-t);

	printf("%d\n", GetTickCount()-t);
	for(int i=0; i<1000; i++)
	{
		enc.resetData();
		enc.append_data(data, msg_size);
		enc.output(codeword+msg_size);
		for(int i=0; i<msg_size; i++)
			codeword[i] = i;
		memset(codeword, i%(PAR*2/3), PAR/3);

		int v = dec.correct_errors_erasures(codeword, msg_size+PAR, 0, NULL);
		assert(v == 1);
	}

	for(int i=0; i<msg_size; i++)
	{
		if (codeword[i] != i)
		{
			printf("err\n");
			break;
		}
	}

	for(int a = 0; a<256; a++)
	{
		for(int b=0; b<256; b++)
		{
			int ab = gmult(a,b);

			int al = a&0xf0;
			int ar = a&0x0f;

			int ba = gmult(ar,b) ^ gmult(al, b);

			if (ab != ba)
			{
				printf("wtf\n");
				break;
			}
		}
	}

	printf("%d\n", GetTickCount()-t);

	init_file();
	init_udp();

	return DialogBox(GetModuleHandle(NULL), MAKEINTRESOURCE(IDD_DIALOG1), NULL, about_window_proc);
}
