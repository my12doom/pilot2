// x264client.cpp : 定义控制台应用程序的入口点。
//

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
RGBQUAD YUV2RGB(RGBQUAD i);	// YUV is stored in RGB


#define INBUF_SIZE 409600

HWND wnd;
int width = 640;
int height = 480;

// sockes variables and functions
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

AVFrame *RGB32Frame = NULL;
SwsContext * sws_ctx = NULL;
void show_picture(AVFrame *frame)
{
	if (RGB32Frame == NULL)
	{
		sws_ctx = sws_getCachedContext(NULL, frame->width, frame->height, AV_PIX_FMT_YUV420P,
			640, 480, AV_PIX_FMT_BGRA, SWS_BILINEAR, NULL, NULL, NULL);

		RGB32Frame = av_frame_alloc();
		RGB32Frame->data[0] = new uint8_t[640*480*4];
		memset(RGB32Frame->data[0], 0, 640*480*4);
		RGB32Frame->width = 640;
		RGB32Frame->height = 480;
		RGB32Frame->linesize[0] = 640*4;
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
	HANDLE hFile = CreateFile(L"H:\\test.h264", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hFile == INVALID_HANDLE_VALUE)
		return -1;

	// 创建文件映射对象
	HANDLE hFileMap = CreateFileMapping(hFile, NULL, PAGE_READWRITE, 0, 0, NULL);
	if (hFileMap == NULL)
		return -2;

	// 得到文件尺寸
	DWORD dwFileSizeHigh;
	__int64 qwFileSize = GetFileSize(hFile, &dwFileSizeHigh);
	qwFileSize |= (((__int64)dwFileSizeHigh) << 32);
	file_size = qwFileSize;
	// 关闭文件对象
	CloseHandle(hFile);

	// 偏移地址 
	__int64 qwFileOffset = 0;

	// 块大小
	DWORD dwBlockBytes = qwFileSize;

	// 映射视图
	pfile = (LPBYTE)MapViewOfFile(hFileMap,FILE_MAP_ALL_ACCESS, 
	(DWORD)(qwFileOffset >> 32), (DWORD)(qwFileOffset & 0xFFFFFFFF),
	dwBlockBytes);

	if (pfile == NULL)
		return -3;

	// 对映射的视图进行访问
	//for(DWORD i = 0; i < dwBlockBytes; i++)
	//BYTE temp = *(lpbMapAddress + i);

	// 撤消文件映像
	//UnmapViewOfFile(lpbMapAddress);

	return 0;
}

int get_h264_frame(void *buf)
{
	/*
	int frame_size = 0;
	if (fread(&frame_size, 1, 4, f) != 4)
		return 0;
	return fread(buf, 1, frame_size, f);
	*/

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
		return get_h264_frame(buf);
	}

	int size = next_pos - file_pos;
	memcpy(buf, pfile+file_pos, size);
	file_pos += size;

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

DWORD WINAPI video_decode_example(LPVOID p)
{
    AVCodec *codec;
    AVCodecContext *c= NULL;
    int frame, got_picture, len;
    AVFrame *picture;
    uint8_t inbuf[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
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
		//file_pos = (unsigned int)(rand()&0xff) * file_size / 255;
		skip = true;
		break;

	case WM_INITDIALOG:
		{
			CreateThread(NULL, NULL, video_decode_example, NULL, 0, 0);
			SetWindowPos(GetDlgItem(hDlg, IDC_S), NULL, 0, 0, 640, 480, SWP_NOMOVE);
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


int _tmain(int argc, _TCHAR* argv[])
{
	init_file();

	return DialogBox(GetModuleHandle(NULL), MAKEINTRESOURCE(IDD_DIALOG1), NULL, about_window_proc);
}
/*

int clip(int i)
{
	if (i>255)
		return 255;
	if (i<0)
		return 0;
	return i;
}

RGBQUAD YUV2RGB(RGBQUAD i)	// YUV is stored in RGB
{
	RGBQUAD o = {0};

	int y,u,v;
	y = i.rgbRed;
	u = i.rgbGreen - 128;
	v = i.rgbBlue - 128;

	o.rgbRed   = clip(y*1.164+ 0.000*u+ 1.596*v);
	o.rgbGreen = clip(y*1.164+-0.391*u+-0.813*v);
	o.rgbBlue  = clip(y*1.164+ 2.018*u+ 0.000*v);

	return o;
}

RGBQUAD RGB2YUV(RGBQUAD i)	// YUV is stored in RGB
{
	RGBQUAD o = {0};

	int r,g,b;
	r = i.rgbRed;
	g = i.rgbGreen;
	b = i.rgbBlue;

	o.rgbRed   = clip(r*+0.299+ 0.587*g+ 0.114*b);
	o.rgbGreen = clip(r*-0.147+-0.289*g+ 0.436*b + 128);
	o.rgbBlue  = clip(r*+0.615+-0.515*g+-0.100*b + 128);

	return o;
}
*/
