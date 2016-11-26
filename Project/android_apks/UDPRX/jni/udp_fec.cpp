#include <jni.h>
#include <pthread.h>
#include <unistd.h>

#define  LOG_TAG    "fec"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

#include <stdio.h>
#include <android/log.h>
// net
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <vector>

#include <YAL/fec/reciever.h>

#define CLASS com_yetanother_station_FrameReciever
#define NAME2(CLZ, FUN) Java_##CLZ##_##FUN
#define NAME1(CLZ, FUN) NAME2(CLZ, FUN)
#define NAME(FUN) NAME1(CLASS,FUN)

using namespace std;

typedef struct _packet
{
	int size;
	uint8_t data[4096];
} packet;

vector<packet> packets;

int socket_descriptor = -1;
struct sockaddr_in address;
int port = 0xbbb;
pthread_t prx;
pthread_t ptx;
pthread_mutex_t cs;
bool init = false;
float gimbal = 0;

void* udp_rx_thread(void* para)
{
	LOGE("udp rx start");
	while(init)
	{
		packet p;

		int size_of_address = sizeof(address);
		p.size = recvfrom(socket_descriptor, (char*)p.data, sizeof(p.data), 0, (struct sockaddr *)&address, &size_of_address);

		//LOGE("udp rx %d bytes", p.size);

		pthread_mutex_lock(&cs);

		packets.push_back(p);

		pthread_mutex_unlock(&cs);

	}
	LOGE("udp rx exit");
	return 0;
}

bool tx_now = false;
void* udp_tx_thread(void* para)
{
	LOGE("udp hearbeat start");

	bool first = false;

	while(init)
	{
		struct sockaddr_in address_tx;
		memset(&address_tx, 0, sizeof(address));
		address_tx.sin_family = AF_INET;
		address_tx.sin_addr.s_addr = inet_addr("192.168.43.1");
		address_tx.sin_port = htons(port);

		char data[15] = "hello";
		memcpy(data+5, &gimbal, 4);

		sendto(socket_descriptor, data, first ? 12 : 9, 0, (struct sockaddr *)&address_tx, sizeof(address_tx));
		first = false;
		tx_now = false;
		for(int i=0; i<30 && !tx_now; i++)
			usleep(1000);
	}

	LOGE("udp hearbeat exit");
	return 0;
}

class cb : public IFrameReciever
{
public:
	cb()
	{
		pthread_mutex_init(&_cs, NULL);
	}
	~cb()
	{
		pthread_mutex_destroy(&_cs);
	}
	int handle_event()
	{
		return 0;
	}
	int handle_frame(const frame &f)
	{
 		LOGE("frame.v=%d, %d/%d bytes\n", f.integrality, *(int*)f.payload, f.payload_size);
		frame * _frame = clone_frame(&f);

		pthread_mutex_lock(&_cs);

		frames.push_back(_frame);

		pthread_mutex_unlock(&_cs);
		return 0;
	}
	frame * get_frame()
	{
		pthread_mutex_lock(&_cs);

		if (frames.size() == 0)
		{
			pthread_mutex_unlock(&_cs);
			return NULL;
		}

		frame * f = frames[0];
		frames.erase(frames.begin());

		pthread_mutex_unlock(&_cs);

		return f;
	}
	pthread_mutex_t _cs;
	vector<frame*> frames;
};

cb * frame_cache = new cb();

pthread_t pfec;
void* fec_thread(void* para)
{
	LOGE("fec thread start");
	reciever *rec = new reciever(frame_cache);
	while(init)
	{
		// get a udp packet and feed it to reciever
		pthread_mutex_lock(&cs);

		if (packets.size() == 0)
		{
			pthread_mutex_unlock(&cs);
			usleep(10000);
			continue;
		}

		packet p = packets[0];
		packets.erase(packets.begin());
		pthread_mutex_unlock(&cs);

		rec->put_packet(p.data, p.size);
	}
	LOGE("fec thread exit");
}

extern "C" 
{
	JNIEXPORT jint NAME(init)(JNIEnv * env, jobject obj)
	{
		if (init)
			return 1;

		LOGE("Hello JNI!\n");
		init = true;

		socket_descriptor = socket(AF_INET, SOCK_DGRAM, 0);  
		if (socket_descriptor <0) 
		{      
			printf("error opening udp socket"); 
			return -1;
		}

		// port
		memset(&address, 0, sizeof(address));
		address.sin_family = AF_INET;
		address.sin_addr.s_addr = htonl(INADDR_ANY);
		address.sin_port = htons(port);

		bind(socket_descriptor, (struct sockaddr *)&address, sizeof(address));

		pthread_mutex_init(&cs, NULL);
		pthread_create(&ptx,NULL,udp_tx_thread,NULL);
		pthread_create(&prx,NULL,udp_rx_thread,NULL);
		pthread_create(&pfec,NULL,fec_thread,NULL);

		return 1;
	}

	JNIEXPORT jint NAME(destroy)(JNIEnv * env, jobject obj)
	{
		LOGE("Bye JNI!\n");

		if (!init)
		{
			LOGE("x JNI!\n");
			return 1;
		}

		init = false;
		pthread_join(ptx,NULL);
		pthread_join(prx,NULL);
		pthread_join(pfec,NULL);

		close(socket_descriptor);


		LOGE("ByeBye JNI!\n");

		return 1;
	}

	JNIEXPORT jint NAME(SetGimbal)(JNIEnv * env, jobject obj, jfloat v)
	{
		float PI = acos(-1.0f);
		if (v > PI)
			v = PI;
		if (v < -PI)
			v = -PI;
		
		if (abs(gimbal - v) > PI/90)
			tx_now = true;
		gimbal = v;
		
		return 0;
	}


	JNIEXPORT jbyteArray NAME(readnonblock)(JNIEnv * env, jobject obj)
	{
		// check for frame result
		frame * f = frame_cache->get_frame();

		if (!f)
			return env->NewByteArray(0);

		if (!f->integrality)
		{
			release_frame(f);
			return env->NewByteArray(0);
		}

		uint32_t size = *(int*)f->payload;
		if (size > f->payload_size-4 || size <= 0)
		{
			release_frame(f);
			return env->NewByteArray(0);
		}

		jbyteArray rtn =env->NewByteArray(size);
		env->SetByteArrayRegion(rtn, 0, size, (jbyte*)f->payload+4);
		release_frame(f);

		LOGE("new frame\n");

		return rtn;
	}

}
