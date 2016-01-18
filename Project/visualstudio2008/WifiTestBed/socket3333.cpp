#include "socket3333.h"
#include <Windows.h>
#include <stdio.h>

int last_packet_tick = 0;
int sockfd;

int last_packet_time()
{
	return GetTickCount() - last_packet_tick;
}

bool thread_up = false;

DWORD last_hello = 0;
DWORD WINAPI reader_thread(LPVOID p)
{
	while(true)
	{
		Sleep(10);

		if(last_packet_time() > 3000)
			continue;

		char buf[500];
		memset(buf, 0, sizeof(buf));
		if (recv(sockfd, buf, sizeof(buf), 0) > 0)
		{
			printf(buf);
			if (strrchr(buf, '\n'))
				last_packet_tick = GetTickCount();
		}
	}
}

DWORD WINAPI heart_beat_thread(LPVOID p)
{
	while(true)
	{
		Sleep(10);

		if (GetTickCount() > last_hello + 1000)
		{
			printf("hello\n");
			send(sockfd, "hello\n", 6, 0);
			last_hello = GetTickCount();
		}
	}
}

int connect()
{
	if (!thread_up)
	{
		thread_up = true;
		CreateThread(NULL, NULL, reader_thread, NULL, NULL, NULL);
		CreateThread(NULL, NULL, heart_beat_thread, NULL, NULL, NULL);
	}

	if (last_packet_time() < 1000)
		return 0;

	disconnect();

	struct hostent *host;
	if((host=gethostbyname("192.168.1.254"))==NULL)                              //取得主机IP地址               
	{               
		fprintf(stderr,"Gethostname error, %s\n", strerror(errno));             
		goto fail;
	}       
	sockfd=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
	if(sockfd==-1)                                                         //建立SOCKET连接             
	{               
		fprintf(stderr,"Socket Error:%s\a\n",strerror(errno));          
		goto fail;
	}       
	// 客户程序填充服务端的资料         
	struct sockaddr_in server_addr = {0};   
	server_addr.sin_family=AF_INET; 
	server_addr.sin_port=htons(3333); 
	server_addr.sin_addr=*((struct in_addr *)host->h_addr);         


	// 客户程序发起连接请求   
	if(connect(sockfd,(struct sockaddr *)(&server_addr),sizeof(struct sockaddr))==-1)//连接网站         
	{
		fprintf(stderr,"Connect Error:%s\a\n",strerror(errno));
		goto fail;
	}
	int buf = 0;
	setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, (char*)&buf, sizeof(buf));
	send(sockfd, "hello\n", 6, 0);

	last_packet_tick = GetTickCount();
	printf("3333 connected\n");
	Sleep(200);
	last_hello = 0;

	return 0;

fail:
	disconnect();
	return -1;
}

int disconnect()
{
	printf("disconnect\n");
	closesocket(sockfd);

	return 0;
}

int cmd(const char *cmd)
{
	return send(sockfd, cmd, strlen(cmd), 0);
}