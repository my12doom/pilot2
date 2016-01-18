#include "http_cmd.h"
#include <Windows.h>
#include <Wininet.h>
#include <string.h>
#include <stdio.h>
#include <winsock.h>

#pragma comment(lib, "Wininet.lib")
#pragma comment(lib,"ws2_32.lib")

typedef struct _download_para
{
	const char *url_to_download;
	char *out;
	int outlen;
	bool *cancel;
	HRESULT *hr;
} download_para;

DWORD WINAPI download_thread(LPVOID para)
{
	download_para * p = (download_para*)para;

	char url_to_download[1024];
	strcpy_s(url_to_download, p->url_to_download);
	void *out = malloc(p->outlen);
	memset(out, 0, p->outlen);
	int outlen = p->outlen;
	bool *cancel = p->cancel;
	HRESULT *hr = p->hr;

	HINTERNET HI;
	HI=InternetOpenA("dwindow",INTERNET_OPEN_TYPE_PRECONFIG,NULL,NULL,0);
	if (HI==NULL)
		return E_FAIL;

	HINTERNET HURL;
	HURL=InternetOpenUrlA(HI, url_to_download,NULL,0,INTERNET_FLAG_RELOAD | NULL,0);
	if (HURL==NULL)
		return E_FAIL;

	DWORD byteread = 0;
	DWORD total_got = 0;
	DWORD to_get = p->outlen;
	p->outlen = -1;
	BOOL internetreadfile = FALSE;
	while (total_got < to_get && (internetreadfile = InternetReadFile(HURL,(BYTE*)out+total_got, outlen-total_got, &byteread)) && byteread > 0)
		total_got += byteread;

	InternetCloseHandle(HURL);
	InternetCloseHandle(HI);

	if (!internetreadfile && !*cancel)
		*hr = S_FALSE;

	if (!*cancel)
	{
		*hr = S_OK;
		p->outlen = total_got;
		memcpy(p->out, out, total_got);
	}

	free(out);
	delete cancel;
	return 0;
}

HRESULT download_url(const char *url_to_download, char *out, int *outlen /*= 64*/, int timeout/*=INFINITE*/)
{
	download_para thread_para = {url_to_download, out, outlen ? *outlen : -1, new bool(false), new HRESULT(E_FAIL)};

	HANDLE thread = CreateThread(NULL, NULL, download_thread, &thread_para, NULL, NULL);

	WaitForSingleObject(thread, timeout);

	if (*thread_para.hr == E_FAIL)
		*thread_para.cancel = true;

	HRESULT hr = *thread_para.hr == S_OK ? S_OK: E_FAIL;
	if(outlen)
		*outlen = thread_para.outlen;
	delete thread_para.hr;
	return hr;
}

int http_cmd(const char *cmd, const char *server)
{
	char url[1024];
	char out[1024];
	int count = sizeof(out);
	sprintf(url, "%s?%s", server ? server : "http://192.168.1.254/", cmd);

	return SUCCEEDED(download_url(url, out, &count, 3000)) ? 0 : -1;
}

int last_test_80 = 0;
int last_result = -1;

int test80(const char *address)
{
	if (GetTickCount() - last_test_80 < 1000)
		return last_result;
	

	struct hostent *host;
	if((host=gethostbyname(address))==NULL)                              //取得主机IP地址               
	{               
		fprintf(stderr,"Gethostname error, %s\n", strerror(errno));             
		goto fail;
	}       
	int sockfd=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
	if(sockfd==-1)                                                         //建立SOCKET连接             
	{               
		fprintf(stderr,"Socket Error:%s\a\n",strerror(errno));          
		goto fail;
	}       
	// 客户程序填充服务端的资料         
	struct sockaddr_in server_addr = {0};   
	server_addr.sin_family=AF_INET; 
	server_addr.sin_port=htons(80); 
	server_addr.sin_addr=*((struct in_addr *)host->h_addr);         

	//set Recv and Send time out  
	int TimeOut=3000; //设置发送超时6秒  
	if(::setsockopt(sockfd,SOL_SOCKET,SO_SNDTIMEO,(char *)&TimeOut,sizeof(TimeOut))==SOCKET_ERROR){  
		return 0;  
	}  
	TimeOut=6000;//设置接收超时6秒  
	if(::setsockopt(sockfd,SOL_SOCKET,SO_RCVTIMEO,(char *)&TimeOut,sizeof(TimeOut))==SOCKET_ERROR){
		return 0;
	}
	//设置非阻塞方式连接
	unsigned long ul = 1;
	int ret = ioctlsocket(sockfd, FIONBIO, (unsigned long*)&ul);
	if(ret==SOCKET_ERROR)return 0;

// 	// 客户程序发起连接请求   
// 	if(connect(sockfd,(struct sockaddr *)(&server_addr),sizeof(struct sockaddr))==-1)//连接网站         
// 	{
// 		fprintf(stderr,"Connect Error:%s\a\n",strerror(errno));
// 		goto fail;
// 	}
	connect(sockfd,(const struct sockaddr *)&server_addr,sizeof(server_addr));  

	//select 模型，即设置超时  
	struct timeval timeout ;  
	fd_set r;  

	FD_ZERO(&r);  
	FD_SET(sockfd, &r);  
	timeout.tv_sec = 3; //连接超时15秒  
	timeout.tv_usec =0;  
	ret = select(0, 0, &r, 0, &timeout);  
	if ( ret <= 0 )  
	{  
		::closesocket(sockfd);  
		goto fail;
	}  

	closesocket(sockfd);

	last_test_80 = GetTickCount();
	last_result = 0;

	return 0;

fail:
	::closesocket(sockfd);  
	last_test_80 = GetTickCount();
	last_result = -1;
	return -1;
}