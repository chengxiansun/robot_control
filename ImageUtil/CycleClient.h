#ifndef CYCLECLIENT_H
#define CYCLECLIENT_H

#include <stdio.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <WinSock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <iostream>
#pragma comment(lib, "ws2_32.lib")
#define SRVIPADDR "192.168.125.201"
#define SRVPORT 8080

using namespace std;
using namespace cv;

//ÕºœÒ–≈œ¢Õ∑
struct MsgHead
{
	int imgRows;
	int imgCols;
	int size;
};

class CycleClient {

public:
	CycleClient();
	~CycleClient();
	void set(Mat, string);
	Mat run();

private:
	Mat m_img;
	string model;
	SOCKET m_client_socket;
	char* binary_image = ".\\Image\\binary.jpg";
	char* cycle_image = ".\\Image\\cycle.jpg";

	bool InitSocket(void);
	bool SendOpcode(int);
	bool SendModel(const char*);
	bool SendImage(void);
	bool RecvImage(void);

};

#endif // !CYCLECLIENT_H
