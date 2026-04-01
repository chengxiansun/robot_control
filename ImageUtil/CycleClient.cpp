#include "CycleClient.h"

CycleClient::CycleClient(){}

CycleClient::~CycleClient() 
{
	WSACleanup();
	closesocket(m_client_socket);
}

void CycleClient::set(Mat src, string model)
{
	this->m_img = src;
	this->model = model;
}
Mat CycleClient::run()
{
	if (InitSocket())
	{
		printf("connect to server successfully\n");
		if (SendOpcode(1))
		{
			cout << "send opcode ： 1\tSendModel()\n";
			if (SendModel(model.c_str()))
			{
				if (SendOpcode(3))
				{
					cout << "send opcode ： 3\tSendImage()\n";
					if (SendImage())
					{
						Sleep(1000);
						if (SendOpcode(6))
						{
							cout << "send opcode ： 6\tCycleOperation()\n";
							Sleep(1000);
							if (SendOpcode(4))
							{
								cout << "send opcode ： 4\tRecvImage(CycleImage)\n";
								if (RecvImage())
								{
									cout << "Receive Cycle-image successfully\n";
									resize(m_img, m_img, Size(300, 400));
									imwrite(binary_image, m_img);
									Sleep(1000);
									if (SendOpcode(7))
									{
										cout << "send opcode ： 7\tBinaryOperation()\n";
										Sleep(5000);
										if (SendOpcode(5))
										{
											Sleep(1500);
											cout << "send opcode ： 5\tRecvImage(BinaryImage)\n";
											Sleep(1500);
											if (RecvImage())
											{
												Sleep(1500);
												cout << "Receive Binary-image successfully\n";
												resize(m_img, m_img, Size(300, 400));
												imwrite(cycle_image, m_img);
												return m_img;
											}
										}
									}
								}
							}							
						}
					}
				}
			}
		}
	}
}

bool CycleClient::InitSocket(void)
{
	//(1)	加载套接字库
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;
	wVersionRequested = MAKEWORD(1, 1);
	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0)
	{
		printf("Init Windows Socket Failed! Error: %d\n", GetLastError());
		return FALSE;
	}
	// 失败则释放套接字库
	if (LOBYTE(wsaData.wVersion) != 1 || HIBYTE(wsaData.wVersion) != 1)
	{
		WSACleanup();
		return FALSE;
	}

	//(2)	创建套接字
	this->m_client_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (m_client_socket == INVALID_SOCKET)
	{
		printf("Create Socket Failed! Error: %d\n", GetLastError());
		return FALSE;
	}

	//(3)	显示初始化信息
	printf("Client Initialization Succeed!\n");

	//(4)	连接服务器
	SOCKADDR_IN srv_addr;
	srv_addr.sin_addr.S_un.S_addr = inet_addr(SRVIPADDR);
	srv_addr.sin_family = AF_INET;
	srv_addr.sin_port = htons(SRVPORT);
	if (connect(m_client_socket, (SOCKADDR*)&srv_addr, sizeof(SOCKADDR)))
	{
		printf("connect error: %s(errno: %d)\n\n", strerror(errno), errno);
	}

	return true;
}

bool CycleClient::SendOpcode(int opcode)
{
	if (send(m_client_socket, (char *)(&opcode), sizeof(opcode), 0) < 0)
	{
		printf("send opcode %d: %s(errno: %d)\n", opcode, strerror(errno), errno);
		return false;
	}
	return true;
}

bool CycleClient::SendModel(const char* s)
{
	int length = strlen(s);
	send(m_client_socket, (char *)(&length), sizeof(length), 0);
	send(m_client_socket, s, length, 0);
	return true;
}

bool CycleClient::SendImage(void)
{
	MsgHead msg;
	Mat image = this->m_img;
	vector<uchar> vec;
	vector<int> param = vector<int>(2);
	param[0] = IMWRITE_JPEG_QUALITY;
	param[1] = 100;//default(95) 0-100
	imencode(".jpg", image, vec, param);

	msg.imgRows = image.rows;
	msg.imgCols = image.cols;
	msg.size = vec.size();

	//发送信息头
	send(m_client_socket, (char *)(&msg), sizeof(msg), 0);

	char* img = (char*)malloc(vec.size());
	for (int i = 0; i <vec.size(); i++)
	{
		img[i] = vec[i];
	}
	int index = 0;
	while (index < vec.size())
	{
		index += send(m_client_socket, img + index, vec.size() - index, 0);
		cout << "index\t" << index << endl;
	}
	return true;
}

bool CycleClient::RecvImage()
{
	MsgHead msg;
	int msgSize = recv(m_client_socket, (char*)&msg, sizeof(msg), 0);
	cout << "ROWS\t" << msg.imgRows << "\tCOLS\t" << msg.imgCols << "\tSIZE\t" << msg.size << endl;
	cout << "msgSize\t" << msgSize << endl;
	if (msgSize <= 0) return false;
	char buf[1024];
	sprintf(buf, "ROWS\t%d\tCOLS\t%d\tSIZE\t%d\0", msg.imgRows, msg.imgCols, msg.size);

	vector<uchar> vec;
	char* img = (char*)malloc(msg.size);

	int index = 0;
	//client_socket->waitForReadyRead();
	while (index < msg.size)
	{
		index += recv(m_client_socket, img + index,msg.size - index, 0);
		cout << "index\t" << index << endl;
	}

	for (int i = 0; i < msg.size; i++)
	{
		vec.push_back(img[i]);
	}
	Mat image = imdecode((Mat)vec, 1);
	//imshow("TTT", image);
	m_img = image;
	return true;
}