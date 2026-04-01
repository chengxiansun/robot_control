#include "CycleClient.h"
#include "cmdline.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <exception>
#include <map>
#include<fstream>

#if defined(_MSC_VER) || defined(WIN32)  || defined(_WIN32) || defined(__WIN32__) \
    || defined(WIN64)    || defined(_WIN64) || defined(__WIN64__)
	#include <io.h>
	#include <fcntl.h>
	#define READ _read
	#define WRITE _write
	#define FILENO _fileno
	#define SET_BINARY_MODE(handle) _setmode(handle, O_BINARY)
#else
	#include <unistd.h>
	#define READ read
	#define WRITE write
	#define FILENO fileno
	#define SET_BINARY_MODE(handle) ((void)0)
#endif

using namespace std;
using namespace cv;



int main(int argc, const char** argv)
{
	

std:; ifstream file("inputImage.jpg", std::ios::binary | std::ios::ate);
	if (!file.is_open()) {
		std::cerr << "no inputImage.jpg" << std::endl;
		return -1;
	}
	std::streamsize size = file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<uchar>buffer(size);
	if (!file.read(reinterpret_cast<char*>(buffer.data()), size) ){
		std::cerr << "read inputImage.jpg fail" << std::endl;
			return -1;
	}
	file.close();

	cv::Mat image = cv::imdecode(buffer, cv::IMREAD_GRAYSCALE);

	if (!image.data)                              // Check for invalid input
	{
		cerr << "Could not decode the image" << std::endl;
		return -1;
	}
	vector<uchar> out_buf;
	cv::imencode(".bmp", image, out_buf);
	SET_BINARY_MODE(FILENO(stdout));
	WRITE(FILENO(stdout), &out_buf[0], out_buf.size());
	
}