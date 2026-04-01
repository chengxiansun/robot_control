#include "CycleClient.h"
#include "cmdline.h"
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <exception>
#include <map>

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

enum Command
{
	BINARIZATION,
	FACE_DETECT,
	CYCLECONVERT
};


struct Options
{
	Command cmd;
	union Param
	{
		struct BinarizationParam
		{
			int ada_blk_size;
			float ada_offset;
			int dark_threshold;
		}binarization;

		struct FaceDetectParam
		{
			float min_size_factor;
		}face_detect;
		
		struct CycleConvertParam
		{
			int model;
		}cycleconvert;
	} param;
};

Options parseCheckOptions(int argc, const char** argv)
{
	map<Command, string> cmd_str_map;
	cmd_str_map[Command::BINARIZATION] = "Binarization";
	cmd_str_map[Command::FACE_DETECT] = "FaceDetect";
	cmd_str_map[Command::CYCLECONVERT] = "CycleConvert";

	// Build string to Command map
	map<string, Command> str_cmd_map;
	for (const auto &pair : cmd_str_map)
	{
		str_cmd_map[pair.second] = pair.first;
	}

	const string str_cmd_type = "Command";
	const string str_dark_threshold = "DarkThreshold";
	const string str_ada_blk_size = "AdaptiveBlkSize";
	const string str_ada_offset = "AdaptiveOffset";
	const string str_min_size_factor = "MinSizeFactor";
	const string str_cycle_convert_model = "CycleModel";

	cmdline::parser parser;

	// Option setting
	parser.footer("\n Image Util. The input is stdin, and output is stdout.");
	parser.add<string>(str_cmd_type, 'c', "Command type. [FaceDetect | Binarization | CycleConvert]", true);

	// setting of binarization
	parser.add<int>(str_dark_threshold, 't', "Required when Binarization. Threshold in global binarization", false);
	parser.add<int>(str_ada_blk_size, 's', "Required when Binarization. Block size in local binarization, should be odd number", false);
	parser.add<float>(str_ada_offset, 'a', "Required when Binarization. Offset in local binarization", false, 4.5f);

	// setting of face detect
	parser.add<float>(str_min_size_factor, 'm', "Required when FaceDetect. The minimum face area region factor", false);

	// setting of CycleConvert
	parser.add<int>(str_cycle_convert_model, 'd', "Required when CycleConvert. The ModelName of CycleConvert", false);

	// Run parser.
	// It returns only if command line arguments are valid.
	// If arguments are invalid, a parser output error msgs then exit program.
	// If help flag ('--help' or '-?') is specified, a parser output usage message then exit program.
	parser.parse_check(argc, argv);

	string cmd_str = parser.get<string>(str_cmd_type);
	auto finded_cmd = str_cmd_map.find(cmd_str);
	if (finded_cmd == str_cmd_map.end())
	{
		cerr << "Unknown Command!" << endl << endl;
		cerr << parser.usage() << endl;
		exit(1);
	}

	Options ret;
	ret.cmd = finded_cmd->second;
	switch (ret.cmd)
	{
	case Command::BINARIZATION:
		ret.param.binarization.dark_threshold = parser.getExisted<int>(str_dark_threshold);
		ret.param.binarization.ada_blk_size = parser.getExisted<int>(str_ada_blk_size);
		ret.param.binarization.ada_offset = parser.getExisted<float>(str_ada_offset);
		break;

	case Command::FACE_DETECT:
		ret.param.face_detect.min_size_factor = parser.getExisted<float>(str_min_size_factor);
		break;

	case Command::CYCLECONVERT:
		ret.param.cycleconvert.model = parser.getExisted<int>(str_cycle_convert_model);
		break;

	default:
		cerr << "Unhandled Command type" << endl;
		exit(1);
	}


	return ret;
}

std::vector<char> stdinReadToEnd( )
{
	SET_BINARY_MODE(FILENO(stdin));
	const int kBufSize = 10240;
	char* buf = new char[kBufSize];
	std::deque<char> data;
	while (true)
	{
		int readbytes = READ(FILENO(stdin), buf, kBufSize);
		if (readbytes != 0)
		{
			data.insert(data.end(), buf, buf + readbytes);
		}
		else
		{
			break;
		}
	}

	std::vector<char> ret;

	if (!data.empty())
	{
		ret.reserve(data.size());
		ret.insert(ret.begin(), data.begin(), data.end());
	}
	return ret;
}

cv::Mat imageBinarization(const cv::Mat& src,
	int dark_thresh, int adaptive_blksize, float ada_thresh_off)
{
	if (src.channels() != 1)
	{
		throw runtime_error("Source image must in gray scale");
	}

	cv::Mat thresh_image;
	cv::Mat thresh_mask;
	cv::Mat ret_image;
	cv::adaptiveThreshold(
		src, ret_image,
		255.0,
		cv::ADAPTIVE_THRESH_MEAN_C,
		cv::THRESH_BINARY,
		adaptive_blksize, ada_thresh_off);


	cv::threshold(src, thresh_image, dark_thresh, 255, cv::THRESH_BINARY);
	cv::bitwise_not(thresh_image, thresh_mask);

	//cvCopy(&thresh_image , &ret_image, &thresh_mask);
	Mat ipl_thresh_image = thresh_image;
	Mat ipl_thresh_mask = thresh_mask;
	Mat ipl_ret = ret_image;
	
	cv::copyTo(ipl_thresh_image , ipl_ret, ipl_thresh_mask);

	// ipl_ret and ret_image has the same content of image
	return ret_image;

}

int handler_ImageBinarization(const cv::Mat& src, const Options& options)
{

	cv::Mat bin_image = imageBinarization(
		src, 
		options.param.binarization.dark_threshold, 
		options.param.binarization.ada_blk_size, 
		options.param.binarization.ada_offset);

	vector<uchar> out_buf;
	cv::imencode(".bmp", bin_image, out_buf);
	SET_BINARY_MODE(FILENO(stdout));
	WRITE(FILENO(stdout), &out_buf[0], out_buf.size());

	return 0;
}

int handler_FaceDetect(const cv::Mat& src, const Options& options)
{
	// Init classifier
	string  cascade_path = "haarcascade_frontalface_default.xml";
	cv::CascadeClassifier cascade;
	if (!cascade.load(cascade_path))
	{
		cerr << "Could not load file: " + cascade_path << endl;
		exit(1);
	}
	// Calc min size
	float min_size_factor = options.param.face_detect.min_size_factor;
	int min_size_w = (int)(src.size().width * min_size_factor);
	int min_size_h = (int)(src.size().height * min_size_factor);
	// Use square
	int min_size = max(min_size_w, min_size_h);

	// Do face detect and ret
	vector<cv::Rect> faces_out;
	cascade.detectMultiScale(src, faces_out, 1.03, 3, 0, cv::Size(min_size, min_size));

	cout << faces_out.size() << endl;
	for (auto& rect : faces_out)
	{
		cout << rect.x << " " 
			<< rect.y << " " 
			<< rect.width << " " 
			<< rect.height << endl;
	}

	return 0;
}

int handler_CycleConvert(const cv::Mat& src, const Options& options)
{
	CycleClient client;
	string modelName;
	switch (options.param.cycleconvert.model)
	{
	case 0:
		modelName = "GrayFace_V2_model";
		break;
	case 1:
		modelName = "GrayFace_V6_model";
		break;
	default:
		modelName = "GrayFace_V2_model";
		break;
	}
	client.set(src, modelName);
	Mat dst = client.run();

	vector<uchar> out_buf;
	cv::imencode(".bmp", dst, out_buf);
	SET_BINARY_MODE(FILENO(stdout));
	WRITE(FILENO(stdout), &out_buf[0], out_buf.size());
	return 0;
}

int main(int argc, const char** argv)
{
	Options options = parseCheckOptions(argc, argv);

	std::vector<char> stdin_data = stdinReadToEnd();
	if (stdin_data.empty())
	{
		cerr << "Empty input" << std::endl;
		return -1;
	}

	cv::Mat mat(stdin_data);
	cv::Mat image = cv::imdecode(mat, cv::IMREAD_GRAYSCALE);

	if (!image.data)                              // Check for invalid input
	{
		cerr << "Could not decode the image" << std::endl;
		return -1;
	}

	switch (options.cmd)
	{
	case Command::BINARIZATION:
		return handler_ImageBinarization(image, options);
		break;

	case Command::FACE_DETECT:
		return handler_FaceDetect(image, options);
		break;

	case Command::CYCLECONVERT:
		return handler_CycleConvert(image, options);
		break;

	default:
		cerr << "Unhandled command type" << endl;
		return -1;
		break;
	}
}