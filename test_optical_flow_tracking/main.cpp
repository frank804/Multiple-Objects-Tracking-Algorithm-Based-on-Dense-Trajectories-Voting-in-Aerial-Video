#include"common.h"
#include "voting.h"


void draw_boxes(Mat &mat_img, vector<bbox_t> result_vec) {

	int font_face = FONT_HERSHEY_SIMPLEX;
	double font_scale = 0.8;

	int i_num = 0;
	for (auto &i : result_vec) {
		Point point(i.x, i.y);
		rectangle(mat_img, Rect(i.x, i.y, i.w, i.h), Scalar(0, 255, 255), 4);//BGR
	}
}

void write_detectResult(const string& file, const vector<bbox_t>&  detect_result, unsigned int frame_id)
{
	ofstream outfile(file, ios_base::app);
	if (!outfile.is_open())
	{
		std::cout << "can not open detect result" << std::endl;
		return;
	}
	outfile << frame_id << " ";
	for (auto bbox : detect_result)
	{
		outfile << bbox.x << " " << bbox.y << " " << bbox.w << " " << bbox.h << " ";
	}
	outfile << std::endl;
	outfile.close();
	return;
}

bool  read_detectResult(const string& file, vector<bbox_t>&  detect_result, unsigned int frame_id)
{
	ifstream infile(file);
	if (!infile.is_open())
	{
		std::cout << "can not find detect result" << std::endl;
		return false;
	}

	string line;

	while (getline(infile, line))
	{
		std::stringstream ss(line);
		unsigned int x;
		ss >> x;
		if (x != frame_id)
			continue;
		std::cout << "Frame ID :" << x << std::endl;
		bbox_t  box;
		int i = 0;
		while (ss >> x)
		{
			switch (i % 4)
			{
			case 0:
			{
				box.x = x;
				break;
			}
			case 1:
			{
				box.y = x ;
				break;
			}
			case 2:
			{
				box.w = x ;
				break;
			}
			case 3:
			{
				box.h = x ;
				detect_result.push_back(box);
				break;
			}
			}
			++i;
		}
		std::cout << "detect_result size " << detect_result.size() << std::endl;
		infile.close();
		return true;
	}

	infile.close();
	return false;
}

int  Video(string path, string TxtPath) {
	cv::VideoCapture capture;
	Mat frame;
	frame = capture.open(path);
	if (!capture.isOpened())
	{
		printf("can not open ...\n");
		return -1;
	}
	cv::Size sWH = cv::Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH),(int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
	cout <<"Frame size: " <<sWH.width << " " << sWH.height << endl;
	//if (sWH.width != 1920 || sWH.height != 1080) {	
	//}
	bool isFirstFrame = 1;
	cv::Mat preFrame, curFrame, showframe;
	vector<bbox_t> preB, curB;
	Vote track;
	cv::namedWindow("trajectory", 0);
	unsigned int framenumber = 0;
	while (capture.read(frame))
	{
		++framenumber;
		if (framenumber % 1 != 0)
			continue;
		//实际计算光流应减小分辨率提高速度
		resize(frame, showframe, Size(frame.cols / 2, frame.rows / 2));
		if (isFirstFrame) {
			//在原始分辨率（1920*1080）做检测，获取目标位置信息保存在txt文件里被程序读取
			vector<bbox_t> preB;
			if (!read_detectResult(TxtPath, preB, framenumber))
				break;
			//目标位置按比例变化到resize后的图像尺寸
			for (auto &i:preB) {
				i.h = int(i.h / 2);
				i.w = int(i.w / 2);
				i.x = int(i.x / 2);
				i.y = int(i.y / 2);
			}

			preFrame = showframe.clone();		
			isFirstFrame = 0;
			track.firstFrame(preFrame, preB);
			continue;
		}
		else
		{
			//在原始分辨率（1920*1080）做检测做检测，获取目标位置信息保存在txt文件里被程序读取
			vector<bbox_t> curB;
			if (!read_detectResult(TxtPath, curB, framenumber))
				break;
			//目标位置按比例变化到resize后的图像尺寸
			for (auto &i : curB) {
				i.h = int(i.h / 2);
				i.w = int(i.w / 2);
				i.x = int(i.x / 2);
				i.y = int(i.y / 2);
			}		

			curFrame = showframe.clone();			
			clock_t start, finish;
			double totaltime;
			start = clock();
			track.vote(curFrame, curB);
			track.assignment(curFrame, curB);
			track.showTrajectory(showframe);
			finish = clock();
			totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
			cout << "\n The time consumption：" << totaltime << " s！" << endl;
			resize(showframe, frame, Size(frame.cols, frame.rows));
			cv::imshow("trajectory", frame);
	     	cv::waitKey(1);
			preFrame = curFrame.clone();
			preB = curB;
		}
	}
	capture.release();
	cv::destroyAllWindows();
	return 0;
}


int main()
{
	string VedioPath = "../video/scene4.MP4";       //Test video  https://pan.baidu.com/s/1wZUUkpHGpBks6QYFKUVSFg?fid=1052882489402014
	string TxtPath = "../video/detect_result.txt";  //Target location obtained in advance
	Video(VedioPath, TxtPath);
	system("pause");
	return 0;
}