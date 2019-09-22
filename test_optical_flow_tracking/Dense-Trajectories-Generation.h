#pragma once
#ifndef OPTFLOW_H
#define OPTFLOW_H

#include<vector>
#include <opencv2/opencv.hpp>

#define USE_GPU


using namespace std;
using namespace cv;

struct bbox_t {
	unsigned int x, y, w, h;    // (x,y) - top-left corner, (w, h) - width & height of bounded box
	float prob;                    // confidence - probability that the object was found correctly
	unsigned int obj_id;        // class of object - from range [0, classes-1]
	unsigned int track_id;        // tracking id for video (0 - untracked, 1 - inf - tracked object)
	unsigned int frames_counter;// counter of frames on which the object was detected
};
inline cv::Mat calOptFlow(const cv::Mat &preFrame,const cv::Mat &curFrame)
{
	cv::Mat preFrameGray, curFrameGray, optFlow;

	if (preFrame.data != NULL && curFrame.data != NULL)
	{
		cvtColor(preFrame, preFrameGray, CV_BGR2GRAY);
		cvtColor(curFrame, curFrameGray, CV_BGR2GRAY);

#ifdef USE_GPU
		cuda::GpuMat g_pg(preFrameGray);     //当前帧的灰度GpuMat
		cuda::GpuMat g_ng(curFrameGray);    //下一帧的灰度GpuMat
		cuda::GpuMat optGpu;            //光流GpuMat
		Ptr<cuda::FarnebackOpticalFlow> opt_tool = cuda::FarnebackOpticalFlow::create();
		opt_tool->calc(g_pg, g_ng, optGpu);
		optFlow = Mat(optGpu);
#else
		//CPU 版本
		calcOpticalFlowFarneback(preFrameGray, curFrameGray, optFlow, 0.5, 3, 5, 3, 5, 1.2, 0);
#endif // USE_GPU
	}
	else
	{
		std::cout << "image is empty" << std::endl;
	}

	return optFlow;
}

inline void showOptFlow(cv::Mat&preFrame, vector<bbox_t>& vecBbox,cv::Mat& optFlow)
{
	for (auto bbox : vecBbox)
	{
		for (int col = bbox.x;col < bbox.x + bbox.w; col += 2)
		{
			for (int row = bbox.y;row < bbox.y + bbox.h; row += 2)
			{
				Point2f delta = optFlow.at<Point2f>(row, col);
				line(preFrame, Point(col,row), Point(cvRound(col + delta.x), cvRound(row + delta.y)), Scalar(0, 255, 0));
			}
		}
	}
	cv::namedWindow("opt_flow", 0);
	cv::imshow("opt_flow",preFrame);
	cv::waitKey(3);
	return;
}

#endif // ! OPTFLOW_H
