#pragma once
#ifndef VOTE_H
#define VOTE_H

#include"common.h"
#include"Dense-Trajectories-Generation.h"
//#include "yolo_v2_class.hpp"

struct idBbox 
{
	int m_id;
	bool m_real;
	bbox_t m_bbox;

	idBbox() {}
	idBbox(int id,const bbox_t& bbox,bool real = true) 
	{
		m_id = id;
		m_real = real;
		m_bbox = bbox;
	}
};

struct Trajectory
{
	int m_id;
	vector<idBbox> m_trajectory;

	Trajectory() {}
	Trajectory(int id, const vector<idBbox> & trajectory)
	{
		m_id = id;
		m_trajectory = trajectory;
	}
};

class Vote {
private:
	vector<cv::Mat> m_frameCache;
	vector<Trajectory> m_Trajectory;
	vector<vector<int>> m_cost;
	vector<idBbox> m_preBbox;
	vector<int> m_assign;
	vector<cv::Scalar> m_color;
	vector<cv::Mat>  m_flowCache;
	int m_traId;


	cv::Scalar icvprGetRandomColor()
	{
		uchar r = 255 * (rand() / (1.0 + RAND_MAX));
		uchar g = 255 * (rand() / (1.0 + RAND_MAX));
		uchar b = 255 * (rand() / (1.0 + RAND_MAX));
		return cv::Scalar(b, g, r);
	}
	
	inline bool isFlowCorrect(Point2f u)
	{
		return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.x) < 1e9 && fabs(u.y) < 1e9;
	}
public:
	Vote() {}
	void firstFrame(const cv::Mat& firFrame, const vector<bbox_t> &curB);
	~Vote();
	void vote(const cv::Mat& curF,const vector<bbox_t> &curB);
	void assignment(const cv::Mat& curF,  const vector<bbox_t>& curB);
	void showTrajectory(cv::Mat &curFrame);
};


#endif