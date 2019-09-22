#include "voting.h"


void Vote::firstFrame(const cv::Mat& firFrame, const vector<bbox_t> &curB)
{
	m_traId = 0;
	m_flowCache.resize(5);

	for (size_t i = 0; i < 1000; i++)
	{
		m_color.push_back(icvprGetRandomColor());
	}

	m_frameCache.push_back(firFrame);
	for (size_t i = 0; i < curB.size(); ++i)
	{
		idBbox box(0, curB[i]);
		m_preBbox.push_back(box);

		vector<idBbox> tra;
		tra.push_back(box);
		Trajectory trajectory(m_traId++, tra);
		m_Trajectory.push_back(trajectory);
	}
}

Vote::~Vote()
{

}


void Vote::vote(const cv::Mat& curF, const vector<bbox_t>& curB)
{
	for (auto preBbox:m_preBbox)
	{
		int id = preBbox.m_id;
		bbox_t bbox = preBbox.m_bbox;

		vector<int> tempCost;
		if (m_flowCache[id].data == NULL)
		{
			cv::Mat flow = calOptFlow(m_frameCache[id], curF);
			m_flowCache[id] = flow.clone();
		}

		cv::Mat flow = m_flowCache[id].clone();

		for (auto cur:curB)
		{
			int count = 0;

			for (int row = bbox.y;row < bbox.y + bbox.h; ++row)
			{
				for (int col = bbox.x;col < bbox.x + bbox.w; ++col)
				{
					if (row >= 0 && row < flow.rows && col >= 0 && col < flow.cols)
					{

						Point2f delta = flow.at<Point2f>(row, col);

						if (!isFlowCorrect(delta)) continue;

						if (cur.x < cvRound(col + delta.x) && cvRound(col + delta.x) < (cur.x + cur.w)
							&& cur.y < cvRound(row + delta.y) && cvRound(row + delta.y) < (cur.y + cur.h))
						{
							count++;
						}
					
					}

				}
			}
			tempCost.push_back(count);
		}
		m_cost.push_back(tempCost);
	}
 	return;
}

void Vote::assignment(const cv::Mat & curF,const vector<bbox_t>& curB)
{
	for (auto vec : m_cost)
	{
		int max = 200;
		int task = -1;
		for (size_t i = 0;i < vec.size();++i)
		{
			if (vec[i] > max)
			{
				task = i;
				max = vec[i];
			}
		}
		m_assign.push_back(task);
	}
			
	for (size_t i = 0;i < m_assign.size();++i)
	{
		for (size_t j = i + 1; j < m_assign.size(); ++j)
		{
			if (m_assign[i] == m_assign[j] && m_assign[i] != -1)
			{
				if (m_cost[i][m_assign[j]] >= m_cost[j][m_assign[j]])
				{
					m_assign[j] = -1;
				}
				else
				{
					m_assign[i] = -1;
				}
			}
		}
	}

#ifdef DEBUG_ASSIGN
	for (size_t i = 0; i < m_assign.size(); i++)
	{
		std::cout << m_assign[i] << " ";
	}
	std::cout <<std::endl;
#endif // !1

	m_preBbox.clear();
	vector<int> deleteTra;

	bool flag_full;
	if (m_frameCache.size() == 5) 
	{
		flag_full = true;
		m_frameCache.erase(m_frameCache.begin());
		m_frameCache.push_back(curF);
	}
	else
	{
		flag_full = false;
		m_frameCache.push_back(curF);
	}

	for (size_t i = 0; i < m_Trajectory.size(); ++i)
	{
		if (flag_full)
		{
			if (m_assign[i] != -1)
			{
				idBbox tempidBbox(4, curB[m_assign[i]]);
				m_preBbox.push_back(tempidBbox);
				m_Trajectory[i].m_trajectory.emplace_back(tempidBbox);
			}
			else
			{
				int last = m_Trajectory[i].m_trajectory.size() - 1;

				if (!m_Trajectory[i].m_trajectory[last].m_real)
				{
					--last;
				}

				if (m_Trajectory[i].m_trajectory[last].m_id == 0)
				{
					deleteTra.push_back(i);
					continue;
				}

				--m_Trajectory[i].m_trajectory[last].m_id;

				idBbox idbbox = m_Trajectory[i].m_trajectory.back();
				m_preBbox.push_back(idbbox);
				//根据光流预测检测框的位置  1 出界 2 未出界
				int x, y, w, h;

				x = idbbox.m_bbox.x;
				y = idbbox.m_bbox.y;
				w = idbbox.m_bbox.w;
				h = idbbox.m_bbox.h;

				cv::Point center((x + w / 2), (y + h / 2));
				vector<cv::Point> collection = { cv::Point(center.x,center.y - 1),cv::Point(center.x - 1,center.y) ,
					cv::Point(center.x,center.y) ,cv::Point(center.x + 1,center.y),cv::Point(center.x,center.y + 1) };

				cv::Mat flow = m_flowCache[m_Trajectory[i].m_trajectory[last].m_id + 1];
				int count = 0;
				float deltaU = 0, deltaV = 0;
				for (auto point : collection)
				{
					if (point.x >= 0 && point.x < flow.cols && point.y >= 0 && point.y < flow.rows)
					{
						if (isFlowCorrect(point))
						{
							deltaU += flow.at<cv::Point2f>(point.y, point.x).x;
							deltaV += flow.at<cv::Point2f>(point.y, point.x).y;
							++count;
						}
					}
				}

				deltaU = cvRound(deltaU / count);
				deltaV = cvRound(deltaV / count);

				if (fabs(deltaU) < 1 || fabs(deltaV) < 1)
				{
					deleteTra.push_back(i);
					m_preBbox.pop_back();
					continue;
				}

				idBbox	estidBbox;
				estidBbox.m_real = false;
				estidBbox.m_id = 4;

				if ((center.x + deltaU) > 20 && (center.y + deltaV) > 20 && (center.x + deltaU) < (flow.cols - 20)
					&& (center.y + deltaV) < (flow.rows - 20))
				{
					if ((center.x + deltaU - w / 2) >= 0 && (center.x + deltaU + w / 2) < flow.cols)
					{
						estidBbox.m_bbox.w = w;
					}
					else
					{
						deleteTra.push_back(i);
						m_preBbox.pop_back();
						continue;
					}

					if ((center.y + deltaV - h / 2) >= 0 && (center.y + deltaV + h / 2) < flow.rows)
					{
						estidBbox.m_bbox.h = h;
					}
					else
					{
						deleteTra.push_back(i);
						m_preBbox.pop_back();
						continue;
					}

					estidBbox.m_bbox.x = cvRound(center.x + deltaU - estidBbox.m_bbox.w / 2);
					estidBbox.m_bbox.y = cvRound(center.y + deltaV - estidBbox.m_bbox.h / 2);

					m_Trajectory[i].m_trajectory.emplace_back(estidBbox);
				}
				else 
				{
					deleteTra.push_back(i);
					m_preBbox.pop_back();
					continue;
				}
			}
		}
		else
		{
			if (m_assign[i] != -1)
			{
				idBbox tempidBbox(m_frameCache.size() -1, curB[m_assign[i]]);
				m_preBbox.push_back(tempidBbox);
				m_Trajectory[i].m_trajectory.emplace_back(tempidBbox);
			}
			else
			{
				idBbox tempidBbox(m_frameCache.size() - 1, m_Trajectory[i].m_trajectory.back().m_bbox);
				m_preBbox.push_back(tempidBbox);
				continue;
			}
		}
	}
	
	std::sort(deleteTra.begin(),deleteTra.end());

	for (int i = deleteTra.size()-1; i >= 0; --i)
	{
		m_Trajectory.erase(m_Trajectory.begin() + deleteTra[i]);
	}

	for (size_t i = 0; i < curB.size(); ++i)
	{
		bool flag = true;
		for (size_t j = 0; j < m_assign.size(); j++)
		{
			if (i == m_assign[j]) 
			{
				flag = false;
				break;
			}
		}
		//添加新轨迹 ,同时更新m_preBBox
		if (flag)
		{
			vector<idBbox> tra;

			idBbox box(m_frameCache.size() - 1, curB[i]);
			m_preBbox.push_back(box);
			tra.push_back(box);

			Trajectory trajectory(m_traId++,tra);
			m_Trajectory.emplace_back(trajectory);
		}
	}


	//清空变量 迎接下一帧
	m_flowCache.clear();
	m_flowCache.resize(8);
	m_assign.clear();
	m_cost.clear();
	return;
}

void Vote::showTrajectory(cv::Mat & curFrame)
{
	int id_trajectory = 0;

	for (auto trajectory : m_Trajectory) 
	{
		if (trajectory.m_trajectory.size() <= 5)
			continue;

		id_trajectory = trajectory.m_id ;
		cv::Scalar color = m_color[id_trajectory % 1000];
		cv::Point2i org,pre, cur;
		int frame = 15;

		int last = trajectory.m_trajectory.size() - 1;

		pre.x = trajectory.m_trajectory[last].m_bbox.x + trajectory.m_trajectory[last].m_bbox.w / 2;
		pre.y = trajectory.m_trajectory[last].m_bbox.y + trajectory.m_trajectory[last].m_bbox.h / 2;

		//if (!trajectory.m_trajectory.back().m_real)
		//{
		//	idBbox  idbbox = trajectory.m_trajectory.back();
		//	cv::Point topLeft(idbbox.m_bbox.x,idbbox.m_bbox.y);
		//	cv::Point bootomRight(idbbox.m_bbox.x+idbbox.m_bbox.w,idbbox.m_bbox.y+ idbbox.m_bbox.h);
		//	cv::rectangle(curFrame, topLeft, bootomRight, cv::Scalar(0, 0, 255), 4, 8, 0);
		//}

		idBbox  idbbox = trajectory.m_trajectory.back();
		cv::Point topLeft(idbbox.m_bbox.x,idbbox.m_bbox.y);
		cv::Point bootomRight(idbbox.m_bbox.x+idbbox.m_bbox.w,idbbox.m_bbox.y+ idbbox.m_bbox.h);
		cv::rectangle(curFrame, topLeft, bootomRight, color, 4, 8, 0);

		std::string  text = to_string(id_trajectory);
		org.x = trajectory.m_trajectory[last].m_bbox.x;
		org.y = trajectory.m_trajectory[last].m_bbox.y + 20;
		cv::putText(curFrame,text, org,cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8, 0);

		//for (int i = last - 1;i >= 0; --i)
		//{
		//	if (!trajectory.m_trajectory[i].m_real)
		//		continue;

		//	cur.x = trajectory.m_trajectory[i].m_bbox.x + trajectory.m_trajectory[i].m_bbox.w / 2;
		//	cur.y = trajectory.m_trajectory[i].m_bbox.y + trajectory.m_trajectory[i].m_bbox.h / 2;

		//	cv::line(curFrame, pre, cur, color,5,8,0);
		//	pre = cur;

		//	if (--frame <= 0) break;
		//}
	}

	return;
}