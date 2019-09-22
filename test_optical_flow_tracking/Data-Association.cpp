#include "Data-Association.h"



Hungarian::Hungarian()
{
}


Hungarian::~Hungarian()
{
}

double Hungarian::Solve(vector<vector<double>>& DistMatrix, vector<int>& Assignment)
{
	Enter(DistMatrix, Assignment);
	Init();
	doHungarian();
	//update assignment
	for (int x = 0; x < m; x++)
	{
		Assignment.push_back(matchX[x]);
	}
	for (int x = 0; x < m; x++)
	{
		for (int y = 0; y < n; y++)
		{

			DistMatrix[x][y] = c[x][y];
			if (c[x][y] == maxC)
				DistMatrix[x][y] = 0.f;
		}
	}
	float cost = 0.f;
	for (int x = 0; x < m; x++)
	{
		int y = matchX[x];
		if (c[x][y] < maxC)
		{
			cost += c[x][y];
		}
	}
	return cost;
}

void Hungarian::Enter(vector<vector<double>>& DistMatrix, vector<int>& Assignment)
{
	int i, j;
	m = DistMatrix.size();
	n = DistMatrix[0].size();

	k = m > n ? m : n;

	for (i = 0; i < k; i++)
	{
		for (j = 0; j < k; j++)
		{
			if (i >= m || j >= n)
			{
				c[i][j] = maxC;
				continue;
			}
			if (DistMatrix[i][j] == 0)
				c[i][j] = maxC;
			else
				c[i][j] = DistMatrix[i][j];
		}
	}
}

void Hungarian::Init()
{
	int i, j;

	for (i = 0; i < MAX; i++)
	{
		matchX[i] = -1;
		matchY[i] = -1;
	}

	for (i = 0; i < k; i++)
	{
		Fx[i] = maxC;
		for (j = 0; j < k; j++)
		{
			if (c[i][j] < Fx[i])
			{
				Fx[i] = c[i][j];
			}
		}
	}

	for (j = 0; j < k; j++)
	{
		Fy[j] = maxC;
		for (i = 0; i < k; i++)
		{
			if (c[i][j] - Fx[i] < Fy[j])
			{
				Fy[j] = c[i][j] - Fx[i];
			}
		}
	}
}

double Hungarian::GetC(int i, int j)
{
	return c[i][j] - Fx[i] - Fy[j];
}

void Hungarian::FindAugmentingPath()
{
	queue<int> q;
	int i, j, first, last;

	for (i = 0; i < MAX;i++)
	{
		Trace[i] = -1;
	}

	q.push(start);
	first = 0;
	last = 0;
	do
	{
		i = q.front();
		q.pop();
		for (j = 0; j < k; j++)
		{
			if (Trace[j] == -1 && GetC(i, j) == 0.0f)
			{
				Trace[j] = i;
				if (matchY[j] == -1)
				{
					finish = j;
					return;
				}
				q.push(matchY[j]);
			}
		}
	} while (!q.empty());
}

void Hungarian::SubX_AddY()
{
	int i, j, t;
	double Delta;
	set<int> VisitedX, VisitedY;

	VisitedX.insert(start);
	for (j = 0; j < k; j++)
	{
		if (Trace[j] != -1)
		{
			VisitedX.insert(matchY[j]);
			VisitedY.insert(j);
		}
	}

	Delta = maxC;
	for (i = 0; i < k; i++)
	{
		if (VisitedX.find(i) != VisitedX.end())
		{
			for (j = 0; j < k; j++)
			{
				if ((VisitedY.find(j) == VisitedY.end()) && (GetC(i, j) < Delta))
					Delta = GetC(i, j);
			}
		}
	}
	
	for (t = 0; t < k; t++)
	{
		
		if (VisitedX.find(t) != VisitedX.end())
			Fx[t] = Fx[t] + Delta;
		
		if (VisitedY.find(t) != VisitedY.end())
			Fy[t] = Fy[t] - Delta;
	}
}

void Hungarian::Enlarge()
{
	int x, next;
	do
	{
		x = Trace[finish];
		next = matchX[x];
		matchX[x] = finish;
		matchY[finish] = x;
		finish = next;
	} while (finish != -1);
}

void Hungarian::doHungarian()
{
	int x, y;
	for (x = 0; x < k; x++)
	{

		start = x;
		finish = -1;
		do
		{
			FindAugmentingPath(); 
			if (finish == -1) 
				SubX_AddY();
		} while (finish == -1);
		Enlarge();
	}
}