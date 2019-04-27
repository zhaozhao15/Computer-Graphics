#pragma once
#ifndef _MLBEZIERH_
#define _MLBEZIERH_
#include "../inc/common.h"
#include <vector>
#include <memory>

#define MOUSE_SELECT_SENSITIVENESS 5e-2
constexpr  uint32_t controlPointsNums = 25;
class mlBezier
{
public:
	mlBezier();
	~mlBezier();


	glm::vec3 mlEvalBezierCurve(const glm::vec3 *P, const float &t);
	glm::vec3 mlEvalBezierPatch(const glm::vec3 *controlPoints, const float &u, const float &v);
	glm::vec3 derivBezier(const glm::vec3 *P, const float &t);
	glm::vec3 dUBezier(const glm::vec3 *controlPoints, const float &u, const float &v);
	glm::vec3 dVBezier(const glm::vec3 *controlPoints, const float &u, const float &v);

	void mlCreateBeizermesh();
	void mlTriangularization();

	int getSelectedControlPointIndice(int posX, int posY);
	void getRayScreenToSpace(int posX, int posY, glm::vec3 &outRay_o, glm::vec3 &outRay_d);
	double distancePointToRay(glm::vec3 ray_o, glm::vec3 ray_d, glm::vec3 p);
	void updateControlPointPosition(int posX, int posY, int selectedInd);
public:

	uint32_t divs;
	int Plength = 4;
	std::vector<glm::vec3> Points;
	std::vector<glm::vec3> NewPoints;
	std::vector<glm::vec3> meshPoints;
	std::vector<glm::vec2> st;

	std::vector<int> indicesofControlpoints;
	std::vector<int> indicesofP;
public:

	glm::vec3 controlPoints[controlPointsNums]
		=
	{
		{ -2.0, -2.0,  1.0 },
		{ -0.5, -2.0,  0.0 },
		{ 0.5, -2.0, -2.0 },
		{ 2.0, -2.0,  2.0 },
		{ 3.0, -2.0,  3.0 },

		{ -2.0, -0.5,  2.0 },
		{ -0.5, -0.5,  1.5 },
		{ 0.5, -0.5,  0.0 },
		{ 2.0, -0.5, -2.0 },
		{ 3.0, -0.5,  -3.0 },

		{ -2.0,  0.5,  2.0 },
		{ -0.5,  0.5,  1.0 },
		{ 0.5,  0.5, -1.0 },
		{ 2.0,  0.5,  1.0 },
		{ 3.0, 0.5, 2.0 },

		{ -2.0,  2.0, -1.0 },
		{ 0.5,  2.0,  0.0 },
		{ 2.0,  2.0, -0.5 },
		{ 0.0, 2.0, 0.5 },
		{ 1.0,  2.0, 1.5 },

		{ -2.0,  2.0, -1.0 },
		{ -0.5,  2.0, -1.0 },
		{ 0.5,  2.0,  0.0 },
		{ 2.0,  2.0, -0.5 },
		{ 3.0,  2.0, -0.5 }
	};
};

mlBezier::mlBezier()
{
}

mlBezier::~mlBezier()
{
}


glm::vec3 mlBezier::mlEvalBezierCurve(const glm::vec3 * P, const float & t)
{
	float b0 = (1 - t) * (1 - t) * (1 - t) * (1 - t);
	float b1 = 4 * t * (1 - t) * (1 - t) * (1 - t);
	float b2 = 6 * t * t * (1 - t) * (1 - t);
	float b3 = 4 * t * t * t * (1 - t);
	float b4 = t * t * t * t;
	return P[0] * b0 + P[1] * b1 + P[2] * b2 + P[3] * b3 + P[4] * b4;
}

glm::vec3 mlBezier::mlEvalBezierPatch
(const glm::vec3 * controlPoints, 
	const float & u, const float & v)
{
	glm::vec3 uCurve[5];
	for (int i = 0; i < 5; i++) {
		glm::vec3 curveP[5];
		curveP[0] = controlPoints[i * 5];
		curveP[1] = controlPoints[i * 5 + 1];
		curveP[2] = controlPoints[i * 5 + 2];
		curveP[3] = controlPoints[i * 5 + 3];
		curveP[4] = controlPoints[i * 5 + 4];
		uCurve[i] = mlEvalBezierCurve(curveP, u);
	}
	return mlEvalBezierCurve(uCurve, v);
}

inline glm::vec3 mlBezier::derivBezier(const glm::vec3 * P, const float & t)
{
	return glm::vec3(0.0);
}

glm::vec3 mlBezier::dUBezier(const glm::vec3 * controlPoints, const float & u, const float & v)
{
	return glm::vec3(0.0);
}

glm::vec3 mlBezier::dVBezier(const glm::vec3 * controlPoints, const float & u, const float & v)
{
	return glm::vec3(0.0);

}

void mlBezier::mlCreateBeizermesh()
{
	for (int u = 0; u < 99; u++) {
		for (int v = 0; v < 99; v++) {
			int ind = u * 100 + v;
			indicesofP.push_back(ind);
			indicesofP.push_back(ind + 1);
			indicesofP.push_back(ind + 101);
			indicesofP.push_back(ind + 100);
		}
	}
}

void mlBezier::mlTriangularization()
{
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 5; i++)
		{
			int ind = j * 5 + i;
			indicesofControlpoints.push_back(ind);
			indicesofControlpoints.push_back(ind + 1);

			indicesofControlpoints.push_back(ind + 6);
			indicesofControlpoints.push_back(ind + 5);
		}
	}

}




// you will fill the four function to edit control points
int mlBezier::getSelectedControlPointIndice(int posX, int posY)
{
	return 0;
}

void mlBezier::getRayScreenToSpace(int posX, int posY, glm::vec3 &outRay_o, glm::vec3 &outRay_d)
{

}

double mlBezier::distancePointToRay(glm::vec3 ray_o, glm::vec3 ray_d, glm::vec3 p)
{
	return 0;
}

void mlBezier::updateControlPointPosition(int posX, int posY, int selectedInd)
{
}

#endif // !_MLBEZIERH_
