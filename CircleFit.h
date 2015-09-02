#pragma once

#include <vector>

struct PointOnCircle{
	double x;
	double y;
	double cita;
	PointOnCircle(double x_, double y_, double cita_) : x(x_), y(y_), cita(cita_){}
};
class CircleFit{
	int fit_circle(std::vector<PointOnCircle> points, double &A, double &B, double &R);
public:
	bool ranSacCircleFit(std::vector<PointOnCircle> points, double &A, double &B, double &R);
};