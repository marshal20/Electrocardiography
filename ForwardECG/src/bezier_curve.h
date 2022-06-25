#pragma once
#include <vector>
#include <Eigen/Dense>
#include "math.h"


// Cubic bezier curve: a segment consists of 4 points, 2 of which are shared with the previous and next segment

struct BezierCurve
{
	std::vector<Eigen::Vector3<Real>> points;
	std::vector<Real> segments_duratoins;

	void add_point(const Eigen::Vector3<Real>& new_point);
	void remove_point();
	Real total_duration() const;
	Eigen::Vector3<Real> point_at(const Real t) const;
};


// serializing
bool import_bezier_curve(const std::string& file_name, BezierCurve& curve);
bool export_bezier_curve(const std::string& file_name, const BezierCurve& curve);
