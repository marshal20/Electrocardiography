#include "bezier_curve.h"
#include <stdio.h>


// t is normalized [0:1]
static Eigen::Vector3<Real> third_order_bezier(const Eigen::Vector3<Real>& p0, const Eigen::Vector3<Real>& p1, const Eigen::Vector3<Real>& p2, const Eigen::Vector3<Real>& p3, const Real t)
{
	if (t < 0)
	{
		return p0;
	}
	else if (t > 1)
	{
		return p3;
	}

	// intermediate point (stage 1)
	Eigen::Vector3<Real> q0 = (1-t)*p0 + t*p1;
	Eigen::Vector3<Real> q1 = (1-t)*p1 + t*p2;
	Eigen::Vector3<Real> q2 = (1-t)*p2 + t*p3;

	// intermediate point (stage 2)
	Eigen::Vector3<Real> r0 = (1-t)*q0 + t*q1;
	Eigen::Vector3<Real> r1 = (1-t)*q1 + t*q2;

	// final stage (stage 3)
	return (1-t)*r0 + t*r1;
}

void BezierCurve::add_point(const Eigen::Vector3<Real>& new_point)
{
	points.push_back(new_point);

	// add segment duration if the new point created a new segment
	if ((points.size()-1)%3 == 0)
	{
		segments_duratoins.push_back(1);
	}
}

void BezierCurve::remove_point()
{
	if (points.size() > 0)
	{
		// remove segment duration
		if ((points.size()-1)%3 == 0)
		{
			segments_duratoins.erase(segments_duratoins.begin()+segments_duratoins.size()-1);
		}

		// remove point
		points.erase(points.begin()+points.size()-1);
	}
}

Real BezierCurve::total_duration() const
{
	Real total = 0;
	for (Real t : segments_duratoins)
	{
		total += t;
	}
	return total;
}

Eigen::Vector3<Real> BezierCurve::point_at(const Real t) const
{
	// handle out of duration time value
	if (t > total_duration() && segments_duratoins.size() > 0)
	{
		return points[segments_duratoins.size()*4];
	}
	else if (t < 0 || t > total_duration())
	{
		if (points.size() > 0)
		{
			return points[0];
		}
		else
		{
			return { 0, 0, 0 };
		}
	}

	int segment_idx = 0;
	Real t_begin = 0;
	for (const Real seg_duration : segments_duratoins)
	{
		if (t >= t_begin && t <= t_begin+seg_duration)
		{
			break;
		}
		t_begin += seg_duration;
		segment_idx++;
	}

	return third_order_bezier(points[segment_idx*3 + 0],
							  points[segment_idx*3 + 1],
							  points[segment_idx*3 + 2],
							  points[segment_idx*3 + 3],
							  (t-t_begin)/segments_duratoins[segment_idx]);
}




struct PointSerialized
{
	double x, y, z;
};

static bool import_bezier_curve(const std::string& file_name, BezierCurve& curve)
{
	FILE* file = fopen(file_name.c_str(), "rb");
	if (!file)
	{
		return false;
	}

	BezierCurve new_curve;

	// read points count
	int points_count = 0;
	fread(&points_count, sizeof(points_count), 1, file);
	int durations_count = 0;
	fread(&durations_count, sizeof(durations_count), 1, file);

	// read points
	PointSerialized serialized_point;
	new_curve.points.reserve(points_count);
	for (int i = 0; i < points_count; i++)
	{
		fread(&serialized_point, sizeof(serialized_point), 1, file);
		new_curve.points.push_back({ serialized_point.x, serialized_point.y, serialized_point.z });
	}

	// read durations
	double temp;
	new_curve.segments_duratoins.reserve(durations_count);
	for (int i = 0; i < durations_count; i++)
	{
		fread(&temp, sizeof(temp), 1, file);
		new_curve.segments_duratoins.push_back(temp);
	}

	// assign imported probes
	curve = new_curve;

	fclose(file);
	return true;
}

static bool export_bezier_curve(const std::string& file_name, const BezierCurve& curve)
{
	FILE* file = fopen(file_name.c_str(), "wb");
	if (!file)
	{
		return false;
	}

	// read points count
	const int points_count = curve.points.size();
	fwrite(&points_count, sizeof(points_count), 1, file);
	const int durations_count = curve.segments_duratoins.size();
	fwrite(&durations_count, sizeof(durations_count), 1, file);

	// write points
	PointSerialized serialized_point;
	for (int i = 0; i < points_count; i++)
	{
		serialized_point = { curve.points[i].x(), curve.points[i].y(), curve.points[i].z() };
		fwrite(&serialized_point, sizeof(serialized_point), 1, file);
	}

	// read durations
	double temp;
	for (int i = 0; i < durations_count; i++)
	{
		temp = curve.segments_duratoins[i];
		fwrite(&temp, sizeof(temp), 1, file);
	}

	fclose(file);
	return true;
}
