#ifndef POINT_H_INCLUDED
#define POINT_H_INCLUDED

#pragma warning(disable : 4786)

#include <functional>
#include <iostream>
#include <cmath>

//using namespace std;

class Point
{
public:

	Point(void);
	Point(const float& x, const float& y);

	void toStream(std::ostream& output_stream) const;
	void fromStream(std::istream& input_stream);

	float distance(const Point& p) const {
		float xd = x - p.x;
		float yd = y - p.y;
		return sqrtf(xd * xd + yd * yd);
	}

	void setPoint(const Point& p) {
		this->x = p.x;
		this->y = p.y;
	}

	float x;
	float y;

	Point& operator /= (const float val) {
		x /= val;
		y /= val;
		return *this;
	}

	friend Point operator* (const Point& pt, const float val) {
		return Point(pt.x * val, pt.y * val);
	}

	friend Point operator* (const float val, const Point& pt) {
		return Point(pt.x * val, pt.y * val);
	}

	friend Point operator+ (const Point& pt_1, const Point& pt_2) {
		return Point(pt_1.x + pt_2.x, pt_1.y + pt_2.y);
	}

	friend Point operator- (const Point& pt_1, const Point& pt_2) {
		return Point(pt_1.x - pt_2.x, pt_1.y - pt_2.y);
	}

};


std::ostream& operator<<(std::ostream& output_stream, const Point& point);
std::istream& operator>>(std::istream& input_stream, Point& point);


class PointSmallerXCompare : public std::binary_function<const Point&, const Point&, bool>
{
public:
	bool operator()(const Point& first, const Point& second) const;
};


class PointLargerXCompare : public std::binary_function<const Point&, const Point&, bool>
{
public:
	bool operator()(const Point& first, const Point& second) const;
};


#endif // POINT_H_INCLUDED
