#include "LinearCurveEvaluator.h"
#include <assert.h>


// Static Function Prototype
static void evaluate_line(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap);

static void evaluate_bezier(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap);

static void evaluate_bspline(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap);

static void evaluate_catmullrom(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap);

static void draw_point(const Point& pt_1, std::vector<Point>& ptvEvaluatedCurvePts);

static void draw_point(const Point& pt_1, std::vector<Point>& ptvEvaluatedCurvePts, float wrap_x);

static void draw_line(
	const Point& pt_1, const Point& pt_2,
	std::vector<Point>& ptvEvaluatedCurvePts);

static void draw_line(
	const Point& pt_1, const Point& pt_2,
	std::vector<Point>& ptvEvaluatedCurvePts, float wrap_x);

static void draw_bezier(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth);

static void draw_bezier(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth, float wrap_x);

static void draw_bspline(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth);

static void draw_bspline(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth, float wrap_x);

static void draw_catmullrom(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth);

static void draw_catmullrom(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth, float wrap_x);

static bool check_flat_enough(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4);

static void subdivide(
	const Point& src_1, const Point& src_2, const Point& src_3, const Point& src_4,
	Point& dst_l_1, Point& dst_l_2, Point& dst_l_3, Point& dst_l_4,
	Point& dst_r_1, Point& dst_r_2, Point& dst_r_3, Point& dst_r_4);

static void midpoint(Point &dst, const Point& src_1, const Point& src_2);


// Operation Handling
void LinearCurveEvaluator::evaluateCurve(
	const std::vector<Point>& ptvCtrlPts, 
	std::vector<Point>& ptvEvaluatedCurvePts, 
	const float& fAniLength, 
	const bool& bWrap) const {

	evaluate_line(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
}


void BezierCurveEvaluator::evaluateCurve(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {

	if (bWrap && ptvCtrlPts.size() < 3) {
		evaluate_line(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
		return;
	}
	else if (!bWrap && ptvCtrlPts.size() < 4) {
		evaluate_line(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
		return;
	}

	ptvEvaluatedCurvePts.clear();
	evaluate_bezier(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
}


void BSplineCurveEvaluator::evaluateCurve(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {

	ptvEvaluatedCurvePts.clear();
	evaluate_bspline(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
}


void CatmullRomCurveEvaluator::evaluateCurve(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const {

	ptvEvaluatedCurvePts.clear();
	evaluate_catmullrom(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
}


// Static Function Implementation
static void evaluate_line(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) {

	ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());

	float	x = 0.0;
	float	y1;
	int		iCtrlPtCount = ptvCtrlPts.size();

	// start point
	// if wrapping is on, interpolate the y value at xmin and
	// xmax so that the slopes of the lines adjacent to the
	// wraparound are equal.
	if (bWrap) {

		if ((ptvCtrlPts[0].x + fAniLength) - ptvCtrlPts[iCtrlPtCount - 1].x > 0.0f) {
			y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x) +
				ptvCtrlPts[iCtrlPtCount - 1].y * ptvCtrlPts[0].x) /
				(ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x);
		}
		else {
			y1 = ptvCtrlPts[0].y;
		}

	}
	// if wrapping is off, make the first and last segments of
	// the curve horizontal.
	else {
		y1 = ptvCtrlPts[0].y;
	}

	ptvEvaluatedCurvePts.push_back(Point(x, y1));

	// end point
	float y2;
	x = fAniLength;
	if (bWrap)	y2 = y1;
	else		y2 = ptvCtrlPts[iCtrlPtCount - 1].y;

	ptvEvaluatedCurvePts.push_back(Point(x, y2));

}

static void evaluate_bezier(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) {

	// wrap
	if (bWrap) {

		// middle points
		int i;
		for (i = 0; i + 3 < ptvCtrlPts.size(); i += 3) {
			draw_bezier(
				ptvCtrlPts[i + 0], ptvCtrlPts[i + 1], ptvCtrlPts[i + 2], ptvCtrlPts[i + 3],
				ptvEvaluatedCurvePts, 10);
		}

		// start point and end point
		if (ptvCtrlPts.size() - i < 3) {
			for (; i < ptvCtrlPts.size() - 1; i++) draw_line(ptvCtrlPts[i], ptvCtrlPts[i + 1], ptvEvaluatedCurvePts, fAniLength);
			draw_line(ptvCtrlPts[i], Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y), ptvEvaluatedCurvePts, fAniLength);
		}
		else {
			draw_bezier(
				ptvCtrlPts[i + 0], ptvCtrlPts[i + 1], 
				ptvCtrlPts[i + 2], Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y),
				ptvEvaluatedCurvePts, 10, fAniLength);
		}

	}

	// no wrap
	else {

		// start point
		ptvEvaluatedCurvePts.push_back(
			Point(0, ptvCtrlPts[0].y));

		// middle points
		int i;
		for (i = 0; i + 3 < ptvCtrlPts.size(); i += 3) {
			draw_bezier(
				ptvCtrlPts[i + 0], ptvCtrlPts[i + 1], ptvCtrlPts[i + 2], ptvCtrlPts[i + 3], 
				ptvEvaluatedCurvePts, 10);
		}

		for (; i < ptvCtrlPts.size(); i++) ptvEvaluatedCurvePts.push_back(Point(ptvCtrlPts[i]));

		// end point
		ptvEvaluatedCurvePts.push_back(
			Point(fAniLength, ptvCtrlPts[ptvCtrlPts.size() - 1].y));

	}
}

static void evaluate_bspline(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) {

	// variable
	const int pt_size = ptvCtrlPts.size();
	Point pt_temp_1, pt_temp_2, pt_temp_3, pt_temp_4;

	// wrap
	if (bWrap) {

		if (pt_size < 3) {
			pt_temp_1 = Point(ptvCtrlPts[0].x - fAniLength, ptvCtrlPts[0].y);
			pt_temp_2 = Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y);
			pt_temp_3 = Point(ptvCtrlPts[pt_size - 1].x - fAniLength, ptvCtrlPts[pt_size - 1].y);
			pt_temp_4 = Point(ptvCtrlPts[pt_size - 1].x + fAniLength, ptvCtrlPts[pt_size - 1].y);
		}
		else {
			pt_temp_1 = Point(ptvCtrlPts[pt_size - 3].x - fAniLength, ptvCtrlPts[pt_size - 3].y);
			pt_temp_2 = ptvCtrlPts[2];
			pt_temp_3 = Point(ptvCtrlPts[pt_size - 3]);
			pt_temp_4 = Point(ptvCtrlPts[2].x + fAniLength, ptvCtrlPts[2].y);
		}

		draw_bspline(
			pt_temp_1, 
			Point(ptvCtrlPts[pt_size - 2].x - fAniLength, ptvCtrlPts[pt_size - 2].y), 
			Point(ptvCtrlPts[pt_size - 1].x - fAniLength, ptvCtrlPts[pt_size - 1].y), 
			ptvCtrlPts[0], 
			ptvEvaluatedCurvePts, 10);
		
		draw_bspline(
			Point(ptvCtrlPts[pt_size - 2].x - fAniLength, ptvCtrlPts[pt_size - 2].y),
			Point(ptvCtrlPts[pt_size - 1].x - fAniLength, ptvCtrlPts[pt_size - 1].y), 
			ptvCtrlPts[0], 
			ptvCtrlPts[1], 
			ptvEvaluatedCurvePts, 10);

		draw_bspline(
			Point(ptvCtrlPts[pt_size - 1].x - fAniLength, ptvCtrlPts[pt_size - 1].y),
			ptvCtrlPts[0], 
			ptvCtrlPts[1], 
			pt_temp_2,
			ptvEvaluatedCurvePts, 10);

		int i;
		for (i = 0; i < (int)(ptvCtrlPts.size()) - 3; i++) {
			draw_bspline(
				ptvCtrlPts[i + 0], ptvCtrlPts[i + 1], ptvCtrlPts[i + 2], ptvCtrlPts[i + 3],
				ptvEvaluatedCurvePts, 10);
		}

		draw_bspline(
			pt_temp_3,
			ptvCtrlPts[pt_size - 2],
			ptvCtrlPts[pt_size - 1],
			Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y),
			ptvEvaluatedCurvePts, 10);

		draw_bspline(
			ptvCtrlPts[pt_size - 2],
			ptvCtrlPts[pt_size - 1],
			Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y),
			Point(ptvCtrlPts[1].x + fAniLength, ptvCtrlPts[1].y),
			ptvEvaluatedCurvePts, 10);

		draw_bspline(
			ptvCtrlPts[pt_size - 1],
			Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y),
			Point(ptvCtrlPts[1].x + fAniLength, ptvCtrlPts[1].y),
			pt_temp_4,
			ptvEvaluatedCurvePts, 10);

	}
	else {

		// start point
		draw_point(Point(0, ptvCtrlPts[0].y), ptvEvaluatedCurvePts);

		// start curve
		if (ptvCtrlPts.size() < 3)	draw_bspline(Point(ptvCtrlPts[ptvCtrlPts.size() - 1].x - fAniLength, ptvCtrlPts[ptvCtrlPts.size() - 1].y), ptvCtrlPts[0], ptvCtrlPts[1], Point(fAniLength, ptvCtrlPts[0].y), ptvEvaluatedCurvePts, 10);
		else						draw_bspline(Point(ptvCtrlPts[ptvCtrlPts.size() - 1].x - fAniLength, ptvCtrlPts[ptvCtrlPts.size() - 1].y), ptvCtrlPts[0], ptvCtrlPts[1], ptvCtrlPts[2], ptvEvaluatedCurvePts, 10);

		// middle curve
		int i;
		for (i = 0; i < (int)(ptvCtrlPts.size()) - 3; i++) {
			draw_bspline(
				ptvCtrlPts[i + 0], ptvCtrlPts[i + 1], ptvCtrlPts[i + 2], ptvCtrlPts[i + 3],
				ptvEvaluatedCurvePts, 10);
		}

		// end curve
		if (ptvCtrlPts.size() >= 3)	draw_bspline(ptvCtrlPts[ptvCtrlPts.size() - 3], 
												 ptvCtrlPts[ptvCtrlPts.size() - 2], 
												 ptvCtrlPts[ptvCtrlPts.size() - 1],
												 Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y),
												 ptvEvaluatedCurvePts, 10);

		// end point
		draw_point(Point(fAniLength, ptvCtrlPts[ptvCtrlPts.size() - 1].y), ptvEvaluatedCurvePts);
	}
}

static void evaluate_catmullrom(
	const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) {

	// variable
	const int pt_size = ptvCtrlPts.size();
	Point pt_temp_1, pt_temp_2, pt_temp_3, pt_temp_4;

	// wrap
	if (bWrap) {

		if (pt_size < 3) {
			pt_temp_1 = Point(ptvCtrlPts[0].x - fAniLength, ptvCtrlPts[0].y);
			pt_temp_2 = Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y);
			pt_temp_3 = Point(ptvCtrlPts[pt_size - 1].x - fAniLength, ptvCtrlPts[pt_size - 1].y);
			pt_temp_4 = Point(ptvCtrlPts[pt_size - 1].x + fAniLength, ptvCtrlPts[pt_size - 1].y);
		}
		else {
			pt_temp_1 = Point(ptvCtrlPts[pt_size - 3].x - fAniLength, ptvCtrlPts[pt_size - 3].y);
			pt_temp_2 = ptvCtrlPts[2];
			pt_temp_3 = Point(ptvCtrlPts[pt_size - 3]);
			pt_temp_4 = Point(ptvCtrlPts[2].x + fAniLength, ptvCtrlPts[2].y);
		}

		draw_catmullrom(
			pt_temp_1,
			Point(ptvCtrlPts[pt_size - 2].x - fAniLength, ptvCtrlPts[pt_size - 2].y),
			Point(ptvCtrlPts[pt_size - 1].x - fAniLength, ptvCtrlPts[pt_size - 1].y),
			ptvCtrlPts[0],
			ptvEvaluatedCurvePts, 10);

		draw_catmullrom(
			Point(ptvCtrlPts[pt_size - 2].x - fAniLength, ptvCtrlPts[pt_size - 2].y),
			Point(ptvCtrlPts[pt_size - 1].x - fAniLength, ptvCtrlPts[pt_size - 1].y),
			ptvCtrlPts[0],
			ptvCtrlPts[1],
			ptvEvaluatedCurvePts, 10);

		draw_catmullrom(
			Point(ptvCtrlPts[pt_size - 1].x - fAniLength, ptvCtrlPts[pt_size - 1].y),
			ptvCtrlPts[0],
			ptvCtrlPts[1],
			pt_temp_2,
			ptvEvaluatedCurvePts, 10);

		int i;
		for (i = 0; i < (int)(ptvCtrlPts.size()) - 3; i++) {
			draw_catmullrom(
				ptvCtrlPts[i + 0], ptvCtrlPts[i + 1], ptvCtrlPts[i + 2], ptvCtrlPts[i + 3],
				ptvEvaluatedCurvePts, 10);
		}

		draw_catmullrom(
			pt_temp_3,
			ptvCtrlPts[pt_size - 2],
			ptvCtrlPts[pt_size - 1],
			Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y),
			ptvEvaluatedCurvePts, 10);

		draw_catmullrom(
			ptvCtrlPts[pt_size - 2],
			ptvCtrlPts[pt_size - 1],
			Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y),
			Point(ptvCtrlPts[1].x + fAniLength, ptvCtrlPts[1].y),
			ptvEvaluatedCurvePts, 10);

		draw_catmullrom(
			ptvCtrlPts[pt_size - 1],
			Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y),
			Point(ptvCtrlPts[1].x + fAniLength, ptvCtrlPts[1].y),
			pt_temp_4,
			ptvEvaluatedCurvePts, 10);

	}
	else {

		// start point
		draw_point(Point(0, ptvCtrlPts[0].y), ptvEvaluatedCurvePts);

		// start curve
		if (ptvCtrlPts.size() < 3)	draw_catmullrom(Point(ptvCtrlPts[ptvCtrlPts.size() - 1].x - fAniLength, ptvCtrlPts[ptvCtrlPts.size() - 1].y), ptvCtrlPts[0], ptvCtrlPts[1], Point(fAniLength, ptvCtrlPts[0].y), ptvEvaluatedCurvePts, 10);
		else						draw_catmullrom(Point(ptvCtrlPts[ptvCtrlPts.size() - 1].x - fAniLength, ptvCtrlPts[ptvCtrlPts.size() - 1].y), ptvCtrlPts[0], ptvCtrlPts[1], ptvCtrlPts[2], ptvEvaluatedCurvePts, 10);

		// middle curve
		int i;
		for (i = 0; i < (int)(ptvCtrlPts.size()) - 3; i++) {
			draw_catmullrom(
				ptvCtrlPts[i + 0], ptvCtrlPts[i + 1], ptvCtrlPts[i + 2], ptvCtrlPts[i + 3],
				ptvEvaluatedCurvePts, 10);
		}

		// end curve
		if (ptvCtrlPts.size() >= 3)	draw_catmullrom(ptvCtrlPts[ptvCtrlPts.size() - 3],
			ptvCtrlPts[ptvCtrlPts.size() - 2],
			ptvCtrlPts[ptvCtrlPts.size() - 1],
			Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y),
			ptvEvaluatedCurvePts, 10);

		// end point
		draw_point(Point(fAniLength, ptvCtrlPts[ptvCtrlPts.size() - 1].y), ptvEvaluatedCurvePts);
	}

}


static void draw_bezier(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth) {

	draw_bezier(pt_1, pt_2, pt_3, pt_4, ptvEvaluatedCurvePts, depth, -1);
}



static void draw_point(const Point& pt_1, std::vector<Point>& ptvEvaluatedCurvePts) {
	draw_point(pt_1, ptvEvaluatedCurvePts, -1);
}



static void draw_point(const Point& pt_1, std::vector<Point>& ptvEvaluatedCurvePts, float wrap_x) {
	if (wrap_x < 0) {
		ptvEvaluatedCurvePts.push_back(Point(pt_1));
	}
	else {
		if (pt_1.x >= wrap_x) {
			ptvEvaluatedCurvePts.push_back(Point(pt_1));
			ptvEvaluatedCurvePts.push_back(Point(pt_1.x - wrap_x, pt_1.y));
		}
		else {
			ptvEvaluatedCurvePts.push_back(Point(pt_1));
		}
	}
}


static void draw_line(
	const Point& pt_1, const Point& pt_2,
	std::vector<Point>& ptvEvaluatedCurvePts) {

	draw_line(pt_1, pt_2, ptvEvaluatedCurvePts, -1);
}


static void draw_line(
	const Point& pt_1, const Point& pt_2,
	std::vector<Point>& ptvEvaluatedCurvePts, float wrap_x) {

	if (wrap_x < 0) {
		ptvEvaluatedCurvePts.push_back(Point(pt_1));
		ptvEvaluatedCurvePts.push_back(Point(pt_2));
	}
	else {
		if (pt_1.x < wrap_x && pt_2.x > wrap_x) {
			const float ratio = (wrap_x - pt_1.x) / (pt_2.x - pt_1.x);
			Point boundary = Point(wrap_x, pt_1.y + (pt_2.y - pt_1.y) * ratio);
			ptvEvaluatedCurvePts.push_back(Point(pt_1));
			draw_point(boundary, ptvEvaluatedCurvePts, wrap_x);
			ptvEvaluatedCurvePts.push_back(Point(pt_2.x - wrap_x, pt_2.y));
		}
		else if (pt_1.x > wrap_x) {
			ptvEvaluatedCurvePts.push_back(Point(pt_1.x - wrap_x, pt_1.y));
			ptvEvaluatedCurvePts.push_back(Point(pt_2.x - wrap_x, pt_2.y));
		}
		else {
			ptvEvaluatedCurvePts.push_back(Point(pt_1));
			ptvEvaluatedCurvePts.push_back(Point(pt_2));
		}
	}

}


static void draw_bezier(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth, float wrap_x) {

	if (depth == 0 || check_flat_enough(pt_1, pt_2, pt_3, pt_4)) {
		draw_line(pt_1, pt_4, ptvEvaluatedCurvePts, wrap_x);
		return;
	}

	Point dst_l[4], dst_r[4];
	subdivide(
		pt_1, pt_2, pt_3, pt_4,
		dst_l[0], dst_l[1], dst_l[2], dst_l[3],
		dst_r[0], dst_r[1], dst_r[2], dst_r[3]);

	draw_bezier(dst_l[0], dst_l[1], dst_l[2], dst_l[3], ptvEvaluatedCurvePts, depth - 1, wrap_x);
	draw_bezier(dst_r[0], dst_r[1], dst_r[2], dst_r[3], ptvEvaluatedCurvePts, depth - 1, wrap_x);
}


static void draw_bspline(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth) {

	draw_bspline(pt_1, pt_2, pt_3, pt_4, ptvEvaluatedCurvePts, depth, -1);
}


static void draw_bspline(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth, float wrap_x) {

	Point pt_temp_1 = pt_1 * 1 + pt_2 * 4 + pt_3 * 1;
	Point pt_temp_2 = pt_2 * 4 + pt_3 * 2;
	Point pt_temp_3 = pt_2 * 2 + pt_3 * 4;
	Point pt_temp_4 = pt_2 * 1 + pt_3 * 4 + pt_4 * 1;

	pt_temp_1 /= 6;
	pt_temp_2 /= 6;
	pt_temp_3 /= 6;
	pt_temp_4 /= 6;

	draw_bezier(pt_temp_1, pt_temp_2, pt_temp_3, pt_temp_4, ptvEvaluatedCurvePts, depth, wrap_x);
}



static void draw_catmullrom(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth) {

	draw_catmullrom(pt_1, pt_2, pt_3, pt_4, ptvEvaluatedCurvePts, depth, -1);
}



static void draw_catmullrom(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4,
	std::vector<Point>& ptvEvaluatedCurvePts, int depth, float wrap_x) {

	draw_point(pt_2, ptvEvaluatedCurvePts);

	Point pt_tangent_1 = Point(pt_3.x - pt_1.x, pt_3.y - pt_1.y);
	Point pt_tangent_2 = Point(pt_4.x - pt_2.x, pt_4.y - pt_2.y);

	pt_tangent_1 /= 6;
	pt_tangent_2 /= 6;

	draw_bezier(
		pt_2, 
		Point(pt_2.x + pt_tangent_1.x, pt_2.y + pt_tangent_1.y), 
		Point(pt_3.x - pt_tangent_2.x, pt_3.y - pt_tangent_2.y), 
		pt_3, ptvEvaluatedCurvePts, depth, wrap_x);

	draw_point(pt_3, ptvEvaluatedCurvePts);
}


static bool check_flat_enough(
	const Point& pt_1, const Point& pt_2, const Point& pt_3, const Point& pt_4) {
	
	const float threshold = 0.00001;
	const float length_actual = abs(pt_1.distance(pt_2)) + abs(pt_2.distance(pt_3)) + abs(pt_3.distance(pt_4));
	const float length_ideal = abs(pt_1.distance(pt_4));

	if (length_ideal == 0) return true;
	return ((length_actual / length_ideal) < 1 + threshold) ? true : false;
}


static void subdivide(
	const Point& src_1, const Point& src_2, const Point& src_3, const Point& src_4,
	Point& dst_l_1, Point& dst_l_2, Point& dst_l_3, Point& dst_l_4,
	Point& dst_r_1, Point& dst_r_2, Point& dst_r_3, Point& dst_r_4) {

	Point temp_1;

	midpoint(temp_1, src_2, src_3);

	dst_l_1.setPoint(src_1);
	midpoint(dst_l_2, src_1, src_2);
	midpoint(dst_l_3, dst_l_2, temp_1);

	dst_r_4.setPoint(src_4);
	midpoint(dst_r_3, src_3, src_4);
	midpoint(dst_r_2, temp_1, dst_r_3);

	midpoint(dst_l_4, dst_l_3, dst_r_2);
	dst_r_1.setPoint(dst_l_4);
}


static void midpoint(Point& dst, const Point& src_1, const Point& src_2) {
	dst.x = (src_1.x + src_2.x) / 2;
	dst.y = (src_1.y + src_2.y) / 2;
}
