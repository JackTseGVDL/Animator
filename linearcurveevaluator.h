#ifndef INCLUDED_LINEAR_CURVE_EVALUATOR_H
#define INCLUDED_LINEAR_CURVE_EVALUATOR_H


#pragma warning(disable : 4786)  


#include "CurveEvaluator.h"


//using namespace std;


class LinearCurveEvaluator : public CurveEvaluator
{
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
		std::vector<Point>& ptvEvaluatedCurvePts, 
		const float& fAniLength, 
		const bool& bWrap) const;
};


class BezierCurveEvaluator : public CurveEvaluator {
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		const float& fAniLength,
		const bool& bWrap) const;
};


class BSplineCurveEvaluator : public CurveEvaluator {
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		const float& fAniLength,
		const bool& bWrap) const;
};


class CatmullRomCurveEvaluator : public CurveEvaluator {
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		const float& fAniLength,
		const bool& bWrap) const;
};


#endif