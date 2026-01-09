// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>

#include <Functions4U/EnableWarnings.h>

namespace Upp {
using namespace Eigen;

bool PointInSegment(const Pointf &p, const Pointf &from, const Pointf &to) {
	double dpa = Distance(p, from);
	double dpb = Distance(p, to);
	double dab = Distance(from, to);
	
	return abs(dpa + dpb - dab) < EPS_LEN;
}

ContainsPointRes ContainsPoint(const Vector<Pointf>& polygon, const Pointf &pt) {
	if (PointInPoly(polygon, pt))
		return POLY_SECT;
	Array<Pointf> poly;
	Copy(polygon, poly);
	return static_cast<ContainsPointRes>(ContainsPoint(poly, pt));
}

bool PointInPoly(const UVector<Pointf> &xy, const Pointf &pxy) {
	for (int i = 0; i < xy.size()-1; ++i) {
		if (PointInSegment(pxy, xy[i], xy[i+1]))
			return true;
	}
	return false;
}

bool IsClockwise(const UVector<Pointf> &p) {
	int n = p.size();
    double area = 0;

	if (Last(p) == First(p))
		--n;
	ASSERT(n > 2);
	
	for (int i = 0; i < n; i++) 
		area += (p[i]%p[(i + 1)%n]);
    
	return area < 0;
}

double Area(const UVector<Pointf> &p) {
	int n = p.size();
    double area = 0;

	if (Last(p) == First(p))
		--n;
	ASSERT(n > 2);
	
    for (int i = 0; i < n; i++) 
        area += (p[i]%p[(i + 1) % n]);
  
    return 0.5*abs(area);
}

Pointf Centroid(const UVector<Pointf> &p) {
	int n = p.size();
    Pointf ret(0, 0);
	
	for (int i = 0; i < n; ++i) 
		ret += p[i];
	ret /= n;
	
	return ret;
}

double Angle(const Pointf &a, const Pointf &b) {
	double val = (a.x*b.x + a.y*b.y)/Length(a)/Length(b);
	if (val <= -1)
		return M_PI;
	else if (val >= 1)
		return 0;
	return acos(val);
}

double AngleNormal(const Pointf &a, const Pointf &b) {
	double val = (a.x*b.x + a.y*b.y);
	if (val <= -1)
		return M_PI;
	else if (val >= 1)
		return 0;
	return acos(val);
}

bool IsRectangle(const UVector<Pointf> &p) {
	int n = p.size();

	if (n == 0)
		return false;
    
	if (Last(p) == First(p))
		--n;
	
	if (n != 4)
		return false;

	Pointf v01 = p[1] - p[0],
	 	   v12 = p[2] - p[1],
	 	   v23 = p[3] - p[2],
	 	   v30 = p[0] - p[3];
	double l01 = Length(v01),
		   l12 = Length(v12),
		   l23 = Length(v23),
		   l30 = Length(v30);
		
	if (abs(l01 - l23) > EPS_LEN)	// Sides same size
		return false;
	if (abs(l12 - l30) > EPS_LEN)
		return false;
	
	if (abs(Angle(v01, v12) - M_PI/2) > EPS_LEN)		// 90 deg
		 return false;
	if (abs(Angle(v12, v23) - M_PI/2) > EPS_LEN)
		 return false;
	if (abs(Angle(v23, v30) - M_PI/2) > EPS_LEN)
		 return false;	
	
	return true;	
}

void Range(const UVector<Pointf> &p, double &minx, double &maxx, double &miny, double &maxy) {
	int n = p.size();
	
	const Pointf &first = First(p);
	minx = maxx = first.x;
	miny = maxy = first.y;
	
	for (int i = 1; i < n; ++i) {
		const Pointf &p_i = p[i];	
		if (p_i.x < minx)
			minx = p_i.x;
		else if (p_i.x > maxx)
			maxx = p_i.x;
		if (p_i.y < miny)
			miny = p_i.y;
		else if (p_i.y > maxy)
			maxy = p_i.y;
	}
}

double DistanceToLine(const Pointf &p, const Pointf &a, const Pointf &b) {
	double numerator = abs((b.x - a.x) * (a.y - p.y) - (a.x - p.x) * (b.y - a.y));
    double denominator = sqrt(sqr(b.x - a.x) + sqr(b.y - a.y));

	if (abs(numerator) < 1E-8 || abs(denominator) < 1E-8)
		return 0;

    return numerator/denominator;
}

}