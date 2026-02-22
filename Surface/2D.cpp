// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors
#include <Core/Core.h>
#include "Surface.h"

namespace Upp {

bool PointInSegment(const Pointf &p, const Pointf &from, const Pointf &to) {
	double dpa = Distance(p, from);
	double dpb = Distance(p, to);
	double dab = Distance(from, to);
	
	return abs(dpa + dpb - dab) < EPS_LEN;
}

bool IsClockwise(const Pointf& a, const Pointf& b, const Pointf& c) {
	return Orientation(a, b, c) < 0;
}

bool IsCounterClockwise(const Pointf& a, const Pointf& b, const Pointf& c) {
	return Orientation(a, b, c) > 0;
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

double DeltaLenToDeltaAngle(double d, double deltaLen) {
	ASSERT(deltaLen > 0);
	return asin(deltaLen/2/d);
}

Pointf Rotate(const Pointf& point, double angle, const Pointf& center) {
    Pointf ret;
    
    double cosA = cos(angle);
    double sinA = sin(angle);

    double translatedX = point.x - center.x;
    double translatedY = point.y - center.y;
    double rotatedX = translatedX*cosA - translatedY*sinA;
    double rotatedY = translatedX*sinA + translatedY*cosA;
    ret.x = rotatedX + center.x;
    ret.y = rotatedY + center.y;

    return ret;
}

double DistanceToLine(const Pointf &p, const Pointf &a, const Pointf &b) {
	double numerator = Cross(a, b, p);
    double denominator = Distance(a, b);

	if (abs(numerator) < 1E-8 || abs(denominator) < 1E-8)
		return 0;

    return numerator/denominator;
}


// ─────────────────────────────────────────────────────────────────────────────
// Tests intersection of p1,p2 against p3,p4
// Returns number of intersections
// Output arrays pt[], t[], u[] receive up to 2 results:
//   pt[k]  — the intersection point
//   t[k]   — parametric position along (p1→p2)  in (0,1)
//   u[k]   — parametric position along (p3→p4)  in (0,1)
// epsilon — tolerance used for all floating-point inside tests.
// ─────────────────────────────────────────────────────────────────────────────
int SegmentIntersection(const Pointf &p1, const Pointf &p2, const Pointf &p3, const Pointf &p4,
                              Pointf pt[2], double t[2], double u[2], double epsilon) {
	Pointf d1(p2 - p1);
	Pointf d2(p4 - p3);

	double denom = Cross(d1, d2);

	if (fabs(denom) >= epsilon) {		// Non-parallel case, classic single intersection point
		Pointf d(p3 - p1);

		t[0] = Cross(d, d2)/denom;
		u[0] = Cross(d, d1)/denom;

		if (t[0] < -epsilon || t[0] > 1 + epsilon || u[0] < -epsilon || u[0] > 1 + epsilon)
			return 0;

		pt[0] = p1 + t[0]*d1;
		
		return 1;
	}

	// ── Parallel branches — check for collinearity ───────────────────────────
	// Vector from p1 to p3; if cross-product with d1 is non-zero, lines differ.
	Pointf c(p3 - p1);
	double cross = Cross(c, d1);

	double axisLen34 = Squared(d1);
	if (fabs(cross) > epsilon*sqrt(axisLen34))   // lines are truly parallel
		return 0;

	double epsilon2 = epsilon*epsilon;
	
	// ── Collinear: project everything onto the axis of segment 1 ─────────────
	// Use the longer component as the projection axis for numerical stability.
	// Parametric values t ∈ [0,1] cover (p1→p2); values outside that range
	// mean the point is beyond the segment.

	if (axisLen34 < epsilon2)
		return 0;   // degenerate zero-length segment

	// Project p3 and p4 onto the (p1→p2) axis → gives t-values for those points
	double tP3 = ((p3.x - p1.x)*d1.x + (p3.y - p1.y)*d1.y)/axisLen34;
	double tP4 = ((p4.x - p1.x)*d1.x + (p4.y - p1.y)*d1.y)/axisLen34;

	// Project p1 and p2 onto the (p3→p4) axis → gives u-values for those points
	double axisLen12 = Squared(d2);
	if (axisLen12 < epsilon2)
		return 0;   // degenerate zero-length segment

	double uP1 = ((p1.x - p3.x)*d2.x + (p1.y - p3.y)*d2.y)/axisLen12;
	double uP2 = ((p2.x - p3.x)*d2.x + (p2.y - p3.y)*d2.y)/axisLen12;

	// Collect candidate intersections: endpoint of one segment strictly inside
	// the other segment.  "Strictly inside" means parametric value in (0,1).
	struct Candidate {
		Pointf p; 
		double tVal, uVal;
	};
	Candidate cands[4];

	auto strictlyInside = [&](double v) {return v > epsilon && v < 1 - epsilon;};

	int nc = 0;
	if (strictlyInside(tP3))		// p3 inside (p1-p2)?
		cands[nc++] = {p3, tP3, 0};
	if (strictlyInside(tP4))		// p4 inside (p1-p2)?
		cands[nc++] = {p4, tP4, 1};
	if (strictlyInside(uP1))		// p1 inside (p3-p4)?
		cands[nc++] = {p1, 0, uP1};
	if (strictlyInside(uP2))		// p2 inside (p3-p4)?
		cands[nc++] = {p2, 1, uP2};

	if (nc == 0)
		return 0;

	// Deduplicate: two candidates may describe the same geometric point. Keep unique ones.
	Vector<int> keep;
	keep << 0;
	for (int i = 1; i < nc; i++) {
		bool dup = false;
		for (int k : keep) {
			if (CompareDelta(cands[i].p, cands[k].p, epsilon)) { 
				dup = true; 
				break; 
			}
		}
		if (!dup)
			keep << i;
	}

	// Sort kept candidates by ascending t so output order is consistent
	Sort(keep, [&](int a, int b_) {return cands[a].tVal < cands[b_].tVal;});

	int nOut = min(keep.size(), 2);
	for (int k = 0; k < nOut; k++) {
		const Candidate& c = cands[keep[k]];
		pt[k] = c.p;
		t[k]  = c.tVal;
		u[k]  = c.uVal;
	}

	return nOut;
}

bool CompareDelta(const Pointf &a, const Pointf &b, double similThres) {
	if (IsNull(a) && IsNull(b))
		return true;
	if (abs(a.x - b.x) < similThres && abs(a.y - b.y) < similThres)
		return true;
	return false;
}

bool CompareDelta(const Pointf *a, const Pointf *b, int num, double similThres) {
	for (int i = 0; i < num; ++i) {
		if (!CompareDelta(a[i], b[i], similThres))
			return false;
	}
	return true;
}


}
