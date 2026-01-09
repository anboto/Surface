// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors
#include <Core/Core.h>
#include "Surface.h"

namespace Upp {

// Extracts true corner points from a dense perimeter by removing collinear points
UVector<Pointf> ExtractCorners(const UVector<Pointf>& perimeter, double tol) {
	if(perimeter.GetCount() < 3)
		return clone(perimeter);

	UVector<Pointf> cleaned = clone(perimeter);

	// Remove closing point if polygon is closed
	if(Distance(perimeter[0], perimeter.Top()) < tol)
		cleaned.Drop();

	int n = cleaned.GetCount();
	if(n < 3)
		return cleaned;

	// Find first non-collinear point as start
	int startIdx = 0;
	for(int i = 0; i < n; i++) {
		if(!Collinear(cleaned[(i - 1 + n) % n], cleaned[i], cleaned[(i + 1) % n], tol)) {
			startIdx = i;
			break;
		}
	}

	UVector<Pointf> corners;
	corners.Add(cleaned[startIdx]);

	// Walk perimeter and collect corner points
	int i = (startIdx + 1) % n;
	while(i != startIdx) {
		const Pointf& prev = corners.Top();
		const Pointf& curr = cleaned[i];
		const Pointf& next = cleaned[(i + 1) % n];

		if(!Collinear(prev, curr, next, tol))
			corners.Add(curr);

		i = (i + 1) % n;
	}

	// Remove duplicates
	UVector<Pointf> unique;
	for(const Pointf& p : corners) {
		bool duplicate = false;
		for(const Pointf& up : unique) {
			if(Distance(p, up) < tol) {
				duplicate = true;
				break;
			}
		}
		if(!duplicate)
			unique.Add(p);
	}
	return unique;
}

// Validates if exactly 4 points form a rectangle
bool ValidateRectangle(const UVector<Pointf>& corners, double tol) {
	ASSERT(corners.size() == 4);

	UVector<Pointf> sides;
	for(int i = 0; i < 4; i++)
		sides.Add(corners[(i + 1) % 4] - corners[i]);

	// Check perpendicular adjacent sides
	for(int i = 0; i < 4; i++) {
		double len1 = Length(sides[i]);
		double len2 = Length(sides[(i + 1) % 4]);

		if(len1 < tol || len2 < tol)
			return false; // Degenerate

		double dot = Dot(sides[i], sides[(i + 1) % 4]);
		double cosAngle = fabs(dot) / (len1 * len2);

		if(cosAngle > tol)
			return false; // Not perpendicular
	}

	// Check opposite sides equal length
	if(fabs(Length(sides[0]) - Length(sides[2])) > tol ||
	   fabs(Length(sides[1]) - Length(sides[3])) > tol)
		return false;

	// Check equal diagonals (most reliable test)
	double diag1 = Distance(corners[0], corners[2]);
	double diag2 = Distance(corners[1], corners[3]);
	if(fabs(diag1 - diag2) > tol)
		return false;

	// Check diagonal bisection
	Pointf mid1 = (corners[0] + corners[2]) / 2;
	Pointf mid2 = (corners[1] + corners[3]) / 2;
	if(Distance(mid1, mid2) > tol)
		return false;

	return true;
}

UVector<Pointf> IsRectangle(const UVector<Pointf>& perimeter, double tol) {
	if(perimeter.size() < 4)
		return UVector<Pointf>();

	UVector<Pointf> corners = ExtractCorners(perimeter, tol);

	if(corners.size() != 4 || !ValidateRectangle(corners, tol))
		return UVector<Pointf>();

	return corners;
}

bool IsClockwise(const Pointf& a, const Pointf& b, const Pointf& c) {
	return Orientation(a, b, c) < 0;
}

bool IsCounterClockwise(const Pointf& a, const Pointf& b, const Pointf& c) {
	return Orientation(a, b, c) > 0;
}

UVector<Pointf> ConvexContour(const UVector<Pointf>& points) {
	UVector<Pointf> a = clone(points);

	if(a.size() <= 1)
		return a;

	Sort(a, [](const Pointf& a, const Pointf& b) { return a.x == b.x ? a.y < b.y : a.x < b.x; });

	Pointf p1 = a[0], p2 = a.Top();

	UVector<Pointf> up, down;
	up << p1;
	down << p1;

	for(int i = 1; i < a.size(); i++) {
		if(i == a.size() - 1 || IsClockwise(p1, a[i], p2)) {
			while(up.size() >= 2 && !IsClockwise(up[up.size() - 2], up[up.size() - 1], a[i]))
				up.Remove(up.size() - 1);
			up << a[i];
		}
		if(i == a.size() - 1 || IsCounterClockwise(p1, a[i], p2)) {
			while(down.size() >= 2 && !IsCounterClockwise(down[1], down[0], a[i]))
				down.Remove(0);
			down.Insert(0, a[i]);
		}
	}
	up.Remove(up.size() - 1);
	down.Remove(down.size() - 1);

	up.Append(down);

	return up;
}

Vector<Pointf> OffsetContourBevel(const Vector<Pointf>& contour, double d) {
	int n = contour.size();
	ASSERT(n >= 3);

	Vector<Pointf> out;
	out.Reserve(2 * n);

	for(int i = 0; i < n; i++) {
		const Pointf& p0 = contour[i];
		const Pointf& p1 = contour[(i + 1) % n];

		Pointf edge = Normalize(p1 - p0);
		Pointf normal = Pointf(-edge.y, edge.x);

		out.Add(p0 + normal * d); // Offset both endpoints of the edge
		out.Add(p1 + normal * d);
	}
	return out;
}

Vector<Pointf> OffsetContourRoundArcSteps(const Vector<Pointf>& contour, double d, int arc_steps) {
	int n = contour.size();
	ASSERT(n > 2);
	ASSERT(arc_steps > 0);

	Vector<Pointf> out;

	for(int i = 0; i < n; i++) {
		const Pointf& p_prev = contour[(i - 1 + n) % n];
		const Pointf& p = contour[i];
		const Pointf& p_next = contour[(i + 1) % n];

		Pointf e1 = Normalize(p - p_prev);
		Pointf e2 = Normalize(p_next - p);

		Pointf n1 = Pointf(-e1.y, e1.x);
		Pointf n2 = Pointf(-e2.y, e2.x);

		double a1 = atan2(n1.y, n1.x);
		double a2 = atan2(n2.y, n2.x);

		if(a2 > a1)
			a2 -= 2 * M_PI;

		double angle = a1 - a2;

		for(int k = 0; k <= arc_steps; k++) {
			double t = (double)k / arc_steps;
			double a = a1 - t * angle;
			out.Add(p + Pointf(cos(a), sin(a)) * d);
		}
	}
	return out;
}

Vector<Pointf> OffsetContourRoundAngle(const Vector<Pointf>& contour, double d, double deltaangle) {
	int n = contour.size();
	ASSERT(n >= 3);
	ASSERT(deltaangle > 0);

	Vector<Pointf> out;

	for(int i = 0; i < n; i++) {
		const Pointf& p_prev = contour[(i - 1 + n) % n];
		const Pointf& p = contour[i];
		const Pointf& p_next = contour[(i + 1) % n];

		Pointf e1 = Normalize(p - p_prev);
		Pointf e2 = Normalize(p_next - p);

		Pointf n1 = Pointf(-e1.y, e1.x);
		Pointf n2 = Pointf(-e2.y, e2.x);

		double a1 = atan2(n1.y, n1.x);
		double a2 = atan2(n2.y, n2.x);

		if(a2 > a1)
			a2 -= 2 * M_PI;

		double angle = a1 - a2;
		int arc_steps = int(angle / deltaangle);

		if(arc_steps > 0) {
			for(int k = 0; k <= arc_steps; k++) {
				double t = (double)k / arc_steps;
				double a = a1 - t * angle;
				out.Add(p + Pointf(cos(a), sin(a)) * d);
			}
		}
	}
	return out;
}

double DeltaLenToDeltaAngle(double d, double deltaLen) {
	ASSERT(deltaLen > 0);
	return asin(deltaLen / 2 / d);
}

UVector<Pointf> RemoveCollinear(const UVector<Pointf>& pts, bool isclosed, double distanceCollinear) {
	int n = pts.size();
	if(n < 3)
		return clone(pts);

	Vector<Pointf> out;
	out.Reserve(n);

	int ref = 0;
	out << pts[0];
	if(!isclosed) {
		for(int i = 1; i < n - 1; ++i) {
			if(!Collinear(pts[ref], pts[i], pts[i + 1], distanceCollinear)) {
				out << pts[i];
				ref = i;
			}
		}
		out << pts.Top();
	} else {
		int i = 1;
		while(i < n + ref) {
			int curr = i % n;
			int next = (i + 1) % n;

			if(!Collinear(pts[ref], pts[curr], pts[next], distanceCollinear)) {
				out << pts[curr];
				ref = curr;
			}
			++i;
		}
		// Remove duplicate closure if produced
		if(out.size() > 1 && out[0] == out.Top())
			out.Drop();
	}
	return out;
}

Vector<Pointf> RemoveClosest(const Vector<Pointf>& pts, bool isclosed, double minDist) {
    int n = pts.size();
    if(n == 0)
        return clone(pts);

    Vector<Pointf> out;
    out.Reserve(n);

    double minDist2 = sqr(minDist);

	int ref = 0;
    out << pts[0];
    
    if(!isclosed) {
        for(int i = 1; i < n; ++i) {
            if(SquaredDistance(pts[ref], pts[i]) >= minDist2) {
                out << pts[i];
                ref = i;
            }
        }
    } else {
        int i = 1;
        while(i < n + ref) {
            int curr = i % n;

            if(SquaredDistance(pts[ref], pts[curr]) >= minDist2) {
                out << pts[curr];
                ref = curr;
            }
            ++i;
        }
    }
   	// Remove duplicate closure if produced
    if(out.size() > 1 && SquaredDistance(out[0], out.Top()) < minDist2)
    	out.Drop();
    
    return out;
}

Vector<Pointf> BreakLongSides(const Vector<Pointf>& pts, bool isclosed, double targetLen) {
	Vector<Pointf> ret = clone(pts);
	
    int n = pts.GetCount();
    int seg_count = isclosed ? n : (n - 1);
    if (seg_count < 1)
        return ret;


	Vector<double> lens(seg_count);
	for (int i = 0; i < seg_count; ++i) {
        int i0 = i;
        int i1 = (i + 1) % n; // wraps for the closing segment if isclosed
        lens[i] = Distance(ret[i0], ret[i1]);
    }

	for (int i = seg_count - 1; i >= 0; --i) {		// Breaks long 
		double rat = lens[i]/targetLen;	
		if (rat > 1.2) {
			int num = int(round(rat));
			if (num >= 2) {
				int i0 = i;
	            int i1 = (i + 1) % n;
	
	            double x0 = ret[i0].x;
	            double y0 = ret[i0].y;
	            double lenx = ret[i1].x - ret[i0].x;
	            double leny = ret[i1].y - ret[i0].y;
	
	            int insert_pos;
	            if (isclosed && i == n - 1)
	                insert_pos = ret.size(); // append in order along last->first
	            else
	                insert_pos = i0 + 1;
	
	            for (int in = num - 1; in >= 1; --in) {
	                Pointf p(x0 + lenx*(double)in/(double)num,
	                         y0 + leny*(double)in/(double)num);
	                ret.Insert(insert_pos, p);
				}
			}
		}
	}
	return ret;
}

Pointf Rotate(const Pointf& point, double angle, const Pointf& center) {
    Pointf ret;
    
    double cosA = cos(angle);
    double sinA = sin(angle);

    double translatedX = point.x - center.x;
    double translatedY = point.y - center.y;
    double rotatedX = translatedX * cosA - translatedY * sinA;
    double rotatedY = translatedX * sinA + translatedY * cosA;
    ret.x = rotatedX + center.x;
    ret.y = rotatedY + center.y;

    return ret;
}

Vector<Pointf> Rotate(const Vector<Pointf>& points, double angle, const Pointf& center) {
	Vector<Pointf> ret(points.size());
	
    double cosA = cos(angle);
    double sinA = sin(angle);

    for (int i = 0; i < points.size(); ++i) {
        double translatedX = points[i].x - center.x;
        double translatedY = points[i].y - center.y;
        double rotatedX = translatedX * cosA - translatedY * sinA;
        double rotatedY = translatedX * sinA + translatedY * cosA;
        ret[i].x = rotatedX + center.x;
        ret[i].y = rotatedY + center.y;
    }
    return ret;
}

bool IsSimilar(const Pointf &a, const Pointf &b, double similThres) {
	if (IsNull(a) && IsNull(b))
		return true;
	if (abs(a.x - b.x) < similThres && abs(a.y - b.y) < similThres)
		return true;
	return false;
}

}
