// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors   
#include <Core/Core.h>
#include "Surface.h"

namespace Upp {
	
// Extracts true corner points from a dense perimeter by removing collinear points
UVector<Pointf> ExtractCorners(const UVector<Pointf>& perimeter, double tol) {
	if (perimeter.GetCount() < 3)
		return clone(perimeter);
	
	UVector<Pointf> cleaned = clone(perimeter);
	
	// Remove closing point if polygon is closed
	if (Distance(perimeter[0], perimeter.Top()) < tol)
		cleaned.Drop();
	
	int n = cleaned.GetCount();
	if (n < 3)
		return cleaned;
	
	// Find first non-collinear point as start
	int startIdx = 0;
	for (int i = 0; i < n; i++) {
		if (!Collinear(cleaned[(i - 1 + n) % n], cleaned[i], cleaned[(i + 1) % n], tol)) {
			startIdx = i;
			break;
		}
	}
	
	UVector<Pointf> corners;
	corners.Add(cleaned[startIdx]);
	
	// Walk perimeter and collect corner points
	int i = (startIdx + 1) % n;
	while (i != startIdx) {
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
	for (int i = 0; i < 4; i++)
		sides.Add(corners[(i + 1) % 4] - corners[i]);
	
	// Check perpendicular adjacent sides
	for (int i = 0; i < 4; i++) {
		double len1 = Length(sides[i]);
		double len2 = Length(sides[(i + 1) % 4]);
		
		if (len1 < tol || len2 < tol)
			return false; // Degenerate
		
		double dot = Dot(sides[i], sides[(i + 1) % 4]);
		double cosAngle = fabs(dot) / (len1 * len2);
		
		if (cosAngle > tol)
			return false; // Not perpendicular
	}
	
	// Check opposite sides equal length
	if (fabs(Length(sides[0]) - Length(sides[2])) > tol ||
	   fabs(Length(sides[1]) - Length(sides[3])) > tol)
		return false;
	
	// Check equal diagonals (most reliable test)
	double diag1 = Distance(corners[0], corners[2]);
	double diag2 = Distance(corners[1], corners[3]);
	if (fabs(diag1 - diag2) > tol)
		return false;
	
	// Check diagonal bisection
	Pointf mid1 = (corners[0] + corners[2])/2;
	Pointf mid2 = (corners[1] + corners[3])/2;
	if (Distance(mid1, mid2) > tol)
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

static int Orientation(const Pointf &a, const Pointf &b, const Pointf &c, bool include_collinear, double distanceCollinear) {
	if (include_collinear) {
		double ab = Distance(a, b);
		double bc = Distance(b, c);
		double ca = Distance(c, a);
		double mind;
		if (ab > bc) {
			if (ab > ca)
				mind = DistanceToLine(c, a, b);
			else
				mind = DistanceToLine(b, a, c);
		} else
			mind = DistanceToLine(a, b, c);
		if (mind <= distanceCollinear)
			return 0;
	}
    double v = a.x*(b.y-c.y) + b.x*(c.y-a.y) + c.x*(a.y-b.y);
    if (v < 0) 
    	return -1; // clockwise
    if (v > 0) 
    	return +1; // counter-clockwise
    return 0;
}

static bool IsClockwise(const Pointf &a, const Pointf &b, const Pointf &c, bool include_collinear, double distanceCollinear) {
    int o = Orientation(a, b, c, include_collinear, distanceCollinear);
    return o < 0 || (include_collinear && o == 0);
}

static bool IsCounterClockwise(const Pointf &a, const Pointf &b, const Pointf &c, bool include_collinear, double distanceCollinear) {
    int o = Orientation(a, b, c, include_collinear, distanceCollinear);
    return o > 0 || (include_collinear && o == 0);
}

UVector<Pointf> ConvexHull(const UVector<Pointf>& points, bool include_collinear, double distanceCollinear) {
	UVector<Pointf> a = clone(points);
	
    if (a.size() <= 1)
        return a;

    Sort(a, [](const Pointf &a, const Pointf &b) {return a.x == b.x ? a.y < b.y : a.x < b.x;});
    
    Pointf p1 = a[0], p2 = a.back();
    
    UVector<Pointf> up, down;
    up << p1;
    down << p1;
    for (int i = 1; i < a.size(); i++) {
        if (i == a.size() - 1 || IsClockwise(p1, a[i], p2, include_collinear, distanceCollinear)) {
            while (up.size() >= 2 && !IsClockwise(up[up.size()-2], up[up.size()-1], a[i], include_collinear, distanceCollinear))
                up.Remove(up.size()-1);
            up << a[i];
        }
        if (i == a.size() - 1 || IsCounterClockwise(p1, a[i], p2, include_collinear, distanceCollinear)) {
            while (down.size() >= 2 && !IsCounterClockwise(down[down.size()-2], down[down.size()-1], a[i], include_collinear, distanceCollinear))
                down.Remove(down.size()-1);
            down << a[i];
        }
    }

    if (include_collinear && up.size() == a.size()) {
        Upp::Reverse(a);
        return a;
    }
    a = pick(up);
    int ic = a.size();
    a.SetCount(ic + down.size() - 2);
    for (int i = down.size() - 2; i > 0; i--)
        a[ic++] = down[i];
    
    return a;
}

}