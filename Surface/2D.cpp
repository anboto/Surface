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

}