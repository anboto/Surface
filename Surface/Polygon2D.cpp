// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2024, the Anboto author and contributors
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>

namespace Upp {
using namespace Eigen;


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
        area += (p[i]%p[(i + 1) % n]);
    
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

}