// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>


namespace Upp {
using namespace Eigen;

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

UVector<Pointf> SetDirection(const UVector<Pointf> &p, bool clockwise) {
	UVector<Pointf> ret = clone(p);
	if (IsClockwise(p) && clockwise)
		return ret;
	Reverse(ret);
	return ret;
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

	for (int i = 0; i < n; i++) {
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

	for (int i = 0; i < n; i++) {
		const Pointf& p_prev = contour[(i - 1 + n) % n];
		const Pointf& p = contour[i];
		const Pointf& p_next = contour[(i + 1) % n];

		Pointf e1 = Normalize(p - p_prev);
		Pointf e2 = Normalize(p_next - p);

		Pointf n1 = Pointf(-e1.y, e1.x);
		Pointf n2 = Pointf(-e2.y, e2.x);

		double a1 = atan2(n1.y, n1.x);
		double a2 = atan2(n2.y, n2.x);

		if (a2 > a1)
			a2 -= 2 * M_PI;

		double angle = a1 - a2;

		for (int k = 0; k <= arc_steps; k++) {
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

	for (int i = 0; i < n; i++) {
		const Pointf& p_prev = contour[(i - 1 + n) % n];
		const Pointf& p = contour[i];
		const Pointf& p_next = contour[(i + 1) % n];

		Pointf e1 = Normalize(p - p_prev);
		Pointf e2 = Normalize(p_next - p);

		Pointf n1 = Pointf(-e1.y, e1.x);
		Pointf n2 = Pointf(-e2.y, e2.x);

		double a1 = atan2(n1.y, n1.x);
		double a2 = atan2(n2.y, n2.x);

		if (a2 > a1)
			a2 -= 2*M_PI;

		double angle = a1 - a2;
		int arc_steps = int(angle / deltaangle);

		if (arc_steps > 0) {
			for (int k = 0; k <= arc_steps; k++) {
				double t = (double)k / arc_steps;
				double a = a1 - t * angle;
				out.Add(p + Pointf(cos(a), sin(a)) * d);
			}
		}
	}
	return out;
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
			if (!Collinear(pts[ref], pts[i], pts[i + 1], distanceCollinear)) {
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


// ─────────────────────────────────────────────────────────────────────────────
// AddPolygonIntersections
//
// Given two closed polygons 'a' and 'b'
// finds every intersection and inserts the intersection
// points into both polygons at the correct positions along each edge.
//
//   epsilon   — tolerance for floating-point comparisons
// ─────────────────────────────────────────────────────────────────────────────
int AddPolygonIntersections(const Vector<Pointf>& a0, const Vector<Pointf>& b0, Vector<Pointf>& a, Vector<Pointf>& b, double epsilon) {
	a = clone(a0);
	if (CompareDelta(Last(a), First(a), 0.00001))
		a.Remove(a.size()-1);
	b = clone(b0);
	if (CompareDelta(Last(b), First(b), 0.00001))
		b.Remove(b.size()-1);
	
	int na = a.size();
	int nb = b.size();

	if (na < 2 || nb < 2)
		return 0;

	struct IntersectionPt : Moveable<IntersectionPt> {
		Pointf pt;
		int    edgeIndex;   // index of the edge's first vertex in the polygon
		double t;           // parametric position along that edge (0..1)
	};

	// Collect all intersections, keyed to each polygon's edges
	// intersA[i] = list of intersections found on edge i of polygon a
	// intersB[j] = list of intersections found on edge j of polygon b
	Vector<Vector<IntersectionPt>> intersA(na), intersB(nb);

	for (int i = 0; i < na; i++) {
		Pointf ai = a[i],  ai1 = a[(i + 1)%na];

		for (int j = 0; j < nb; j++) {
			Pointf bi = b[j],  bi1 = b[(j + 1)%nb];

			Pointf pts[2];
			double tv[2], uv[2];

			int nPts = SegmentIntersection(ai, ai1, bi, bi1, pts, tv, uv, epsilon);
			if (nPts > 0) {
				for (int k = 0; k < nPts; k++) {
					IntersectionPt ia;
					ia.pt        = pts[k];
					ia.edgeIndex = i;
					ia.t         = tv[k];
					intersA[i]  << ia;
	
					IntersectionPt ib;
					ib.pt        = pts[k];
					ib.edgeIndex = j;
					ib.t         = uv[k];
					intersB[j]  << ib;
				}
			}
		}
	}

	Vector<Pointf> newA, newB;
	newA.Reserve(na + intersA.size());

	for (int i = 0; i < na; i++) {
		if (i == 0 || !CompareDelta(Last(newA), a[i], epsilon))
			newA << a[i];

		Vector<IntersectionPt>& pts = intersA[i];
		if (!pts.IsEmpty()) {
			Sort(pts, [](const IntersectionPt& x, const IntersectionPt& y) {return x.t < y.t;});
	
			for (const IntersectionPt& ip : pts)
				if (!CompareDelta(Last(newA), ip.pt, epsilon))
					newA << ip.pt;
		}
	}
	if (CompareDelta(First(newA), Last(newA), epsilon))
		newA.Remove(newA.size()-1);
	a = pick(newA);

	newB.Reserve(na + intersB.size());
	int ret = 0;
	for (int i = 0; i < nb; i++) {
		if (i == 0 || !CompareDelta(Last(newB), b[i], epsilon))
			newB << b[i];

		Vector<IntersectionPt>& pts = intersB[i];
		if (!pts.IsEmpty()) {
			Sort(pts, [](const IntersectionPt& x, const IntersectionPt& y) {return x.t < y.t;});
	
			for (const IntersectionPt& ip : pts)
				if (!CompareDelta(Last(newB), ip.pt, epsilon)) {
					newB << ip.pt;
					ret++;
				}
		}
	}
	if (CompareDelta(First(newB), Last(newB), epsilon))
		newB.Remove(newB.size()-1);
	b = pick(newB);
	
	return ret;
}
/*
struct Segment2 {
	int idfrom, idto;	
};

// Detect points of crossing lines
static void AddPendingPoints(Vector<Segment2> &seg, Vector<Pointf> &points) {
	
}

static void JoinSegments(Vector<Segment2> &seg, Vector<Pointf> &points, Vector<Segment2> &seg2, Vector<Pointf> &points2) {
	// Joins them
	// Remove too close points
}
	
static void PolyToSegments(const Vector<Pointf> &a, Vector<Segment2> &seg, Vector<Pointf> &points) {
	
}

void Intersection(const Vector<Pointf> &a, const Vector<Pointf> &b, Vector<Vector<Pointf>> &res) {
	res.Clear();
	
	Vector<Segment2> seg, segb;
	Vector<Pointf> points, pointsb;
	
	PolyToSegments(a, seg, points);
	AddPendingPoints(seg, points);
	
	PolyToSegments(b, segb, pointsb);
	AddPendingPoints(segb, pointsb);
	
	JoinSegments(seg, points, segb, pointsb);
	
	struct IsUsedSeg {
		bool leftRight = false, rightLeft = false;	
	};

	Vector<IsUsedSeg> used(seg.size());
	
	// Loops sobre los sgmentos hasta que used no tenga nada en true
	// Los poligonos se crean uno a uno hasta que se cierra o se acaba
	// Se crea un de mas. el exterios, que se ha de quitar
	
}
*/
void ReplaceSharedContiguousAverage(Vector<Pointf> &a, Vector<Pointf> &b, double epsilon) {
	Vector<int> todeletea, todeleteb;
	int nb = b.size();
    for (int i = 0; i < a.size()-1;) {
        int idb, idb1;
        if((idb = FindDelta(b, a[i], epsilon)) < 0) {
            i++;
            continue;
        }
        if((idb1 = FindDelta(b, a[i+1], epsilon)) < 0) {
            i++;
            continue;
        }
        if (!Collinear(a[i], a[i+1], b[(idb-1+nb)%nb], epsilon) && !Collinear(a[i], a[i+1], b[(idb+1)%nb], epsilon)) {
            i++;
            continue;
        }   
        a[i] = b[idb] = Middle(a[i], a[i+1]);
		todeletea << i+1;
		todeleteb << idb1;
		
		i += 2;
    }
    for (int i = todeletea.size()-1; i >= 0; --i)
        a.Remove(todeletea[i]);
    Sort(todeleteb);
    for (int i = todeleteb.size()-1; i >= 0; --i)
        b.Remove(todeleteb[i]);
}
  
void TrimAtIntersection(const Vector<Pointf> &a, const Vector<Pointf> &b, Vector<Pointf> &aret, Vector<Pointf> &bret, double epsilon) {
	Vector<Pointf> a2 = clone(a);
	Vector<Pointf> b2 = clone(b);
	
	a2 = RemoveCollinear(a2, true, epsilon);
	b2 = RemoveCollinear(b2, true, epsilon);
	
	if (0 == AddPolygonIntersections(a2, b2, a2, b2, epsilon))
		return;

	auto ReOrder = [](Vector<Pointf> &a, const Vector<Pointf> &b)->bool {
		int na = a.size();
		int id0 = -1;
		for (int i = 0; i < na; ++i) {
			//Pointf middle = Middle(a[i], a[(i + 1)%na]);
			if (POLY_OUT == ContainsPoint(b, a[i]) && POLY_OUT == ContainsPoint(b, a[i+1])) {
				id0 = i;
				break;
			}
		}
		if (id0 < 0)
			return false;
		
		Rotate(a, id0);
		
		return true;
	};
	if (!ReOrder(a2, b2))
		return;
	if (!ReOrder(b2, a2))
		return;
	
	ReplaceSharedContiguousAverage(a2, b2, epsilon);
	
	if (!ReOrder(a2, b2))
		return;
	if (!ReOrder(b2, a2))
		return;

	auto AddBetween = [](int id0, int id1, Vector<int> &remove) {
		if (id0 > id1)
			Swap(id0, id1);
		for (int i = id0+1; i < id1; ++i)
			remove << i;
	};
	Vector<int> removea, removeb;
	int idstarta = -1, idstartb;
	for (int i = 0; i < a2.size(); ++i) {
		int idb = FindDelta(b2, a2[i], epsilon);
		if (idb < 0)
			continue;
		
		if (idstarta < 0) {
			idstarta = i;
			idstartb = idb;
		} else {
			AddBetween(idstarta, i, removea);
			AddBetween(idstartb, idb, removeb);
			idstarta = -1;
		}
	}
	Sort(removea);
	for (int i = removea.size()-1; i >= 0; --i)
		a2.Remove(removea[i]);
	Sort(removeb);
	for (int i = removeb.size()-1; i >= 0; --i)
		b2.Remove(removeb[i]);
	
	aret = pick(a2);
	bret = pick(b2);
}

}