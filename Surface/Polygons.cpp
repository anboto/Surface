// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2024, the Anboto author and contributors
#include <Core/Core.h>

#include <Eigen/Eigen.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>
#include <Functions4U/Functions4U.h>
#include <numeric> 
#include <STEM4U/Rootfinding.h>

namespace Upp {
using namespace Eigen;


// Valid for concave shapes, no order required
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

// Valid for concave shapes, no order required
ContainsPointRes ContainsPoint(const UVector<Point3D> &_polygon, const Point3D &point, double distanceTol, double angleNormalTol) {
    if (_polygon.size() < 3) 
        return POLY_3;
    
    UVector<Point3D> polygon = clone(_polygon);
    if (First(polygon) != Last(polygon))
        polygon << First(polygon);
    
	UVector<Direction3D> normals;
	Direction3D normal;
	normal.Zero();
	for (int i = 0; i < polygon.size()-2; ++i) {
		if (!Collinear(polygon[i], polygon[i+1], polygon[i+2])) {
			normals << Normal(polygon[i], polygon[i+1], polygon[i+2]);
			normal += normals[i];
		} 
	}
	normal /= normals.size();
	//normal.Normalize();
	
	double maxAngle = 0;
	for (int i = 0; i < normals.size()-1; ++i) {
		double ang = normals[i].Angle(normals[i+1]);
		if (ang > M_PI/2)
			ang = M_PI - ang;
		maxAngle = max(maxAngle, ang); // abs() to handle convex shapes
	}
	if (maxAngle > angleNormalTol)
		return POLY_NOPLAN;
	
	Point3D intersection = Intersection(normal, point, polygon[0], normal);
	double distance = Distance(intersection, point);
	if (distance > distanceTol)
		return POLY_FAR;
	
	UVector<Pointf> xy = Point3Dto2D_XY(polygon);
	UVector<Pointf> xz = Point3Dto2D_XZ(polygon);
	UVector<Pointf> yz = Point3Dto2D_YZ(polygon);
	Pointf pxy = Pointf(point.x, point.y);
	Pointf pxz = Pointf(point.x, point.z);
	Pointf pyz = Pointf(point.y, point.z);
	
	ContainsPointRes res;

	res = ContainsPoint(xy, pxy);
	if (res > 0)
		return POLY_IN;
	else if (res < 0)
		return POLY_OUT;
	
	res = ContainsPoint(xz, pxz);
	if (res > 0)
		return POLY_IN;
	else if (res < 0)
		return POLY_OUT;
	
	res = ContainsPoint(yz, pyz);
	if (res > 0)
		return POLY_IN;
	else if (res < 0)
		return POLY_OUT;
	
	return POLY_SECT;
}

bool IsClockwise(const UVector<Pointf> &p) {
    int n = p.size();
    double area = 0;

    for (int i = 0; i < n; i++) {
        const Pointf &curr = p[i];
        const Pointf &next = p[(i + 1) % n];

        area += (next.x - curr.x) * (next.y + curr.y);
    }
	return area < 0;
}

double Area(const UVector<Pointf> &p) {		// Valid for concave shapes, no order required
	int n = p.size();
    double area = 0.0;

	if (Last(p) == First(p))
		--n;
	ASSERT(n > 2);
	
    for (int i = 0; i < n; i++) 
        area += (p[i]%p[(i + 1) % n]);
  
    return 0.5*abs(area);
}

double Area(const UVector<Point3D> &p) {	// Valid for concave shapes, no order required
	int n = p.size();
    Value3D area;
    area.Zero();

	if (Last(p) == First(p))
		--n;
	ASSERT(n > 2);
	
    for (int i = 0; i < n; i++) 
        area += (p[i]%p[(i + 1) % n]);
    
    return 0.5*area.Length();
}

Pointf Centroid(const UVector<Pointf> &p) {	// Valid for concave shapes, no order required
	int n = p.size();
    Pointf ret(0, 0);
	
	for (int i = 0; i < n; ++i) 
		ret += p[i];
	ret /= n;
	
	return ret;
}

Point3D Centroid(const UVector<Point3D> &p) {	// Valid for concave shapes, no order required
	int n = p.size();
    Point3D ret;
    ret.Zero();
	
	for (int i = 0; i < n; ++i) 
		ret += p[i];
	ret /= n;
	
	return ret;
}

Vector<Pointf> Point3Dto2D_XY(const Vector<Point3D> &bound) {
	Vector<Pointf> ret;
	for (const auto &d: bound)
		ret << Pointf(d.x, d.y);
	return ret;
}

Vector<Pointf> Point3Dto2D_XZ(const Vector<Point3D> &bound) {
	Vector<Pointf> ret;
	for (const auto &d: bound)
		ret << Pointf(d.x, d.z);
	return ret;
}

Vector<Pointf> Point3Dto2D_YZ(const Vector<Point3D> &bound) {
	Vector<Pointf> ret;
	for (const auto &d: bound)
		ret << Pointf(d.y, d.z);
	return ret;
}

Vector<Point3D> Point2Dto3D_XY(const Vector<Pointf> &bound) {
	Vector<Point3D> ret;
	for (const auto &d: bound)
		ret << Point3D(d.x, d.y, 0);
	return ret;
}

Vector<Point3D> Point2Dto3D_XZ(const Vector<Pointf> &bound) {
	Vector<Point3D> ret;
	for (const auto &d: bound)
		ret << Point3D(d.x, 0, d.y);
	return ret;
}

Vector<Point3D> Point2Dto3D_YZ(const Vector<Pointf> &bound) {
	Vector<Point3D> ret;
	for (const auto &d: bound)
		ret << Point3D(0, d.x, d.y);
	return ret;
}

}