// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2024, the Anboto author and contributors
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>

namespace Upp {
using namespace Eigen;


ContainsPointRes ContainsPoint(const UVector<Point3D> &_polygon, const Point3D &point, double distanceTol, double angleNormalTol) {
    if (_polygon.size() < 3) 
        return POLY_3;
    
    UVector<Point3D> polygon2;
    if (First(_polygon) != Last(_polygon)) {
        polygon2 = clone(_polygon);
        polygon2 << First(polygon2);
    }
    const UVector<Point3D> &polygon = polygon2.size() > 0 ? polygon2 : _polygon;
    
	UVector<Vector3D> normals;
	Vector3D normal = Vector3D::Zero();
	
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

double Area(const UVector<Point3D> &p) {
	int n = p.size();
	ASSERT(n > 0);
    Value3D area = Value3D::Zero();

	if (Last(p) == First(p))
		--n;
	ASSERT(n > 2);
	
    for (int i = 0; i < n; i++) 
        area += (p[i]%p[(i + 1) % n]);
    
    return 0.5*area.Length();
}

Point3D Centroid(const UVector<Point3D> &p) {
	int n = p.size();
    ASSERT(n > 1);
    Point3D ret = Point3D::Zero();
    
	if (Last(p) == First(p))
		--n;
			
	for (int i = 0; i < n; ++i) 
		ret += p[i];
	ret /= n;
	
	return ret;
}

bool IsRectangle(const UVector<Point3D> &p) {
	int n = p.size();

	if (n == 0)
		return false;
    
	if (Last(p) == First(p))
		--n;
	
	if (n != 4)
		return false;
	
	Value3D v01 = p[1] - p[0],
	 		v12 = p[2] - p[1],
	 		v23 = p[3] - p[2],
	 		v30 = p[0] - p[3];
	
	if (abs(v01.Length() - v23.Length()) > EPS_LEN)	// Sides same size
		return false;
	if (abs(v12.Length() - v30.Length()) > EPS_LEN)
		return false;
	
	if (abs(v01.Angle(v12) - M_PI/2) > EPS_LEN)		// 90 deg
		 return false;
	if (abs(v12.Angle(v23) - M_PI/2) > EPS_LEN)
		 return false;
	if (abs(v23.Angle(v30) - M_PI/2) > EPS_LEN)
		 return false;
	
	if (!IsFlat(p))
		return false;
	
	return true;
}

bool IsFlat(const UVector<Point3D> &p) {
	int n = p.size();

	if (n == 0)
		return false;
    
	if (Last(p) == First(p))
		--n;
	
	if (n > 3)
		return false;
	else if (n == 3)
		return true;
	
	const Point3D &p0 = p[0];
    Value3D vec1 = p[1] - p[0];
    Value3D vec2 = p[2] - p[0];
	Value3D normal = vec1%vec2;
	
	if (normal.Norm() == 0) 		// Collinear
        return false;
    
    normal.Normalize();
    
	Vector<Value3D> vect;
	for (int i = 2; i < n; ++i) {
		vec1 = p[i] - p0;
        vec2 = p[i + 1] - p0;
        Value3D current_normal = vec1%vec2;
        if (current_normal.Norm() != 0) 
            current_normal.Normalize();
        
        if (!normal.IsSimilar(current_normal, EPS_LEN*EPS_LEN)) {
            return false;
        }
	}
	return true;
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