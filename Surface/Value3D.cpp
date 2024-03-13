// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2024, the Anboto author and contributors
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>

namespace Upp {
using namespace Eigen;


Value3D Middle(const Value3D &a, const Value3D &b) {
	return Value3D(avg(a.x, b.x), avg(a.y, b.y), avg(a.z, b.z));	
}

Value3D WeightedMean(const Value3D &a, double va, const Value3D &b, double vb) {
	ASSERT (va >= 0 && vb >= 0);
	double t = va + vb;
	return Value3D((a.x*va+b.x*vb)/t, (a.y*va+b.y*vb)/t, (a.z*va+b.z*vb)/t);	
}

Value3D Centroid(const Value3D &a, const Value3D &b, const Value3D &c) {
	return Value3D(avg(a.x, b.x, c.x), avg(a.y, b.y, c.y), avg(a.z, b.z, c.z));	
}

// The normal of three collinear points may be misleading. Check collinearity before.
Direction3D Normal(const Value3D &a, const Value3D &b, const Value3D &c) {
	return Direction3D((a - b) % (b - c)).Normalize();
}

bool Collinear(const Value3D &a, const Value3D &b, const Value3D &c) {
	return Direction3D((a - b) % (b - c)).Length() < 0.0001;
}

bool Collinear(const Pointf &a, const Pointf &b, const Pointf &c) {
	double area = Area(a, b, c);
	if (area < 1e-12)
		return true;
	double areasq = (max(a.x, b.x, c.x) - min(a.x, b.x, c.x))*(max(a.y, b.y, c.y) - min(a.y, b.y, c.y));
	return area/areasq < 0.001;		// To validate the scale
}

/*double Area(const Value3D &p0, const Value3D &p1, const Value3D &p2) {
	double l01 = Distance(p0, p1);
	double l12 = Distance(p1, p2);
	double l02 = Distance(p0, p2);

	double s = (l01 + l12 + l02)/2;
	return sqrt(max(s*(s - l01)*(s - l12)*(s - l02), 0.)); 
}*/

double Area(const Value3D &a, const Value3D &b, const Value3D &c) {
    return abs(0.5*((b - a)%(c - a)).Length());
}

double Area(const Pointf &a, const Pointf &b, const Pointf &c) {
    return abs(0.5*((b - a)%(c - a)));
}

double Direction(const Pointf& a, const Pointf& b) {
	return atan2(b.y - a.y, b.x - a.x);
}


Point3D Intersection(const Direction3D &lineVector, const Point3D &linePoint, const Point3D &planePoint, const Direction3D &planeNormal) {
	Direction3D diff = planePoint - linePoint;
	double prod1 = diff.dot(planeNormal);
	double prod2 = lineVector.dot(planeNormal);
	if (abs(prod2) < EPS_LEN)
		return Null;
	double factor = prod1/prod2;
	return linePoint + lineVector*factor;	
}

double Manhattan(const Value3D &p1, const Value3D &p2) {
	return abs(p1.x-p2.x) + abs(p1.y-p2.y) + abs(p1.z-p2.z);
}

double Length(const Value3D &p1, const Value3D &p2) {
	return ::sqrt(sqr(p1.x-p2.x) + sqr(p1.y-p2.y) + sqr(p1.z-p2.z));
}

double Distance(const Value3D &p1, const Value3D &p2) {
	return Length(p1, p2);
}

void TranslateForce(const Point3D &from, const VectorXd &ffrom, Point3D &to, VectorXd &fto) {
	Direction3D r(from.x - to.x, from.y - to.y, from.z - to.z);
	Direction3D F(ffrom[0], ffrom[1], ffrom[2]);
	Direction3D M = r%F;
	
	M = r%F;
	
	fto = clone(ffrom);
	fto(3) += M.x;
	fto(4) += M.y;
	fto(5) += M.z;
}

void Value3D::Translate(double dx, double dy, double dz) {
	x += dx;
	y += dy;
	z += dz;
}

void Value3D::Rotate(double ax, double ay, double az, double cx, double cy, double cz) {
	Affine3d aff;
	GetTransform(aff, ax, ay, az, cx, cy, cz);
	TransRot(aff);
}

void Value3D::TransRot(double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz) {
	Affine3d aff;
	GetTransform(aff, dx, dy, dz, ax, ay, az, cx, cy, cz);
	TransRot(aff);
}

void Value3D::TransRot(const Affine3d &quat) {
	Vector3d pnt0(x, y, z);	
	Vector3d pnt = quat * pnt0;

	x = pnt[0];
	y = pnt[1];
	z = pnt[2];
}

void TransRot(const Value3D &pos, const Value3D &ref, double x, double y, double z, double rx, double ry, double rz, Value3D &npos) {
	npos = clone(pos);
	npos.TransRot(x, y, z, rx, ry, rz, ref.x, ref.y, ref.z);
}

void TransRot(const Value3D &pos, const Value3D &ref, const VectorXd &transf, Value3D &npos) {
	npos = clone(pos);
	npos.TransRot(transf[0], transf[1], transf[2], transf[3], transf[4], transf[5], ref.x, ref.y, ref.z);
}

void TransRot(const Value3D &pos, const Value3D &ref, VectorXd &x, VectorXd &y, VectorXd &z, VectorXd &rx, VectorXd &ry, VectorXd &rz) {
	ASSERT((x.size() == y.size()) && (y.size() == z.size()) && (z.size() == rx.size()) && (rx.size() == ry.size()) && (ry.size() == rz.size()));
	for (int i = 0; i < x.size(); ++i) {
		Value3D ps = clone(pos);
		ps.TransRot(x[i], y[i], z[i], rx[i], ry[i], rz[i], ref.x, ref.y, ref.z);
		x[i] = ps.x;
		y[i] = ps.y;
		z[i] = ps.z;
	}
}

void TransRot(const Affine3d &aff, const Value3D &pos, Value3D &npos) {
	npos = clone(pos);
	npos.TransRot(aff);
}

bool TransRotChangeRef(const Value3D &ref, const VectorXd &transf, const Value3D &nref, VectorXd &ntransf) {
	int nump = 4;
	
	UArray<Value3D> points(nump), tpoints(nump);
	
	for (int i = 0; i < nump; ++i) {
		points[i][0] = Random(100);
		points[i][1] = Random(100);
		points[i][2] = Random(100);
		TransRot(points[i], ref, transf, tpoints[i]);
	}
	
	ntransf = clone(transf);		// Initial values
	ntransf.array() += 0.1;			// Some noise
	
	if (!NonLinearOptimization(ntransf, 3*nump, [&](const VectorXd &x, VectorXd &res)->int {
		for(int i = 0; i < nump; i += 2) {
			Value3D np0, np1;
			TransRot(points[i],   nref, x, np0);
			TransRot(points[i+1], nref, x, np1);
			res[3*i+0] = tpoints[i].x - np0.x;
			res[3*i+1] = tpoints[i].y - np0.y;
			res[3*i+2] = tpoints[i].z - np0.z;
			res[3*i+3] = tpoints[i+1].x - np1.x;
			res[3*i+4] = tpoints[i+1].y - np1.y;
			res[3*i+5] = tpoints[i+1].z - np1.z;
		}
		return 0;	
	}))
		return false;
	
	return true;
}

void GetTransform(Affine3d &aff, double ax, double ay, double az, double cx, double cy, double cz) {
	Vector3d c(cx, cy, cz);	
	aff = Translation3d(c) *
		  AngleAxisd(ax, Vector3d::UnitX()) *
		  AngleAxisd(ay, Vector3d::UnitY()) *
		  AngleAxisd(az, Vector3d::UnitZ()) *
		  Translation3d(-c);
}

void GetTransform(Affine3d &aff, double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz) {
	Vector3d d(dx, dy, dz), c(cx, cy, cz);	
	aff = Translation3d(d) *
		  Translation3d(c) *
		  AngleAxisd(ax, Vector3d::UnitX()) *
		  AngleAxisd(ay, Vector3d::UnitY()) *
		  AngleAxisd(az, Vector3d::UnitZ()) *
		  Translation3d(-c);
}

void GetTransform000(Affine3d &aff, double dx, double dy, double dz, double ax, double ay, double az) {
	Vector3d d(dx, dy, dz);	
	aff = Translation3d(d) *
		  AngleAxisd(ax, Vector3d::UnitX()) *
		  AngleAxisd(ay, Vector3d::UnitY()) *
		  AngleAxisd(az, Vector3d::UnitZ());
}

Point3D Segment3D::IntersectionPlaneX(double x) {
	if (from.x >= x && to.x >= x)
		return Point3D(true);
	if (from.x <= x && to.x <= x)
		return Point3D(false);
	
	double factor = (x - from.x)/(to.x - from.x);
	return Point3D(x, from.y + (to.y - from.y)*factor, from.z + (to.z - from.z)*factor);
}

Point3D Segment3D::IntersectionPlaneY(double y) {
	if (from.y >= y && to.y >= y)
		return Point3D(true);
	if (from.y <= y && to.y <= y)
		return Point3D(false);
	
	double factor = (y - from.y)/(to.y - from.y);
	return Point3D(from.x + (to.x - from.x)*factor, y, from.z + (to.z - from.z)*factor);
}

Point3D Segment3D::IntersectionPlaneZ(double z) {
	if (from.z >= z && to.z >= z)
		return Point3D(true);
	if (from.z <= z && to.z <= z)
		return Point3D(false);
	
	double factor = (z - from.z)/(to.z - from.z);
	return Point3D(from.x + (to.x - from.x)*factor, from.y + (to.y - from.y)*factor, z);
}

Point3D Segment3D::Intersection(const Point3D &planePoint, const Direction3D &planeNormal) {
	Direction3D vector = Direction();
	Direction3D diff = planePoint - from;
	double prod1 = diff.dot(planeNormal);
	double prod2 = vector.dot(planeNormal);
	if (abs(prod2) < EPS_LEN)
		return Null;
	double factor = prod1/prod2;
	if (factor >= 1)
		return Point3D(true);
	if (factor <= 0)
		return Point3D(false);
	return from + vector*factor;	
}

bool Segment3D::PointIn(const Point3D &p) const {
	return PointInSegment(p, *this);
}

bool Segment3D::SegmentIn(const Segment3D &in) const {
	return SegmentInSegment(in, *this);
}

bool Segment3D::SegmentIn(const Segment3D &in, double in_len) const {
	return SegmentInSegment(in, in_len, *this);
}

bool PointInSegment(const Point3D &p, const Point3D &from, const Point3D &to) {
	double dpa = Distance(p, from);
	double dpb = Distance(p, to);
	double dab = Distance(from, to);
	
	return abs(dpa + dpb - dab) < EPS_LEN;
}

bool PointInSegment(const Point3D &p, const Segment3D &seg) {
	return PointInSegment(p, seg.from, seg.to);
}

bool PointInSegment(const Pointf &p, const Pointf &from, const Pointf &to) {
	double dpa = Distance(p, from);
	double dpb = Distance(p, to);
	double dab = Distance(from, to);
	
	return abs(dpa + dpb - dab) < EPS_LEN;
}

bool SegmentInSegment(const Segment3D &in, double in_len, const Segment3D &seg) {
	double seg_len = seg.Length();
	
	double seg_from_in_from = Distance(seg.from, in.from);
	double in_to_seg_to 	= Distance(in.to, seg.to);
	if (abs(seg_from_in_from + in_len + in_to_seg_to - seg_len) < EPS_LEN)
		return true;

	double seg_from_in_to = Distance(seg.from, in.to);
	double in_from_seg_to = Distance(in.from, seg.to);
	if (abs(seg_from_in_to + in_len + in_from_seg_to - seg_len) < EPS_LEN)
		return true;
	
	return false;
}

bool SegmentInSegment(const Segment3D &in, const Segment3D &seg) {
	return SegmentInSegment(in, in.Length(), seg);
}

}