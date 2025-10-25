// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2022, the Anboto author and contributors
#ifndef _GLCanvas_surface_h_
#define _GLCanvas_surface_h_

#include <Eigen/Eigen.h>
#include <Functions4U/Functions4U.h>
#include <Painter/Painter.h>
#include <RichText/RichText.h>

namespace Upp {
using namespace Eigen;

const double EPS_LEN  = 0.001,
			 EPS_SURF = EPS_LEN*EPS_LEN,
			 EPS_VOL  = EPS_LEN*EPS_LEN*EPS_LEN;
			 
template<class T>
inline T avg(T a, T b) 			{return T(a+b)/2;}
template<class T>
inline T avg(T a, T b, T c)		{return T(a+b+c)/3;}
 
template<class T>
void Sort(T& a, T& b, T& c, T& d) {
	if (a > b) 
		Swap(a, b);
	if (c > d) 
		Swap(c, d);
	if (a > c) 
		Swap(a, c);
	if (b > d) 
		Swap(b, d);
	if (b > c) 
		Swap(b, c);
}

template<class T>
void Sort(T& a, T& b, T& c) {
	if (a > b) 
		Swap(a, b);
	if (a > c) 
		Swap(a, c);
	if (b > c) 
		Swap(b, c);
}

enum RotationOrder {XYZ, YZX, ZYX};

class Value3D : public Moveable<Value3D> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	double x, y, z;
	
	Value3D() 									{}
	Value3D(const Value3D &p, int)				{Set(p);}
	Value3D(const Value3D &p) 					{Set(p);}
	Value3D(const Vector3d &p) 					{Set(p);}
	Value3D(const Vector<double> &p) 			{Set(p);}
	Value3D(double _x, double _y, double _z) 	{Set(_x, _y, _z);}
	
	Value3D(const Nuller&) 		{SetNull();}
	void SetNull() 				{x = y = z = Null;}
	bool IsNullInstance() const	{return IsNull(x) || IsNull(y) || IsNull(z);}
	
	void SetZero()				{x = y = z = 0;}
	static Value3D Zero() 		{return Value3D(0, 0, 0);}
	
	static int size() 			{return 3;}
	
	Value3D(bool positive)	{x = Null; y = positive ? 1 : -1;}
	bool IsPosInf()			{return IsNull(x) && y == 1;}
	bool IsNegInf()			{return IsNull(x) && y == -1;}
	
	void Set(const Value3D &p) 					{x = p.x;	y = p.y;	z = p.z;}
	void Set(const Vector3d &p) 				{x = p(0);	y = p(1);	z = p(2);}
	void Set(const Vector<double> &p) 			{x = p[0];	y = p[1];	z = p[2];}
	void Set(double _x, double _y, double _z) 	{x = _x;  	y = _y;  	z = _z;}
	
	inline Value3D operator=(const Value3D &p)	{Set(p);	return *this;}
	inline Value3D operator=(const Vector3d &p)	{Set(p);	return *this;}
	
    operator Eigen::Vector3d() const {return Eigen::Vector3d(x, y, z);}
    operator Eigen::VectorXd() const {return Eigen::Vector3d(x, y, z);}
    operator Eigen::MatrixXd() const {
        Eigen::MatrixXd m(1, 3);
        m << x, y, z;
        return m;
    }
    
	String ToString() const {return Format("x: %s. y: %s. z: %s", FDS(x, 10, true), FDS(y, 10, true), FDS(z, 10, true));}
	
	inline bool IsSimilar(const Value3D &p, double similThres) const {
		if (IsNull(x) && IsNull(p.x))
			return true;
		if (abs(p.x - x) < similThres && abs(p.y - y) < similThres && abs(p.z - z) < similThres)
			return true;
		return false;
	}
	#pragma GCC diagnostic ignored "-Wattributes"
	friend bool operator==(const Value3D& a, const Value3D& b) {return a.IsSimilar(b, EPS_LEN);}
	friend bool operator!=(const Value3D& a, const Value3D& b) {return !a.IsSimilar(b, EPS_LEN);}
	#pragma GCC diagnostic warning "-Wattributes"
	
	void Translate(double dx, double dy, double dz);
	void TransRot(const Affine3d &quat);
	void TransRot(double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz, RotationOrder order = RotationOrder::XYZ);
	void TransRot(const Value3D &trans, const Value3D &rot, const Value3D &centre, RotationOrder order = RotationOrder::XYZ);
	void Rotate(double ax, double ay, double az, double cx, double cy, double cz, RotationOrder order = RotationOrder::XYZ);	
	void TransRot000(double dx, double dy, double dz, double ax, double ay, double az, RotationOrder order = RotationOrder::XYZ);
	void TransRot000(const Value3D &trans, const Value3D &rot, RotationOrder order = RotationOrder::XYZ);
	
	// Dot product or scalar product
	inline double dot(const Value3D& a) const {return x*a.x + y*a.y + z*a.z;}
	
	// Cross product or vector product X (or wedge product ∧ in 3D) 
	inline friend Value3D operator%(const Value3D& a, const Value3D& b) {return Value3D(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);}
	
	inline friend Value3D operator-(const Value3D& a) {return Value3D(-a.x, -a.y, -a.z);}

	inline friend Value3D operator+(const Value3D& a, const Value3D& b) {return Value3D(a.x+b.x, a.y+b.y, a.z+b.z);}
	inline friend Value3D operator-(const Value3D& a, const Value3D& b) {return Value3D(a.x-b.x, a.y-b.y, a.z-b.z);}
	inline friend Value3D operator*(const Value3D& a, double b) 		{return Value3D(a.x*b, a.y*b, a.z*b);}
	inline friend Value3D operator/(const Value3D& a, double b) 		{return Value3D(a.x/b, a.y/b, a.z/b);}

    friend Value3D operator*(double b, const Value3D& a);
	friend Value3D operator/(double b, const Value3D& a);

	inline void operator+=(const Value3D& a) {x += a.x; y += a.y; z += a.z;}
	inline void operator-=(const Value3D& a) {x -= a.x; y -= a.y; z -= a.z;}
	inline void operator*=(double a) 		 {x *= a; y *= a; z *= a;}
	inline void operator/=(double a) 		 {x /= a; y /= a; z /= a;}

	double& operator[](int id) {
		ASSERT(id >= 0 && id < 3);
		switch (id) {
		case 0:	return x;
		case 1:	return y;
		default:return z;
		}
	}
	const double& operator[](int id) const {
		ASSERT(id >= 0 && id < 3);
		switch (id) {
		case 0:	return x;
		case 1:	return y;
		default:return z;
		}
	}
	
	inline double Length()  const {return ::sqrt(Length2());}
	inline double Length2() const {return x*x + y*y + z*z;}
	inline double Norm() const	  {return Length();}
	inline double Norm2() const	  {return Length2();}
	
	Value3D ClosestPointToLine(const Value3D &a, const Value3D &b) const;
	double DistanceToSegment(const Value3D &a, const Value3D &b) const;
	double DistanceToClosedCylinder(const Value3D &a, const Value3D &b, double r, bool considerBase) const;
	double DistanceToLine(const Value3D &a, const Value3D &b) const;
	double DistanceToCylinder(const Value3D &a, const Value3D &b, double r) const;
		
	Value3D &Normalize() {
		double length = Length();
		
		if (length < 1e-10) 
			SetZero();
		else {
		    x = x/length;
		    y = y/length;
		    z = z/length;
		}
		return *this;
	}
	double Manhattan() const {return abs(x) + abs(y) + abs(z);}

	double Angle(const Value3D &p) 		 const {return acos(dot(p)/(Length()*p.Length()));}
	double AngleNormal(const Value3D &p) const {return acos(dot(p));}
	
	void SimX() {x = -x;}
	void SimY() {y = -y;}
	void SimZ() {z = -z;}
	
	void Mirror(const Value3D &p0) {
		x = 2*p0.x - x;
		y = 2*p0.y - y;
		z = 2*p0.z - z;
	}
	void Mirror() {
		SimX();
		SimY();
		SimZ();
	}
	void Jsonize(JsonIO &json)  {json("x", x)("y", y)("z", z);}
	void Serialize(Stream& s)	{s % x % y % z;}
    void Xmlize(XmlIO& xio)     {xio("x", x)("y", y)("z", z);}
};

bool IsNum(const Value3D &v);

Value3D operator*(double b, const Value3D& a);
Value3D operator/(double b, const Value3D& a);

void TransRot(const Value3D &pos, const Value3D &ref, const VectorXd &transf, Value3D &npos);
void TransRot(const Value3D &pos, const Value3D &ref, double x, double y, double z, double rx, double ry, double rz, Value3D &npos);
void TransRot(const Value3D &pos, const Value3D &ref, VectorXd &x, VectorXd &y, VectorXd &z, VectorXd &rx, VectorXd &ry, VectorXd &rz);
void TransRot000(const Value3D &pos, double x, double y, double z, double rx, double ry, double rz, Value3D &npos);

bool TransRotChangeRef(const Value3D &ref, const VectorXd &transf, const Value3D &nref, VectorXd &ntransf);
	
typedef Value3D Vector3D;
typedef Value3D Point3D;

Affine3d GetTransformRotation000(const Value3D &rot, RotationOrder order = RotationOrder::XYZ);
Affine3d GetTransform000(const Value3D &trans, const Value3D &rot, RotationOrder order = RotationOrder::XYZ);
Affine3d GetTransformRotation(const Value3D &rot, const Point3D &centre, RotationOrder order = RotationOrder::XYZ);
Affine3d GetTransform(const Value3D &trans, const Value3D &rot, const Point3D &centre, RotationOrder order = RotationOrder::XYZ);

void TransRot(const Affine3d &aff, const Value3D &pos, Value3D &npos);
	
class Value6D : public Moveable<Value6D> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Value3D t, r;
	
	Value6D() {}
	Value6D(const Value6D &f)		{Set(f);}
	Value6D(const Value6D &f, int)	{Set(f);}
	Value6D(const VectorXd &v)		{Set(v);}
	template<typename T>
	Value6D(const T *v)				{Set(v);}
	Value6D(double v0, double v1, double v2, double v3, double v4, double v5) {Set(v0, v1, v2, v3, v4, v5);}
	Value6D(const Value3D &tr, const Value3D &ro) {t = tr; r = ro;}
	
	Value6D(const Nuller&) 		{SetNull();}
	void SetNull() 				{t = Null;}
	bool IsNullInstance() const	{return IsNull(t) || IsNull(r);}
	
	void Set(const Value6D &f)	{t.x = f.t.x;	t.y = f.t.y;	t.z = f.t.z;	r.x = f.r.x;	r.y = f.r.y;	r.z = f.r.z;}
	void Set(const VectorXd &v) {
		ASSERT(v.size() == 6);
		Set(v.data());
	}
	template<typename T>
	void Set(const T *v) {
		t.x = v[0];	t.y = v[1];	t.z = v[2];	r.x = v[3];	r.y = v[4];	r.z = v[5];
	}
	void Set(double v0, double v1, double v2, double v3, double v4, double v5) {
		t.x = v0;	t.y = v1;	t.z = v2;	r.x = v3;	r.y = v4;	r.z = v5;
	}
	template<typename T>
	void Add(const T *v) {
		t.x+= v[0];	t.y+= v[1];	t.z+= v[2];	r.x+= v[3];	r.y+= v[4];	r.z+= v[5];
	}
	void Add(const VectorXd &v) {
		ASSERT(v.size() == 6);
		Add(v.data());
	}
	
	void SetZero() 			{t.SetZero();	r.SetZero();}
	static Value6D Zero() 	{return Value6D(0, 0, 0, 0, 0, 0);}
	
	inline bool IsSimilar(const Value6D &p, double similThres) const {
		return t.IsSimilar(p.t, similThres) && r.IsSimilar(p.r, similThres);
	}
	#pragma GCC diagnostic ignored "-Wattributes"
	friend bool operator==(const Value6D& a, const Value6D& b) {return a.IsSimilar(b, EPS_LEN);}
	friend bool operator!=(const Value6D& a, const Value6D& b) {return !a.IsSimilar(b, EPS_LEN);}
	#pragma GCC diagnostic warning "-Wattributes"
	
	static int size() 		{return 6;}
	
	void operator+=(const Value6D &v) 	{t.x += v.t.x;	t.y += v.t.y;	t.z += v.t.z;	r.x += v.r.x;	r.y += v.r.y;	r.z += v.r.z;}
	void operator*=(double v) 			{t.x *= v;		t.y *= v;		t.z *= v;		r.x *= v;		r.y *= v;		r.z *= v;}
	
	double& operator[](int id) {
		ASSERT(id >= 0 && id < 6);
		switch (id) {
		case 0:	return t.x;
		case 1:	return t.y;
		case 2:	return t.z;
		case 3:	return r.x;
		case 4:	return r.y;
		default:return r.z;
		}
	}
	double operator[](int id) const {
		ASSERT(id >= 0 && id < 6);
		switch (id) {
		case 0:	return t.x;
		case 1:	return t.y;
		case 2:	return t.z;
		case 3:	return r.x;
		case 4:	return r.y;
		default:return r.z;
		}
	}
	String ToString() const {
		return Format("x: %s. y: %s. z: %s. rx: %s. ry: %s. rz: %s", 
			FDS(t.x, 10, true), FDS(t.y, 10, true), FDS(t.z, 10, true),
			FDS(r.x, 10, true), FDS(r.y, 10, true), FDS(r.z, 10, true));
	}
	VectorXd ToVector() const {
		VectorXd v(6);
		ToC(v.data());
		return v;
	}
	template <typename T>
	void ToC(T *v) const {
		v[0] = T(t.x);	v[1] = T(t.y);	v[2] = T(t.z);	
		v[3] = T(r.x);	v[4] = T(r.y);	v[5] = T(r.z);
	}
	
	void Jsonize(JsonIO &json)  {json("t", t)("r", r);}
	void Serialize(Stream& s)	{s % t % r;}
    void Xmlize(XmlIO& xio)     {xio("t", t)("r", r);}
};

bool IsNum(const Value6D &v);

class ForceVector;

class Force6D : public Moveable<Force6D>, public Value6D {
public:
	Force6D() : Value6D() {}
	Force6D(const VectorXd &v) : Value6D(v) {}
	Force6D(double v0, double v1, double v2, double v3, double v4, double v5) : Value6D(v0, v1, v2, v3, v4, v5) {}
	
	static Force6D Zero() 	{return Force6D(0, 0, 0, 0, 0, 0);}
		
	void Add(const Vector3D &dir, const Point3D &point, const Point3D &c0);	
	void Add(const Force6D &force, const Point3D &point, const Point3D &c0);
	void Add(const ForceVector &force, const Point3D &c0);
	
	Force6D(const Nuller&) 		{SetNull();}
};

class ForceVector {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	ForceVector() {}
	ForceVector(const Point3D &p, const Force6D &f) {
		Set(p, f);
	}
	ForceVector(double x, double y, double z, double fx, double fy, double fz, double rx, double ry, double rz) {
		Set(x, y, z, fx, fy, fz, rx, ry, rz);
	}
	void Set(double x, double y, double z, double fx, double fy, double fz, double rx, double ry, double rz) {
		point.x = x;
		point.y = y;
		point.z = z;
		force.t.x = fx;
		force.t.y = fy;
		force.t.z = fz;
		force.r.x = rx;
		force.r.y = ry;
		force.r.z = rz;
	}
	void Set(const Point3D &p, const Force6D &f) {
		point = clone(p);
		force = clone(f);
	}
	
	ForceVector &TransRot(double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz);
	ForceVector &TransRot(const Affine3d &aff);
	
	ForceVector &Translate(const Point3D &p);
	
	String ToString() const {return "From: " + point.ToString() + ". Value: " + force.ToString();}
	
	Force6D force;
	Point3D point;
};

// For dummies: https://dynref.engr.illinois.edu/rkg.html
class Velocity6D : public Moveable<Velocity6D>, public Value6D {
public:
	Velocity6D() : Value6D() {}
	template<typename T>
	Velocity6D(const T *v) {Set(v);}
	Velocity6D(const Value3D &tr, const Value3D &ro) {t = tr; r = ro;}
	
	void Translate(const Point3D &from, const Point3D &to) {
		Vector3D rpq = to - from;
		Translate(rpq);
	}
	void Translate(const Vector3D &rpq) {
		t += r%rpq;
	}
	
	Velocity6D(const Nuller&) 		{SetNull();}
};

class Acceleration6D : public Moveable<Acceleration6D>, public Value6D {
public:
	Acceleration6D() : Value6D() {}
	template<typename T>
	Acceleration6D(const T *v) {Set(v);}
	Acceleration6D(const Value3D &tr, const Value3D &ro) {t = tr; r = ro;}
	
	void Translate(const Point3D &from, const Point3D &to, const Velocity6D &vel) {
		Vector3D rpq = to - from;
		Translate(rpq, vel);
	}
	void Translate(const Vector3D &rpq, const Velocity6D &vel) {
		t += r%rpq + vel.r%(vel.r%rpq);
	}
	
	Acceleration6D(const Nuller&) 		{SetNull();}
};

VectorXd C6ToVector(const double *c);
VectorXd C6ToVector(const float *c);
void Vector6ToC(const VectorXd &v, double *c);
void Vector6ToC(const VectorXd &v, float *c);


double Distance(const Value3D &p1, const Value3D &p2);
double Length(const Value3D &p1, const Value3D &p2);
double Manhattan(const Value3D &p1, const Value3D &p2);
Value3D Middle(const Value3D &a, const Value3D &b);
Value3D WeightedMean(const Value3D &a, double va, const Value3D &b, double vb);
Value3D Centroid(const Value3D &a, const Value3D &b, const Value3D &c);
Vector3D Normal(const Value3D &a, const Value3D &b, const Value3D &c);
bool Collinear(const Value3D &a, const Value3D &b, const Value3D &c);
double Area(const Value3D &p0, const Value3D &p1, const Value3D &p2);

bool Collinear(const Pointf &a, const Pointf &b, const Pointf &c);
double Area(const Pointf &p0, const Pointf &p1, const Pointf &p2);
double Direction(const Pointf& a, const Pointf& b);
 

class Segment3D : public Moveable<Segment3D> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Point3D from, to;
	
	Segment3D() {}
	Segment3D(const Nuller&) {SetNull();}
	Segment3D(const Point3D &_from, const Point3D &_to) : from(_from), to(_to) {}
	Segment3D(const Point3D &_from, const Vector3D &normal, double length) : from(_from) {
		to = Point3D(from.x + length*normal.x, from.y + length*normal.y, from.z + length*normal.z);
	}
	void SetNull() 				{from = to = Null;}
	bool IsNullInstance() const	{return IsNull(from) || IsNull(to);}
		
	void Set(const Point3D &_from, const Point3D &_to) {
		from.Set(_from);
		to.Set(_to);
	}
	void Set(const Point3D &_from, const Vector3D &normal, double length) {
		from.Set(_from);
		to.Set(from.x + length*normal.x, from.y + length*normal.y, from.z + length*normal.z);
	}
	void SimX() {
		from.SimX();
		to.SimX();
	}
	void SimY() {
		from.SimY();
		to.SimY();
	}
	void SimZ() {
		from.SimZ();
		to.SimZ();
	}
	double Length() const {return Upp::Length(from, to);}
	double Dx()	const 	  {return to.x - from.x;}
	double Dy()	const 	  {return to.y - from.y;}	
	double Dz()	const 	  {return to.z - from.z;}
	
	void Mirror(const Point3D &p0) {
		from.Mirror(p0);
		to.Mirror(p0);
	}
	
	Vector3D Direction() {return Vector3D(to - from);}
	
	Point3D IntersectionPlaneX(double x);
	Point3D IntersectionPlaneY(double y);
	Point3D IntersectionPlaneZ(double z);
	
	Point3D Intersection(const Point3D &planePoint, const Vector3D &planeNormal);
	
	bool PointIn(const Point3D &p) const;
	bool SegmentIn(const Segment3D &in, double in_len) const;
	bool SegmentIn(const Segment3D &in) const;
	
	void Translate(double dx, double dy, double dz) {
		from.Translate(dx, dy, dz);
		to.Translate(dx, dy, dz);
	}
	void TransRot(const Affine3d &quat) {
		from.TransRot(quat);
		to.TransRot(quat);
	}
	void Scale(double rx, double ry, double rz, const Point3D &c0) {
		from.Translate(rx*(from.x -c0.x), ry*(from.y -c0.y), rz*(from.z -c0.z)); 
		to.Translate(rx*(to.x -c0.x), ry*(to.y -c0.y), rz*(to.z -c0.z)); 
	}
};

void DeleteVoidSegments(Vector<Segment3D> &segs);
void DeleteDuplicatedSegments(Vector<Segment3D> &segs);

Point3D Intersection(const Vector3D &lineVector, const Point3D &linePoint, const Vector3D &planeNormal, const Point3D &planePoint);

void TranslateForce(const Point3D &from, const VectorXd &ffrom, Point3D &to, VectorXd &fto);
	
bool PointInSegment(const Point3D &p, const Point3D &from, const Point3D &to);
bool PointInSegment(const Point3D &p, const Segment3D &seg);
bool PointInSegment(const Pointf &p, const Pointf &from, const Pointf &to);
	
bool SegmentInSegment(const Segment3D &in, double in_len, const Segment3D &seg);
bool SegmentInSegment(const Segment3D &in, const Segment3D &seg);

template <typename T>
inline T const& maxNotNull(T const& a, T const& b) {
	if (IsNull(a))
		return b;
	else if (IsNull(b))
		return a;
	else
    	return a > b ? a : b;
}

template <typename T>
inline T const& minNotNull(T const& a, T const& b) {
	if (IsNull(a))
		return b;
	else if (IsNull(b))
		return a;
	else
    	return a < b ? a : b;
}

class Panel : public Moveable<Panel> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	int id[4];
	Point3D centroid0, centroid1, centroidPaint;
	Point3D normal0, normal1, normalPaint;
	double surface0, surface1;
	
	Panel() {}
	Panel(const Panel &orig, int) {
		memcpy(id, orig.id, sizeof(orig.id));
		centroid0 = orig.centroid0;
		centroid1 = orig.centroid1;
		centroidPaint = orig.centroidPaint;
		normal0 = orig.normal0;
		normal1 = orig.normal1;
		normalPaint = orig.normalPaint;
		surface0 = orig.surface0;
		surface1 = orig.surface1;
	}
	bool operator==(const Panel &p) const {
		int id0 = id[0], id1 = id[1], id2 = id[2], id3 = id[3];
		int pid0 = p.id[0], pid1 = p.id[1], pid2 = p.id[2], pid3 = p.id[3];
		if (id0 + id1 + id2 + id3 != pid0 + pid1 + pid2 + pid3)
			return false;
		Sort(id0, id1, id2, id3);
		Sort(pid0, pid1, pid2, pid3);
		if (id0 == pid0 && id1 == pid1 && id2 == pid2 && id3 == pid3)
			return true;
		return false;
	}
	inline void Swap() {
		if (IsTriangle()) {
			Upp::Swap(id[1], id[2]);
			id[3] = id[2];
		} else
			Upp::Swap(id[1], id[3]);
	}
	inline bool IsTriangle() const	{return id[0] == id[1] || id[0] == id[2] || id[0] == id[3] || 
											id[1] == id[2] || id[1] == id[3] || id[2] == id[3];}
	void RedirectTriangles();
	void ShiftNodes(int shift);
	inline int GetNumNodes() const	{return IsTriangle() ? 3 : 4;}
	bool FirstNodeIs0(int in0, int in1) const;
	
	String ToString() const { return FormatInt(id[0]) + "," + FormatInt(id[1]) + "," + FormatInt(id[2]) + "," + FormatInt(id[3]); }

	void Jsonize(JsonIO &json) {
		Vector<int> ids;
		if (json.IsStoring()) {
			ids << id[0];
			ids << id[1];
			ids << id[2];
			ids << id[3];	
		}
		json
			("ids", ids)
		;
		if (json.IsLoading()) {
			id[0] = ids[0];
			id[1] = ids[1];
			id[2] = ids[2];
			id[3] = ids[3];
		}
	}
	
	Value6D NormalExt(const Point3D &c0) const;
};

class LineSegment : public Moveable<LineSegment> {
public:
	LineSegment() {}
	LineSegment(const LineSegment &orig, int) {Copy(orig);}
	LineSegment(const LineSegment &orig) {Copy(orig);}
	void Copy(const LineSegment &orig) {
		idNod0 = orig.idNod0;
		idNod1 = orig.idNod1;
		idPans = clone(orig.idPans);
	}
	int idNod0, idNod1;
	Upp::Index<int> idPans;
};

class Line : public Moveable<Line> {
public:
	Line() {}
	Line(const Line &orig, int) {Copy(orig);}
	Line(const Line &orig) {Copy(orig);}
	void Copy(const Line &orig) {
		lines = clone(orig.lines);
		radius = clone(orig.radius);
		toPlot = clone(orig.toPlot);
	}
	void Jsonize(JsonIO &json) {
		json
			("lines", lines)
			("radius", radius)
			("toPlot", toPlot)
		;
	}
	
	Upp::Vector<Point3D> lines;
	Upp::Vector<double> radius;
	Upp::Vector<Point3D> toPlot;
};

class VolumeEnvelope : public Moveable<VolumeEnvelope> {
public:
	VolumeEnvelope() {Reset();}
	void Reset() 	 {maxX = minX = maxY = minY = maxZ = minZ = Null;}
	bool IsNull2D() const {return Upp::IsNull(maxX) || Upp::IsNull(minX) || Upp::IsNull(maxY) || Upp::IsNull(minY);}
	bool IsNull() 	const {return IsNull2D() || Upp::IsNull(maxZ) || Upp::IsNull(minZ);}
	VolumeEnvelope(const VolumeEnvelope &orig, int) {
		maxX = orig.maxX;
		minX = orig.minX;
		maxY = orig.maxY;
		minY = orig.minY;
		maxZ = orig.maxZ;
		minZ = orig.minZ;
	}	
	void Set(const Vector<Point3D> &points);
	
	void MixEnvelope(const VolumeEnvelope &env);
	double Max()	{return maxNotNull(maxNotNull(max(abs(maxX), abs(minX)), maxNotNull(abs(maxY), abs(minY))), maxNotNull(abs(maxZ), abs(minZ)));}
	double LenRef()	{return maxNotNull(maxNotNull(maxX - minX, maxY - minY), maxZ - minZ);}
	
	double maxX, minX, maxY, minY, maxZ, minZ;
};

class Surface : Moveable<Surface> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Surface() {}
	Surface(const Surface &surf, int)		{Copy(surf);}
	Surface(const Surface &surf)			{Copy(surf);}
	Surface& operator=(const Surface &surf) {Copy(surf); return *this;};
	Surface(Surface &&surf) noexcept;
	
	bool IsValid() const	{return magic == 1234567890;}
	virtual ~Surface() 		{magic = 505;}
	
	void Copy(const Surface &surf);
	void Clear();
	bool IsEmpty() const;
	
	void LoadSerialization(String fileName);
	void SaveSerialization(String fileName) const;

	Vector<Point3D> nodes;
	Vector<Panel> panels;
	Vector<LineSegment> segments;
	Array<Line> lines;
	
	int GetNumNodes() const		{return nodes.size();}
	int GetNumPanels() const	{return panels.size();}
	int GetNumSegments() const	{return segments.size();}
	
	Vector<Segment3D> skewed;
	Vector<Segment3D> segWaterlevel, segTo1panel, segTo3panel;
	
	VolumeEnvelope env;
	
	int AddLine(const Vector<Point3D> &points3D, const Vector<double> &radius);
	int AddLine(const Vector<Point3D> &points3D);
	int AddLine(const Vector<Pointf> &points);
	
	void SetLine(int id, const Vector<Point3D> &points3D, const Vector<double> &radius);
	void SetLine(int id, const Vector<Point3D> &points3D);
	void SetLine(int id, const Vector<Pointf> &points); 
	
	String Heal(bool basic, double grid, double eps, Function <bool(String, int pos)> Status = Null);
	Surface &Orient();
	Surface &OrientFlat();
	void Mirror(int axis);
	const VolumeEnvelope &GetEnvelope(); 
	void RedirectTriangles();
	void GetPanelParams();
	String CheckErrors() const;
	double GetArea();
	double GetAreaXProjection(bool positive, bool negative) const;
	double GetAreaYProjection(bool positive, bool negative) const;
	double GetAreaZProjection(bool positive, bool negative) const;
	Pointf GetAreaZProjectionCG() const;
	void GetSegments();
	void GetNormals();
	double GetAvgLenSegment() const {return avgLenSegment;}
	void GetVolume();
	int VolumeMatch(double ratioWarning, double ratioError) const;
	double VolumeRatio() const;
	Point3D GetCentreOfBuoyancy() const;
	Point3D GetCentreOfGravity_Surface() const;
	bool GetInertia33_Volume(Matrix3d &inertia, const Point3D &cg, bool refine = false) const;
	bool GetInertia33_Surface(Matrix3d &inertia, const Point3D &cg, bool refine = false) const;
	bool GetInertia33(Matrix3d &inertia, const Point3D &cg, bool byVolume, bool refine = false) const;
	static void GetInertia66(MatrixXd &inertia, const Matrix3d &inertia33, const Point3D &cg, const Point3D &c0, bool refine);
	static void GetInertia33_Radii(Matrix3d &inertia);
	bool GetInertia33_Radii(Matrix3d &inertia, const Point3D &c0, bool byVolume, bool refine) const;
	static void FillInertia66mc(MatrixXd &inertia, const Point3D &cg, const Point3D &c0);
		
	static void TranslateInertia33(Matrix3d &inertia, double m, const Point3D &cg, const Point3D &c0, const Point3D &nc0);
	static void TranslateInertia66(MatrixXd &inertia, const Point3D &cg, const Point3D &c0, const Point3D &nc0);
	Force6D GetHydrostaticForce(const Point3D &c0, double rho, double g) const;
	Force6D GetHydrostaticForceNormalized(const Point3D &c0) const;
	static Force6D GetHydrostaticForceCB(const Point3D &c0, const Point3D &cb, double volume, double rho, double g);
	static Force6D GetHydrostaticForceCBNormalized(const Point3D &c0, const Point3D &cb, double volume);
	static Force6D GetMassForce(const Point3D &c0, const Point3D &cg, const double mass, const double g);
	static Force6D GetMassForce(const Point3D &c0, const Vector<Point3D> &cgs, const Vector<double> &masses, const double g);
	void GetHydrostaticStiffness(MatrixXd &c, const Point3D &c0, const Point3D &cg, 
				const Point3D &cb, double rho, double g, double mass, bool massBuoy);
	Force6D GetHydrodynamicForce(const Point3D &c0, bool clip, Function<double(double x, double y)> GetZSurf,
						Function<double(double x, double y, double z, double et)> GetPress) const;
	double GetWaterPlaneArea() const;
	
	void AddWaterSurface(Surface &surf, const Surface &under, char c, double grid = Null, double eps = Null, double meshRatio = 1, bool quads = false);
	static Vector<Segment3D> GetWaterLineSegments(const Surface &orig);
	bool GetDryPanels(const Surface &surf, bool onlywaterplane, double grid, double eps);
	bool GetSelPanels(const Surface &orig, const Vector<int> &panelIds, double grid, double eps);
		
	char IsWaterPlaneMesh() const; 
	
	void TrianglesToQuadsFlat();
		
	void CutX(const Surface &orig, int factor = 1);
	void CutY(const Surface &orig, int factor = 1);
	void CutZ(const Surface &orig, int factor = 1);
	
	Surface &Append(const Surface &orig);
	Surface &operator<<(const Surface &orig) {return Append(orig);}
	
	Vector<Vector<int>> GetPanelSets(Function <bool(String, int pos)> Status);
	
	void TriangleToQuad(int ip);
	void TriangleToQuad(Panel &pan);
	void TriangleToFalseQuad(int ipanel);
	void QuadToQuad(int ip);
	void QuadToQuad(Panel &pan);
	
	void TrianglesToFalseQuads();
		
	Surface &Translate(double dx, double dy, double dz);
	Surface &Rotate(double ax, double ay, double az, double _c_x, double _c_y, double _c_z);
	Surface &TransRot(double dx, double dy, double dz, double ax, double ay, double az, double _c_x, double _c_y, double _c_z);
	
	bool TranslateArchimede(double allmass, double rho, double ratioError, const UVector<Surface *> &damaged, double tolerance, double &dz, Point3D &cb, double &allvol);
	
	bool PrincipalComponents(Value3D &ax1, Value3D &ax2, Value3D &ax3);
	double YawMainAxis();
		
	void Scale(double rx, double ry, double rz, const Point3D &c0);
	
	bool healing{false};
	int numTriangles, numBiQuads, numMonoQuads;
	double avgLenSegment = -1;
	int numDupPan, numDupP, numSkewed;
	
	double surface = -1, volume = -1, volumex = -1, volumey = -1, volumez = -1;
	double avgFacetSideLen;
	
	void DeployXSymmetry();
	void DeployYSymmetry();
	
	Surface &SelPanels(Vector<int> &_selPanels) {selPanels = pick(_selPanels);	return *this;}
	Surface &SelNodes(Vector<int> &_selNodes) 	{selNodes = pick(_selNodes);	return *this;}
	const Vector<int> &GetSelPanels() const		{return selPanels;}
	const Vector<int> &GetSelNodes() const		{return selNodes;}
	Surface &AddSelPanel(int id) 				{FindAdd(selPanels, id);		return *this;}
	Surface &RemoveSelPanel(int id) {
		int idfind = Find(selPanels, id);
		if (idfind >= 0) 
			selPanels.Remove(idfind);
		return *this;
	}
	void ClearSelPanels() 						{selPanels.Clear();}
	
	void RemovePanels2(const UVector<int> &panels);
	
	void AddNode(const Point3D &p);
	int FindNode(const Point3D &p);
	
	void AddFlatRectangle(double lenX, double lenY, double panelWidth, double panelHeight);
	void AddRevolution(const Vector<Pointf> &points, double panelWidth);
	void AddPolygonalPanel(const Vector<Pointf> &bound, double panelWidth, bool adjustSize, bool quads);
	void Extrude(double dx, double dy, double dz, bool close);
	void AddPanels(const Surface &from, UVector<int> &panelIds);
		
	static void RoundClosest(Vector<Point3D> &_nodes, double grid, double eps);	
	static int RemoveDuplicatedPanels(Vector<Panel> &_panels);
	static int RemoveTinyPanels(Vector<Panel> &_panels);
	static int RemoveDuplicatedPointsAndRenumber(Vector<Panel> &_panels, Vector<Point3D> &_nodes);
	static void DetectTriBiP(Vector<Panel> &panels) {int dum;	DetectTriBiP(panels, dum, dum, dum);}
	
	String CheckNodeIds();
		
	void GetClosestPanels(int idPanel, UVector<int> &panIDs);
	
	Vector<int> GetBoundary();
	Vector<bool> GetBoundaryBool();
	void SmoothLaplacian(double lambda, int iterations, int w);
	void SmoothTaubin(double lambda, double mu, int iterations, int w);
	
	void Jsonize(JsonIO &json) {
		json
			("nodes", nodes)
			("panels", panels)
			("lines", lines)
		;
	}
		
protected:
	struct PanelPoints {
		PanelPoints() {data.SetCount(4);}
		Vector<Point3D> data;
	};
	struct TrianglesPoints2D {
		TrianglesPoints2D() {data.SetCount(3);}
		Vector<Pointf> data;
	};
	void SetPanelPoints(const Array<PanelPoints> &pans);
	
private:
	inline bool CheckId(int id) {return id >= 0 && id < nodes.GetCount()-1;}
	bool side = true;
	
	static void DetectTriBiP(Vector<Panel> &panels, int &numTri, int &numBi, int &numP);
	int FixSkewed();
	bool FixSkewed(int ipanel);
	int SegmentInSegments(int iseg) const;
	void AnalyseSegments(double zTolerance);
	void AddSegment(int ip0, int ip1, int ipanel);
	bool ReorientPanels0(bool side);
	void ReorientPanel(int ip);
	bool GetLowest(int &iLowSeg, int &iLowPanel);
	bool SameOrderPanel(int ip0, int ip1, int in0, int in1);
	static int PanelGetNumNodes(const Vector<Panel> &_panels, int ip) {return _panels[ip].GetNumNodes();}
	bool IsPanelTriangle(int ip) 	{return panels[ip].IsTriangle();}
	void GetPanelParams(Panel &panel) const;
	void JointTriangularPanels(int ip0, int ip1, int inode0, int inode1);
	bool FindMatchingPanels(const Array<PanelPoints> &pans, double x, double y, 
							double panelWidth, int &idpan1, int &idpan2);
	void SmoothLaplacian(double lambda, int iterations, int w, const Vector<bool> &boundaryNodes);
	void SmoothLaplacianWeightLess(double lambda, const Vector<bool> &boundaryNodes);
	void SmoothLaplacianWeight(double lambda, const Vector<bool> &boundaryNodes);
	void SmoothImplicitLaplacian(double lambda, const Vector<bool> &boundaryNodes);
	
	Vector<int> selPanels, selNodes;
	int magic = 1234567890;
};

class SurfaceMass  {
public:
	double mass;
	Point3D cg;
	Surface surface;
	
	void Jsonize(JsonIO &json) {
		if (json.IsLoading())
			mass = Null;
		json
			("mass", mass)
			("cg", cg)
			("surface", surface)
		;
	}	
};

class SurfaceX : public Moveable<SurfaceX> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	SurfaceX() {}
	SurfaceX(const SurfaceX &, int) {}
	
	void Load(const Surface &parent);
	
	static double GetVolume(const SurfaceX &surf, double &vx, double &vy, double &vz);
	static double GetVolume(const SurfaceX &surf);
	static double GetSurface(const SurfaceX &surf);
	
	static double GetSurface(const SurfaceX &surf, bool (*Fun)(double,double,double));
	static double GetVolume(const SurfaceX &surf, bool (*Fun)(double,double,double));
	
	static void TransRot(SurfaceX &surf, const SurfaceX &surf0, const Affine3d &quat); 
	
	static void GetTransformFast(MatrixXd &mat, double dx, double dy, double dz, double ax, double ay, double az);
	static void TransRotFast(double &x, double &y, double &z, double x0, double y0, double z0, const MatrixXd &mat);
	static void TransRotFast(double &x, double &y, double &z, double x0, double y0, double z0,
						 	 double dx, double dy, double dz, double ax, double ay, double az);
	
	//static void GetTransform(Affine3d &aff, double dx, double dy, double dz, double ax, double ay, double az);
	//static void TransRot(double &x, double &y, double &z, double x0, double y0, double z0, const Affine3d &quat);
	//static void TransRot(double &x, double &y, double &z, double x0, double y0, double z0,
	//					 double dx, double dy, double dz, double ax, double ay, double az);
	
private:
	const Surface *parent = nullptr;

public:
	VectorXd surfaces;	// (panels)
	MatrixXd centroids;	// (panels, 3)
	MatrixXd normals;	// (panels, 3)
};

template<class Range>
void Translate(Range &r, double dx, double dy, double dz) {
	for (Value3D &p : r)
		p.Translate(dx, dy, dz);
}

template<class Range>
void TransRot(Range &r, const Affine3d &quat) {
	for (Value3D &p : r)
		p.TransRot(quat);
}

template<class Range>
void TransRot(Range &r, double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz) {
	for (Value3D &p : r)
		p.TransRot(dx, dy, dz, ax, ay, az, cx, cy, cz);
}

template<class Range>
void Rotate(Range &r, double ax, double ay, double az, double cx, double cy, double cz) {
	for (Value3D &p : r)
		p.Rotate(ax, ay, az, cx, cy, cz);
}

	
void LoadStl(String fileName, Surface &surf, bool &isText, String &header);
void LoadStl(String fileName, Surface &surf);
void SaveStlTxt(String fileName, const Surface &surf);
void SaveStlBin(String fileName, const Surface &surf);

void LoadTDynMsh(String fileName, Surface &surf);
void LoadGMSH(String fileName, Surface &surf);

void LoadGRD(String fileName, Surface &surf, bool &y0z, bool &x0z);
void SaveGRD(String fileName, Surface &surf, double g, bool y0z, bool x0z);

void LoadOBJ(String fileName, Surface &surf);
	
enum ContainsPointRes {POLY_NOPLAN = -4, POLY_FAR = -3, POLY_3 = -2, POLY_OUT = -1, POLY_SECT = 0, POLY_IN = 1};
ContainsPointRes ContainsPoint(const Vector<Point3D> &polygon, const Point3D &point, double distanceTol, double angleNormalTol);
ContainsPointRes ContainsPoint(const Vector<Pointf>& polygon, const Pointf &pt);

bool IsClockwise(const UVector<Pointf> &p);
	
template<typename T>
auto cross(const Point_<T>& a, const Point_<T>& b, const Point_<T>& c) {
    return (b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x);
}

template<typename T>
bool ContainsPoint(const Point_<T> &a, const Point_<T> &b, const Point_<T> &c, const Point_<T> &p) {
    auto cross1 = cross(a, b, p);
    auto cross2 = cross(b, c, p);
    auto cross3 = cross(c, a, p);

    bool has_neg = (cross1 < 0) || (cross2 < 0) || (cross3 < 0);
    bool has_pos = (cross1 > 0) || (cross2 > 0) || (cross3 > 0);

    return !(has_neg && has_pos); 
}

template<typename T>
bool ContainsPoint(const Point_<T> &a, const Point_<T> &b, const Point_<T> &c, const Point_<T> &d, const Point_<T> &p) {
    return ContainsPoint(a, b, c, p) || ContainsPoint(a, c, d, p);
}

Point3D Centroid(const UVector<Point3D> &p);
double Area(const UVector<Point3D> &p);
bool IsRectangle(const UVector<Point3D> &p);
bool IsFlat(const UVector<Point3D> &p);

Vector<Point3D> GetClosedPolygons(Vector<Segment3D> &segs);
Vector<Point3D> GetCircle(const Point3D &centre, const Vector3D &normal, double radius, int numPoints);
	
Pointf Centroid(const UVector<Pointf> &p);
double Area(const UVector<Pointf> &p);
bool IsRectangle(const UVector<Pointf> &p);

void Range(const UVector<Pointf> &p, double &minx, double &maxx, double &miny, double &maxy);
	
Vector<Pointf>  Point3Dto2D_XY(const Vector<Point3D> &bound);
Vector<Point3D> Point2Dto3D_XY(const Vector<Pointf>  &bound);
Vector<Pointf>  Point3Dto2D_XZ(const Vector<Point3D> &bound);
Vector<Point3D> Point2Dto3D_XZ(const Vector<Pointf>  &bound);
Vector<Pointf>  Point3Dto2D_YZ(const Vector<Point3D> &bound);
Vector<Point3D> Point2Dto3D_YZ(const Vector<Pointf>  &bound);

bool PointInPoly(const UVector<Pointf> &xy, const Pointf &pxy);

class ItemView : public Moveable<ItemView> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	int id[4];
	unsigned numIds;
	Point3D centroid;
	Point3D normal;
	Color color, meshColor;
	double thickness;
	int idBody = -1;
	int idSubBody = -1;
	int idString = -1;
	char overunder = '\0';
	
	ItemView() {}
	ItemView(const ItemView &orig, int) {
		memcpy(id, orig.id, sizeof(orig.id));
		numIds = orig.numIds;
		centroid = orig.centroid;
		normal = orig.normal;
		color = orig.color;
		meshColor = orig.meshColor;
		thickness = orig.thickness;
		overunder = orig.overunder;
		idBody = orig.idBody;
		idSubBody = orig.idSubBody;
		idString = orig.idString;
	}

	void Jsonize(JsonIO &json) {
		Vector<int> ids;
		if (json.IsStoring()) {
			ids << id[0];
			ids << id[1];
			ids << id[2];
			ids << id[3];	
		}
		json
			("ids", ids)
		;
		if (json.IsLoading()) {
			id[0] = ids[0];
			id[1] = ids[1];
			id[2] = ids[2];
			id[3] = ids[3];
		}
	}
};


class SurfaceView {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	enum ShowMesh  {SHOW_MESH, SHOW_VISIBLE_MESH, SHOW_FACES, SHOW_MESH_FACES, UNKNOWN};
	enum ShowColor {SHOW_DARKER, SHOW_BRIGHTER, SHOW_FLAT};

	constexpr static double OVER_ALL = std::numeric_limits<double>::max();
	constexpr static double UNDER_ALL = std::numeric_limits<double>::lowest();
	
	void Clear();

	SurfaceView &PaintSurface(Surface &mesh, Color color, Color meshColor, double thick, int idBody = -1, bool showNormals = false, double normalLen = -1);
	SurfaceView &PaintLine(double x0, double y0, double z0, double x1, double y1, double z1, const Color &color, double thick, double lenDelta = -20);
	SurfaceView &PaintLine(const Point3D &p0, const Point3D &p1, const Color &color, double thick, double lenDelta = -20);
	SurfaceView &PaintLine(const Segment3D &p, const Color &color, double thick, double lenDelta = -20);
	SurfaceView &PaintLines(const Vector<Point3D>& lines, const Color &color, double thick = Null, double lenDelta = -20);
	SurfaceView &PaintLines(const Vector<double>& x, const Vector<double>& y, const Vector<double>& z, const Color &color, double thick = Null, double lenDelta = -20);
	SurfaceView &PaintMesh(const Point3D &p0, const Point3D &p1, const Point3D &p2, const Point3D &p3, const Color &linCol);
	SurfaceView &PaintSegments(const Vector<Segment3D>& segs, const Color &color, double lenDelta = -1);
	SurfaceView &PaintSegments(const Surface &surf, const Color &linCol, double lenDelta = -1);
	SurfaceView &PaintCuboid(const Point3D &p0, const Point3D &p1, const Color &color, double lenDelta = -1);	
	SurfaceView &PaintGrid(const Point3D &p0, const Point3D &p1, const Color &color, double lenDelta = -1);
	SurfaceView &PaintAxis(double x, double y, double z, double len, double lenDelta = -1);
	SurfaceView &PaintAxis(const Point3D &p, double len, double lenDelta = -1);
	SurfaceView &PaintDoubleAxis(double x, double y, double z, double len, const Color &color, double lenDelta = -1);
	SurfaceView &PaintDoubleAxis(const Point3D &p, double len, const Color &color, double lenDelta = -1);
	SurfaceView &PaintCube(const Point3D &p, double side, const Color &color, double lenDelta = -1);
	SurfaceView &PaintCube(double x, double y, double z, double side, const Color &color, double lenDelta = -1);
	SurfaceView &PaintArrow(double x0, double y0, double z0, double x1, double y1, double z1, const Color &color, double lenDelta = -1); 
	SurfaceView &PaintArrow(const Point3D &p0, const Point3D &p1, const Color &color, double lenDelta = -1);
	SurfaceView &PaintArrow2(const Point3D &p0, const Point3D &p1, const Color &color, double lenDelta = -1);
	SurfaceView &PaintText(double x, double y, double z, const char *str, double where = Null);
		
	Image GetImage(Size sz, double scale, double dx, double dy, const Affine3d &rot);
	
	bool GetEnvelope2D();
	VolumeEnvelope env2D;
	
	SurfaceView &SetLightDir(const Point3D &p)  {
		if (!IsNull(p))
			lightDir = clone(p).Normalize(); 
		return *this;
	}
	SurfaceView &SetLightColor(Color c)			{lightColor = c; 					return *this;}
	SurfaceView &SetBackgroundColor(Color c)	{background = c; 					return *this;}
	SurfaceView &SetLineThickness(int t)		{lineThickness = float(t);			return *this;}
	
	SurfaceView &SetShowMesh(ShowMesh s)		{showMesh = s; 						return *this;}
	SurfaceView &SetShowColor(ShowColor s)		{showColor = s; 					return *this;}
	
	SurfaceView &SetDeselectIfClickOut(bool b = true){deselectIfClickOutside = b;	return *this;}
	
	SurfaceView &SetPainter(bool b)				{painter = b; 						return *this;}

	SurfaceView &SetSort(bool s)				{sort = s;							return *this;}
	
	int GetNumItems()	{return items.size();}
	int GetNumNodes()	{return nodes0.size();}
	
	void SelectPoint(Point p, Size sz, double scale, double dx, double dy, int &idBody, int &idSubBody, bool select);	
	
protected:
	void Render(const Affine3d &quat);
	
	template <class T>
	void Paint(T& w, Size sz, double scale, double dx, double dy) const;
	
	const SurfaceView &ZoomToFit(Size sz, double &scale, Pointf &pos) const;
	
private:
	Array<Vector3d> nodes0;		// Base data
	Vector<ItemView> items;
	Vector<int> order;
	bool sort = true;
	Vector<String> strings;
	
	Array<Vector3d> nodesRot;	// Rendered data (panels is sorted but internal values are not changed)
	Vector<Color> colors;
	
	VectorMap<int, Surface *> surfs;
	
	Value3D lightDir = Value3D(0, 0, -1);
	Color background, lightColor;
	ShowMesh showMesh;
	ShowColor showColor;
	bool painter = false;
	float lineThickness = 1;
	bool deselectIfClickOutside = true;
};

template <class T>
void SurfaceView::Paint(T& w, Size sz, double scl, double dx, double dy) const {
	dx += sz.cx/2;
	dy = sz.cy/2 - dy;
	
	if (painter) {
		DrawPainter im(w, sz);
		
		im.LineCap(LINECAP_SQUARE);
		im.LineJoin(LINEJOIN_MITER);
		
		im.DrawRect(sz, background);
		for (int i = 0; i < items.size(); ++i) {
			const ItemView &p = items[i];
			
			const Point3D &n0 = nodesRot[p.id[0]];
			const Point3D &n1 = nodesRot[p.id[1]];
			im.Move(n0.x*scl + dx, n0.y*scl + dy).Line(n1.x*scl + dx, n1.y*scl + dy);
			if (p.numIds > 2) {
				const Point3D &n2 = nodesRot[p.id[2]];
				im.Line(n2.x*scl + dx, n2.y*scl + dy);
				if (p.numIds == 3) 
				  im.Line(n0.x*scl + dx, n0.y*scl + dy);
				else {
					const Point3D &n3 = nodesRot[p.id[3]];
					im.Line(n3.x*scl + dx, n3.y*scl + dy).Line(n0.x*scl + dx, n0.y*scl + dy);
				}
			}
			if (showMesh == SHOW_MESH || p.numIds == 2)
				im.Stroke(p.thickness, p.meshColor);	
			else if (showMesh == SHOW_VISIBLE_MESH)
				im.Stroke(p.thickness, p.meshColor).Fill(background);
			else if (showMesh == SHOW_FACES)
				im.Stroke(p.thickness, colors[i]).Fill(colors[i]);
			else if (showMesh == SHOW_MESH_FACES)
				im.Stroke(p.thickness, p.meshColor).Fill(colors[i]);
		}
	} else {
		w.DrawRect(sz, background);
		
		auto DoPaint = [&](const ItemView &p, int ip) {
			if (p.numIds == 1) {
				if (p.idString >= 0) {
					const Point3D &n0 = nodesRot[p.id[0]];
					if (IsNull(n0))
						return;
			        RichText txt = ParseQTF(strings[p.idString]);
			        PaintInfo pi;
					txt.ApplyZoom(GetRichTextStdScreenZoom());
					pi.darktheme = IsDarkTheme();
			        txt.Paint(w, int(n0.x*scl + dx), int(n0.y*scl + dy), 200, pi);
				}
			} else if (p.numIds == 2) {
				const Point3D &n0 = nodesRot[p.id[0]];
				const Point3D &n1 = nodesRot[p.id[1]];
				if (IsNull(n0) || IsNull(n1))
					return;
				w.DrawLine(int(n0.x*scl + dx), int(n0.y*scl + dy), int(n1.x*scl + dx), int(n1.y*scl + dy), (int)p.thickness, p.meshColor);
			} else {
				Vector<Point> pi;
				for (unsigned i = 0; i < p.numIds; ++i) {
					const Point3D &n = nodesRot[p.id[i]];
					if (IsNull(n))
						return;
					pi << Point(int(n.x*scl + dx), int(n.y*scl + dy));
				}
				if (showMesh == SHOW_MESH) {
					pi << pi[0];	// Close the figure
					w.DrawPolyline(pi, (int)p.thickness, p.meshColor);	
				} else if (showMesh == SHOW_VISIBLE_MESH) {
					pi << pi[0];
					w.DrawPolygon(pi, background, (int)p.thickness, p.meshColor);	
				} else if (showMesh == SHOW_FACES)
					w.DrawPolygon(pi, colors[ip]);	
				else if (showMesh == SHOW_MESH_FACES)
					w.DrawPolygon(pi, colors[ip], (int)p.thickness, p.meshColor);
			}
		};
		
		if (sort) {
			for (int io = 0; io < order.size(); ++io) {
				int ip = order[io];
				DoPaint(items[ip], ip);
			}
		} else {
			for (int io = 0; io < items.size(); ++io) 
				DoPaint(items[io], io);
		}
	}       
}	


}
	
#endif
