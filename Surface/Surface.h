// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2022, the Anboto author and contributors
#ifndef _GLCanvas_surface_h_
#define _GLCanvas_surface_h_

#include <Eigen/Eigen.h>
#include <Functions4U/Functions4U.h>

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
	

class Value3D : public Moveable<Value3D> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	double x, y, z;
	
	Value3D() 									{}
	Value3D(const Nuller&) 						{SetNull();}
	Value3D(const Value3D &p) 					{Set(p);}
	Value3D(const Vector3d &p) 					{Set(p);}
	Value3D(double _x, double _y, double _z) 	{Set(_x, _y, _z);}
	
	void SetNull() 				{x = Null; y = 0;}
	bool IsNullInstance() const	{return IsNull(x);}
	
	void Zero() 			{x = y = z = 0;}
	
	static int size() 		{return 3;}
	
	Value3D(bool positive)	{x = Null; y = positive ? 1 : -1;}
	bool IsPosInf()			{return IsNull(x) && y == 1;}
	bool IsNegInf()			{return IsNull(x) && y == -1;}
	
	void Set(const Value3D &p) 					{x = p.x;	y = p.y;	z = p.z;}
	void Set(const Vector3d &p) 				{x = p(0);	y = p(1);	z = p(2);}
	void Set(double _x, double _y, double _z) 	{x = _x;  	y = _y;  	z = _z;}
	
	inline Value3D operator=(const Value3D &p)	{Set(p);	return *this;}
	inline Value3D operator=(const Vector3d &p)	{Set(p);	return *this;}
	
    operator Eigen::Vector3d() const {return Eigen::Vector3d(x, y, z);}
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
	void TransRot(double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz);
	void Rotate(double ax, double ay, double az, double cx, double cy, double cz);	
		
	// Dot product or scalar product
	inline double dot(const Value3D& a) const {return x*a.x + y*a.y + z*a.z;}
	
	// Cross product or vector product X (or wedge product ∧ in 3D) 
	inline friend Value3D operator%(const Value3D& a, const Value3D& b) {return Value3D(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);}
	
	inline friend Value3D operator+(const Value3D& a, const Value3D& b) {return Value3D(a.x+b.x, a.y+b.y, a.z+b.z);}
	inline friend Value3D operator-(const Value3D& a, const Value3D& b) {return Value3D(a.x-b.x, a.y-b.y, a.z-b.z);}
	inline friend Value3D operator*(const Value3D& a, double b) 		{return Value3D(a.x*b, a.y*b, a.z*b);}
	inline friend Value3D operator/(const Value3D& a, double b) 		{return Value3D(a.x/b, a.y/b, a.z/b);}

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
	Value3D &Normalize() {
		double length = Length();
		
		if (length < 1e-10) 
			Zero();
		else {
		    x = x/length;
		    y = y/length;
		    z = z/length;
		}
		return *this;
	}
	double Manhattan() const {return abs(x) + abs(y) + abs(z);}

	double Angle(const Value3D &p) const {return acos(dot(p)/(Length()*p.Length()));}
	
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
	void Jsonize(JsonIO &json) {
		json
			("x", x)
			("y", y)
			("z", z)
		;
	}
};

void TransRot(const Value3D &pos, const Value3D &ref, const VectorXd &transf, Value3D &npos);
void TransRot(const Value3D &pos, const Value3D &ref, double x, double y, double z, double rx, double ry, double rz, Value3D &npos);
void TransRot(const Value3D &pos, const Value3D &ref, VectorXd &x, VectorXd &y, VectorXd &z, VectorXd &rx, VectorXd &ry, VectorXd &rz);

bool TransRotChangeRef(const Value3D &ref, const VectorXd &transf, const Value3D &nref, VectorXd &ntransf);
	
typedef Value3D Direction3D;
typedef Value3D Point3D;

void GetTransform(Affine3d &aff, double ax, double ay, double az, double cx, double cy, double cz);
void GetTransform(Affine3d &aff, double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz);	
void GetTransform000(Affine3d &aff, double dx, double dy, double dz, double ax, double ay, double az);

void TransRot(const Affine3d &aff, const Value3D &pos, Value3D &npos);
	
class Value6D {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Value6D() {}
	Value6D(const Value6D &f)	{Set(f);}
	Value6D(const VectorXd &v)	{Set(v);}
	template<typename T>
	Value6D(const T *v)			{Set(v);}
	Value6D(double v0, double v1, double v2, double v3, double v4, double v5) {Set(v0, v1, v2, v3, v4, v5);}
	
	void Set(const Value6D &f)	{t.x = f.t.x;	t.y = f.t.y;	t.z = f.t.z;	r.x = f.r.x;	r.y = f.r.y;	r.z = f.r.z;}
	void Set(const VectorXd &v) {
		ASSERT(v.size() == 6);
		Set(v.data());
	}
	template<typename T>
	void Set(const T *v) {
		t.x = v[0];	t.y = v[1];	t.z = v[2];	r.x = v[3];	r.y = v[4];	r.z = v[5];
	}
	template<typename T>
	void Add(const T *v) {
		t.x+= v[0];	t.y+= v[1];	t.z+= v[2];	r.x+= v[3];	r.y+= v[4];	r.z+= v[5];
	}
	void Add(const VectorXd &v) {
		ASSERT(v.size() == 6);
		Add(v.data());
	}
	void Set(double v0, double v1, double v2, double v3, double v4, double v5) {
		t.x = v0;	t.y = v1;	t.z = v2;	r.x = v3;	r.y = v4;	r.z = v5;
	}
	void Zero() {t.Zero();	r.Zero();}
	
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
	
	Value3D t, r;
};

class ForceVector;

class Force6D : public Value6D {
public:
	Force6D() {}
	Force6D(const VectorXd &v) : Value6D(v) {}
	Force6D(double v0, double v1, double v2, double v3, double v4, double v5) : Value6D(v0, v1, v2, v3, v4, v5) {}
	
	void AddLinear(const Direction3D &dir, const Point3D &point, const Point3D &c0);	
	void Add(const Force6D &force, const Point3D &point, const Point3D &c0);
	void Add(const ForceVector &force, const Point3D &c0);
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
class Velocity6D : public Value6D {
public:
	template<typename T>
	Velocity6D(const T *v) {Set(v);}
	
	void Translate(const Point3D &from, const Point3D &to) {
		Direction3D rpq = to - from;
		Translate(rpq);
	}
	void Translate(const Direction3D &rpq) {
		t += r%rpq;
	}
};

class Acceleration6D : public Value6D {
public:
	template<typename T>
	Acceleration6D(const T *v) {Set(v);}
	
	void Translate(const Point3D &from, const Point3D &to, const Velocity6D &vel) {
		Direction3D rpq = to - from;
		Translate(rpq, vel);
	}
	void Translate(const Direction3D &rpq, const Velocity6D &vel) {
		t += r%rpq + vel.r%(vel.r%rpq);
	}
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
Direction3D Normal(const Value3D &a, const Value3D &b, const Value3D &c);
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
	Segment3D(const Point3D &_from, const Direction3D &normal, double length) : from(_from) {
		to = Point3D(from.x + length*normal.x, from.y + length*normal.y, from.z + length*normal.z);
	}
	void SetNull() 				{from = Null;}
	bool IsNullInstance() const	{return IsNull(from) || IsNull(to);}
		
	void Set(const Point3D &_from, const Point3D &_to) {
		from.Set(_from);
		to.Set(_to);
	}
	void Set(const Point3D &_from, const Direction3D &normal, double length) {
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
	
	Direction3D Direction() {return Direction3D(to - from);}
	
	Point3D IntersectionPlaneX(double x);
	Point3D IntersectionPlaneY(double y);
	Point3D IntersectionPlaneZ(double z);
	
	Point3D Intersection(const Point3D &planePoint, const Direction3D &planeNormal);
	
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

Point3D Intersection(const Direction3D &lineVector, const Point3D &linePoint, const Direction3D &planeNormal, const Point3D &planePoint);

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

class Panel : public MoveableAndDeepCopyOption<Panel> {
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
};

class LineSegment : public MoveableAndDeepCopyOption<LineSegment> {
public:
	LineSegment() {}
	LineSegment(const LineSegment &orig, int) {
		inode0 = orig.inode0;
		inode1 = orig.inode1;
		panels = clone(orig.panels);
	}
	int inode0, inode1;
	Upp::Index<int> panels;
};

class VolumeEnvelope : MoveableAndDeepCopyOption<VolumeEnvelope> {
public:
	VolumeEnvelope() {Reset();}
	void Reset() 	 {maxX = minX = maxY = minY = maxZ = minZ = Null;}
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
	double Max()	{return max(max(max(abs(maxX), abs(minX)), max(abs(maxY), abs(minY))), max(abs(maxZ), abs(minZ)));}
	double LenRef()	{return max(max(maxX - minX, maxY - minY), maxZ - minZ);}
	
	double maxX, minX, maxY, minY, maxZ, minZ;
};

class Surface : DeepCopyOption<Surface> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Surface() {}
	Surface(const Surface &surf, int);
		
	void Clear();
	bool IsEmpty() const;
	
	void Load(String fileName);
	void Save(String fileName);

	Vector<Point3D> nodes;
	Vector<Panel> panels;
	Vector<LineSegment> segments;
	Array<Vector<Point3D>> lines;
	
	int GetNumNodes() const		{return nodes.size();}
	int GetNumPanels() const	{return panels.size();}
	int GetNumSegments() const	{return segments.size();}
	
	Vector<Segment3D> skewed;
	Vector<Segment3D> segWaterlevel, segTo1panel, segTo3panel;
	
	VolumeEnvelope env;
	
	void AddLine(const Vector<Point3D> &points3D);
	void AddLine(const Vector<Pointf> &points);
		
	String Heal(bool basic, double grid, double eps, Function <bool(String, int pos)> Status = Null);
	Surface &Orient();
	Surface &OrientFlat();
	void Image(int axis);
	const VolumeEnvelope &GetEnvelope(); 
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
	void GetInertia33_Volume(Matrix3d &inertia, const Point3D &cg, bool refine = false) const;
	void GetInertia33_Surface(Matrix3d &inertia, const Point3D &cg, bool refine = false) const;
	void GetInertia33(Matrix3d &inertia, const Point3D &cg, bool volume, bool refine = false) const;
	void GetInertia66(MatrixXd &inertia, const Matrix3d &inertia33, const Point3D &cg, const Point3D &c0, bool refine) const;
	static void GetInertia33_Radii(Matrix3d &inertia);
	void GetInertia33_Radii(Matrix3d &inertia, const Point3D &c0, bool volume, bool refine) const;
	static void FillInertia66mc(MatrixXd &inertia, const Point3D &cg, const Point3D &c0);
		
	static void TranslateInertia33(Matrix3d &inertia, double m, const Point3D &cg, const Point3D &c0, const Point3D &nc0);
	static void TranslateInertia66(MatrixXd &inertia, const Point3D &cg, const Point3D &c0, const Point3D &nc0);
	Force6D GetHydrostaticForce(const Point3D &c0, double rho, double g) const;
	Force6D GetHydrostaticForceNormalized(const Point3D &c0) const;
	Force6D GetHydrostaticForceCB(const Point3D &c0, const Point3D &cb, double rho, double g) const;
	Force6D GetHydrostaticForceCBNormalized(const Point3D &c0, const Point3D &cb) const;
	static Force6D GetMassForce(const Point3D &c0, const Point3D &cg, const double mass, const double g);
	void GetHydrostaticStiffness(MatrixXd &c, const Point3D &c0, const Point3D &cg, 
				const Point3D &cb, double rho, double g, double mass, bool massBuoy);
	Force6D GetHydrodynamicForce(const Point3D &c0, bool clip, Function<double(double x, double y)> GetZSurf,
						Function<double(double x, double y, double z, double et)> GetPress) const;
	double GetWaterPlaneArea() const;
	
	void AddWaterSurface(Surface &surf, const Surface &under, char c, double grid = Null, double eps = Null);
	static Vector<Segment3D> GetWaterLineSegments(const Surface &orig);
	bool GetDryPanels(const Surface &surf, bool onlywaterplane, double grid, double eps);
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
		
	Surface &Translate(double dx, double dy, double dz);
	Surface &Rotate(double ax, double ay, double az, double _c_x, double _c_y, double _c_z);
	Surface &TransRot(double dx, double dy, double dz, double ax, double ay, double az, double _c_x, double _c_y, double _c_z);
	
	bool TranslateArchimede(double mass, double rho, double &dz, Surface &under);
	bool Archimede(double mass, Point3D &cg, const Point3D &c0, double rho, double g, double &dz, double &droll, double &dpitch, Surface &under);
	
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
	
	void AddNode(const Point3D &p);
	int FindNode(const Point3D &p);
	
	void AddFlatRectangle(double lenX, double lenY, double panelWidth, double panelHeight);
	void AddRevolution(const Vector<Pointf> &points, double panelWidth);
	void AddPolygonalPanel(const Vector<Pointf> &bound, double panelWidth, bool adjustSize);
	//void AddPolygonalPanel2(const Vector<Pointf> &bound, double panelWidth, bool adjustSize);
	
	static void RoundClosest(Vector<Point3D> &_nodes, double grid, double eps);	
	static int RemoveDuplicatedPanels(Vector<Panel> &_panels);
	static int RemoveTinyPanels(Vector<Panel> &_panels);
	static int RemoveDuplicatedPointsAndRenumber(Vector<Panel> &_panels, Vector<Point3D> &_nodes);
	static void DetectTriBiP(Vector<Panel> &panels) {int dum;	DetectTriBiP(panels, dum, dum, dum);}
	
	//inline const Point3D &GetPos() const 	{return pos;}
	//inline const Point3D &GetAngle() const	{return angle;}
	
	void GetClosestPanels(int idPanel, UVector<int> &panIDs);
	
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
	
	Vector<int> selPanels, selNodes;
	
	//Point3D pos, angle;
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

class SurfaceX : DeepCopyOption<SurfaceX> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	SurfaceX() {}
	SurfaceX(const SurfaceX &surf, int) {}
	
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
	
	static void GetTransform(Affine3d &aff, double dx, double dy, double dz, double ax, double ay, double az);
	static void TransRot(double &x, double &y, double &z, double x0, double y0, double z0, const Affine3d &quat);
	static void TransRot(double &x, double &y, double &z, double x0, double y0, double z0,
						 double dx, double dy, double dz, double ax, double ay, double az);
	
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
	
enum ContainsPointRes {POLY_NOPLAN = -4, POLY_FAR = -3, POLY_3 = -2, POLY_OUT = -1, POLY_SECT = 0, POLY_IN = 1};
ContainsPointRes ContainsPoint(const Vector<Point3D> &polygon, const Point3D &point, double distanceTol, double angleNormalTol);
ContainsPointRes ContainsPoint(const Vector<Pointf>& polygon, const Pointf &pt);

Point3D Centroid(const UVector<Point3D> &p);
double Area(const UVector<Point3D> &p);

Pointf Centroid(const UVector<Pointf> &p);
double Area(const UVector<Pointf> &p);

Vector<Pointf>  Point3Dto2D_XY(const Vector<Point3D> &bound);
Vector<Point3D> Point2Dto3D_XY(const Vector<Pointf>  &bound);
Vector<Pointf>  Point3Dto2D_XZ(const Vector<Point3D> &bound);
Vector<Point3D> Point2Dto3D_XZ(const Vector<Pointf>  &bound);
Vector<Pointf>  Point3Dto2D_YZ(const Vector<Point3D> &bound);
Vector<Point3D> Point2Dto3D_YZ(const Vector<Pointf>  &bound);

Vector<Point3D> GetClosedPolygons(Vector<Segment3D> &segs);

bool PointInPoly(const UVector<Pointf> &xy, const Pointf &pxy);
	
}
	
#endif
