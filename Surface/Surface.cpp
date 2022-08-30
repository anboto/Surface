// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2022, the Anboto author and contributors
#include <Core/Core.h>

#include <Eigen/Eigen.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>
#include <Functions4U/Functions4U.h>
#include <numeric> 
#include <STEM4U/Rootfinding.h>

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

Direction3D Normal(const Value3D &a, const Value3D &b, const Value3D &c) {
	return Direction3D((a - b) % (b - c)).Normalize();
}

double Area(const Value3D &p0, const Value3D &p1, const Value3D &p2) {
	double l01 = Distance(p0, p1);
	double l12 = Distance(p1, p2);
	double l02 = Distance(p0, p2);

	double s = (l01 + l12 + l02)/2;
	return sqrt(max(s*(s - l01)*(s - l12)*(s - l02), 0.)); 
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

void Point3D::Translate(double dx, double dy, double dz) {
	x += dx;
	y += dy;
	z += dz;
}

void Point3D::Rotate(double ax, double ay, double az, double cx, double cy, double cz) {
	Affine3d aff;
	GetTransform(aff, ax, ay, az, cx, cy, cz);
	TransRot(aff);
}

void Point3D::TransRot(double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz) {
	Affine3d aff;
	GetTransform(aff, dx, dy, dz, ax, ay, az, cx, cy, cz);
	TransRot(aff);
}

void Point3D::TransRot(const Affine3d &quat) {
	Vector3d pnt0(x, y, z);	
	Vector3d pnt = quat * pnt0;

	x = pnt[0];
	y = pnt[1];
	z = pnt[2];
}

void TransRot(const Point3D &pos, const Point3D &ref, VectorXd &x, VectorXd &y, VectorXd &z, VectorXd &rx, VectorXd &ry, VectorXd &rz) {
	ASSERT((x.size() == y.size()) && (y.size() == z.size()) && (z.size() == rx.size()) && (rx.size() == ry.size()) && (ry.size() == rz.size()));
	for (int i = 0; i < x.size(); ++i) {
		Point3D ps = clone(pos);
		ps.TransRot(x[i], y[i], z[i], rx[i], ry[i], rz[i], ref.x, ref.y, ref.z);
		x[i] = ps.x;
		y[i] = ps.y;
		z[i] = ps.z;
	}
}

void GetTransform(Affine3d &aff, double ax, double ay, double az, double cx, double cy, double cz) {
	Vector3d c(cx, cy, cz);	
	aff =	Translation3d(c) *
			AngleAxisd(ax, Vector3d::UnitX()) *
		 	AngleAxisd(ay, Vector3d::UnitY()) *
		 	AngleAxisd(az, Vector3d::UnitZ()) *
		 	Translation3d(-c);
}

void GetTransform(Affine3d &aff, double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz) {
	Vector3d d(dx, dy, dz), c(cx, cy, cz);	
	aff =	Translation3d(d) *
			Translation3d(c) *
			AngleAxisd(ax, Vector3d::UnitX()) *
		 	AngleAxisd(ay, Vector3d::UnitY()) *
		 	AngleAxisd(az, Vector3d::UnitZ()) *
		 	Translation3d(-c);
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


bool PointInSegment(const Point3D &p, const Segment3D &seg) {
	double dpa = Distance(p, seg.from);
	double dpb = Distance(p, seg.to);
	double dab = seg.Length();
	
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

void Surface::Clear() {
	nodes.Clear();
	panels.Clear();
	skewed.Clear();
	segWaterlevel.Clear();
	segTo1panel.Clear();
	segTo3panel.Clear();
	segments.Clear();
	selPanels.Clear();
	selNodes.Clear();
}

Surface::Surface(const Surface &orig, int) {
	healing = orig.healing;
	numTriangles = orig.numTriangles;
	numBiQuads = orig.numBiQuads;
	numMonoQuads = orig.numMonoQuads;
	
	panels = clone(orig.panels);
	nodes = clone(orig.nodes);
	skewed = clone(orig.skewed);
	segWaterlevel = clone(orig.segWaterlevel);
	segTo1panel = clone(orig.segTo1panel);
	segTo3panel = clone(orig.segTo3panel);
	segments = clone(orig.segments);
	
	env = clone(orig.env);
	
	surface = orig.surface;
	volume = orig.volume;
	volumex = orig.volumex;
	volumey = orig.volumey;
	volumez = orig.volumez;
	
	//pos = clone(orig.pos);
	//angle = clone(orig.angle);
	
	avgFacetSideLen = orig.avgFacetSideLen;
}

bool Surface::IsEmpty() const {
	return nodes.IsEmpty();
}

bool Surface::FixSkewed(int ipanel) {
	Panel &pan = panels[ipanel];
	
	int &id0 = pan.id[0];
	int &id1 = pan.id[1];
	int &id2 = pan.id[2];
	int &id3 = pan.id[3];
	Point3D &p0 = nodes[id0];
	Point3D &p1 = nodes[id1];
	Point3D &p2 = nodes[id2];
	Point3D &p3 = nodes[id3];
	if (id0 != id3) {		// Is not triangular 
		Direction3D normal301 = Normal(p3, p0, p1);
		Direction3D normal012 = Normal(p0, p1, p2);
		Direction3D normal123 = Normal(p1, p2, p3);
		Direction3D normal230 = Normal(p2, p3, p0);
		double d0  = normal301.Manhattan();
		double d01 = Manhattan(normal301, normal012);
		double d02 = Manhattan(normal301, normal123);
		double d03 = Manhattan(normal301, normal230);
		
		int numg = 0;
		if (d0 < d01)
			numg++;
		if (d0 < d02)
			numg++;
		if (d0 < d03)
			numg++;	 
		if (numg > 1) {
			skewed << Segment3D(p0, p1) << Segment3D(p1, p2) << Segment3D(p2, p3) << Segment3D(p3, p0);
			
			if (d0 < d01)
				Swap(pan.id[1], pan.id[2]);
			else if (d0 < d02)
				Swap(pan.id[2], pan.id[3]);
			return true;
		}
	}
	return false;
}

int Surface::FixSkewed() {	
	int num = 0;
	for (int i = 0; i < panels.GetCount(); ++i) 
		if (FixSkewed(i))
			num++;
	return num;
}

void Surface::DetectTriBiP(Vector<Panel> &panels, int &numTri, int &numBi, int &numP) {
	numTri = numBi = numP = 0;
	for (int i = panels.GetCount()-1; i >= 0; --i) {
		Panel &panel = panels[i];
		Upp::Index<int> ids;
		ids.FindAdd(panel.id[0]);
		ids.FindAdd(panel.id[1]);
		ids.FindAdd(panel.id[2]);
		ids.FindAdd(panel.id[3]);
		if (ids.GetCount() == 4)
			;
		else if (ids.GetCount() == 3) {
			numTri++;
			panel.id[0] = ids[0];
			panel.id[1] = ids[1];
			panel.id[2] = ids[2];	
			panel.id[3] = ids[0];	
		} else if (ids.GetCount() == 2) {
			numBi++;
			panels.Remove(i, 1);
		} else {
			numP++;
			panels.Remove(i, 1);
		}
	}
}

void Surface::TriangleToQuad(Panel &pan) {
	panels << pan;
	TriangleToQuad(panels.size() - 1);
}

void Surface::TriangleToQuad(int ipanel) {
	Panel &pan00 = panels[ipanel];
	int id0 = pan00.id[0];
	int id1 = pan00.id[1];
	int id2 = pan00.id[2];
	Point3D &p0 = nodes[id0];
	Point3D &p1 = nodes[id1];
	Point3D &p2 = nodes[id2];
		
	Point3D p012= Centroid(p0, p1, p2);	nodes.Add(p012);	int id012= nodes.size()-1;
	Point3D p01 = Middle(p0, p1);		nodes.Add(p01);		int id01 = nodes.size()-1;
	Point3D p12 = Middle(p1, p2);		nodes.Add(p12);		int id12 = nodes.size()-1;
	Point3D p20 = Middle(p2, p0);		nodes.Add(p20);		int id20 = nodes.size()-1;
	
	panels.Remove(ipanel, 1);
	Panel &pan0 = panels.Add();	pan0.id[0] = id0;	pan0.id[1] = id01;	pan0.id[2] = id012;	pan0.id[3] = id20;
	Panel &pan1 = panels.Add();	pan1.id[0] = id01;	pan1.id[1] = id1;	pan1.id[2] = id12;	pan1.id[3] = id012;
	Panel &pan2 = panels.Add();	pan2.id[0] = id012;	pan2.id[1] = id12;	pan2.id[2] = id2;	pan2.id[3] = id20;
}

int Surface::RemoveDuplicatedPanels(Vector<Panel> &_panels) {		
	int num = 0;
	for (int i = 0; i < _panels.size()-1; ++i) {
		Panel &panel = _panels[i];
		for (int j = _panels.size()-1; j >= i+1; --j) {
			if (panel == _panels[j]) {
				num++;
				_panels.Remove(j, 1);
			}
		}
	}
	return num;
}

int Surface::RemoveTinyPanels(Vector<Panel> &_panels) {		
	int num = 0;
	double avgsurface = 0;
	for (int i = 0; i < _panels.GetCount(); ++i) 
		avgsurface += _panels[i].surface0 + _panels[i].surface1;
	avgsurface /= _panels.GetCount();
	double tiny = avgsurface/1000000;
			
	for (int i = _panels.GetCount()-1; i >= 0; --i) {
		double surface = _panels[i].surface0 + _panels[i].surface1;
		if (surface < tiny) {
			num++;
			_panels.Remove(i, 1);
		}
	}
	return num;
}

int Surface::RemoveDuplicatedPointsAndRenumber(Vector<Panel> &_panels, Vector<Point3D> &_nodes) {
	int num = 0;
	
	// Detect duplicated points in nodes
	double similThres = EPS_LEN;
	Upp::Index<int> duplic, goods;
	for (int i = 0; i < _nodes.GetCount()-1; ++i) {
		if (duplic.Find(i) >= 0)
			continue;
		for (int j = i+1; j < _nodes.GetCount(); ++j) {
			if (_nodes[i].IsSimilar(_nodes[j], similThres)) {
				duplic << j;
				goods << i;
				num++;
			}
		}
	}
	
	// Replace duplicated points with good ones in panels
	for (int i = 0; i < _panels.GetCount(); ++i) {
		for (int j = 0; j < 4; ++j) {
			int &id = _panels[i].id[j];
			int pos = duplic.Find(id);
			if (pos >= 0)
				id = goods[pos];
		}
	}
	
	// Find unused nodes
	Vector<int> newId;
	newId.SetCount(_nodes.GetCount());
	int avId = 0;
	for (int i = 0; i < _nodes.GetCount(); ++i) {
		bool found = false;
		for (int ip = 0; ip < _panels.GetCount() && !found; ++ip) {
			int numP = PanelGetNumNodes(_panels, ip);
			for (int j = 0; j < numP; ++j) {
				if (_panels[ip].id[j] == i) {
					found = true;
					break;
				}
			}
		}
		if (!found)
			newId[i] = Null;	// Remove unused nodes
		else if (duplic.Find(i) >= 0)
			newId[i] = Null;	
 		else {	
			newId[i] = avId;
			avId++;
		} 
	}
	
	// Remove duplicated nodes
	for (int i = _nodes.GetCount()-1; i >= 0; --i) {
		if (IsNull(newId[i]))
			_nodes.Remove(i, 1);
	}
	
	// Renumber panels
	for (int i = 0; i < _panels.GetCount(); ++i) {
		for (int j = 0; j < 4; ++j) {
			int& id = _panels[i].id[j];
			id = newId[id];
		}
	}
	return num;
}
	
void Surface::AddSegment(int inode0, int inode1, int ipanel) {
	ASSERT(!IsNull(inode0));
	ASSERT(!IsNull(inode1));
	for (int i = 0; i < segments.GetCount(); ++i) {
		if ((segments[i].inode0 == inode0 && segments[i].inode1 == inode1) ||
			(segments[i].inode1 == inode0 && segments[i].inode0 == inode1)) {
			segments[i].panels << ipanel;
			return;
		}
	}
	Segment &sg = segments.Add();
	sg.inode0 = inode0;
	sg.inode1 = inode1;
	sg.panels << ipanel;
}

int Surface::SegmentInSegments(int iseg) const {
	Segment3D seg(nodes[segments[iseg].inode0], nodes[segments[iseg].inode1]);
	double lenSeg = seg.Length();
			
	for (int i = 0; i < segments.GetCount(); ++i) {
		if (i != iseg) {
			const Segment &segment = segments[i];
			Segment3D is(nodes[segment.inode0], nodes[segment.inode1]);
			if (is.SegmentIn(seg, lenSeg))		
				return i;
			if (seg.SegmentIn(is))
				return i;
		}
	}
	return -1;
}

void Surface::GetSegments() {
	segments.Clear();
		
	for (int i = 0; i < panels.GetCount(); ++i) {
		int id0 = panels[i].id[0];
		int id1 = panels[i].id[1];
		int id2 = panels[i].id[2];
		int id3 = panels[i].id[3];
		AddSegment(id0, id1, i);
		AddSegment(id1, id2, i);
		if (IsPanelTriangle(i)) 
			AddSegment(id2, id0, i);
		else {
			AddSegment(id2, id3, i);
			AddSegment(id3, id0, i);
		}
	}
	avgLenSegment = 0;
	for (const auto &s : segments)
		avgLenSegment += Distance(nodes[s.inode0], nodes[s.inode1]);
	avgLenSegment /= segments.GetCount();
}
	
void Surface::AnalyseSegments(double zTolerance) {
	GetSegments();
	
	for (int i = 0; i < segments.GetCount(); ++i) {
		int inode0 = segments[i].inode0;
		int inode1 = segments[i].inode1;
		
		if (inode0 >= nodes.GetCount())
			throw Exc(Format(t_("Node %d is pointing out of scope"), inode0+1));	
		if (inode1 >= nodes.GetCount())
			throw Exc(Format(t_("Node %d is pointing out of scope"), inode1+1));
		
		int num = segments[i].panels.GetCount();
				
		if (num == 1) {
			if (nodes[inode0].z >= zTolerance && nodes[inode1].z >= zTolerance)
				segWaterlevel << Segment3D(nodes[inode0], nodes[inode1]);
			else {
				if (SegmentInSegments(i) < 0)	
					segTo1panel << Segment3D(nodes[inode0], nodes[inode1]);
			}
		} else if (num > 2)
			segTo3panel << Segment3D(nodes[inode0], nodes[inode1]);
	}
}

bool Surface::GetLowest(int &iLowSeg, int &iLowPanel) {	// Get the lowest panel with normal non horizontal
	iLowSeg = iLowPanel = Null;
	double zLowSeg = DBL_MAX;
	for (int i = 0; i < segments.GetCount(); ++i) {
		const Segment &seg = segments[i];
		if (seg.panels.GetCount() == 2) {
			for (int ip = 0; ip < seg.panels.GetCount(); ++ip) {
				if (panels[seg.panels[ip]].normal0.z != 0) {
					double zz = max(nodes[seg.inode0].z, nodes[seg.inode1].z);
					if (zz < zLowSeg) {
						zLowSeg = zz;
						iLowSeg = i;
						iLowPanel = ip;
					}
				}
			}
		}
	}
	if (!IsNull(iLowSeg))
		return true;
	for (int i = 0; i < segments.GetCount(); ++i) {
		const Segment &seg = segments[i];
		if (seg.panels.GetCount() == 2) {
			for (int ip = 0; ip < seg.panels.GetCount(); ++ip) {
				double zz = min(nodes[seg.inode0].z, nodes[seg.inode1].z);
				if (zz < zLowSeg) {
					zLowSeg = zz;
					iLowSeg = i;
					iLowPanel = ip;
				}
			}
		}
	}
	return !IsNull(iLowSeg);
}
	
bool Surface::ReorientPanels0(bool _side) {
	numUnprocessed = -1;
	
	int iLowSeg, iLowPanel;
	if (!GetLowest(iLowSeg, iLowPanel))
		return false;
	
	// Reorient lowest panel downwards to be the seed
	int ip = segments[iLowSeg].panels[iLowPanel];
	if (panels[ip].normal0.z != 0) {
		if (_side && panels[ip].normal0.z > 0 || !_side && panels[ip].normal0.z < 0)
			ReorientPanel(ip);
	} else if (panels[ip].normal0.x != 0) {
		if (_side && panels[ip].normal0.x > 0 || !_side && panels[ip].normal0.x < 0)
			ReorientPanel(ip);
	} else {
		if (_side && panels[ip].normal0.y > 0 || !_side && panels[ip].normal0.y < 0)
			ReorientPanel(ip);
	}
	
	Vector<int> panelStack;
	Upp::Index<int> panelProcessed;
	
	panelStack << ip;
	while (!panelStack.IsEmpty()) {
		int id = panelStack.GetCount() - 1;
		int ipp = panelStack[id];
		panelStack.Remove(id, 1);
		panelProcessed << ipp;
		
		for (int is = 0; is < segments.GetCount(); ++is) {
			const Upp::Index<int> &segPanels = segments[is].panels;
			if (segPanels.Find(ipp) >= 0) {
				for (int i = 0; i < segPanels.GetCount(); ++i) {
					int ipadyac = segPanels[i];
					if (ipadyac != ipp && panelProcessed.Find(ipadyac) < 0) {
						panelStack << ipadyac;
						if (!SameOrderPanel(ipp, ipadyac, segments[is].inode0, segments[is].inode1))
							ReorientPanel(ipadyac);
					}
				}
			}
		}
	}
	
	numUnprocessed = panels.GetCount() - panelProcessed.GetCount();

	return true;
}

Vector<Vector<int>> Surface::GetPanelSets(Function <bool(String, int pos)> Status) {
	Vector<Vector<int>> ret;
	
	double zTolerance = -0.1;
	AnalyseSegments(zTolerance);	
	
	Index<int> allPanels;
	for (int i = 0; i < panels.GetCount(); ++i)
		allPanels << i;
	
	while (allPanels.GetCount() > 0) {
		Vector<int> panelStack;
		Upp::Index<int> panelProcessed;
	
		panelStack << allPanels[0];
		while (!panelStack.IsEmpty()) {
			int id = panelStack.GetCount() - 1;
			int ipp = panelStack[id];
			panelStack.Remove(id, 1);
			int iall = allPanels.Find(ipp);
			if (iall < 0)
				continue;
			allPanels.Remove(iall);
			panelProcessed << ipp;
			
			for (int is = 0; is < segments.GetCount(); ++is) {
				const Upp::Index<int> &segPanels = segments[is].panels;
				if (segPanels.Find(ipp) >= 0) {
					for (int i = 0; i < segPanels.GetCount(); ++i) {
						int ipadyac = segPanels[i];
						if (ipadyac != ipp && panelProcessed.Find(ipadyac) < 0) 
							panelStack << ipadyac;
					}
				}
			}
		}
		ret << panelProcessed.PickKeys();
		panelProcessed.Clear();
		Status(Format(t_("Split mesh #%d"), ret.GetCount()), 0);
	}
	return ret;
}

void Surface::ReorientPanel(int ip) {
	panels[ip].Swap();
	panels[ip].normal0.Mirror();
	if (panels[ip].IsTriangle()) 
		panels[ip].normal1.Mirror();
}

bool Panel::FirstNodeIs0(int in0, int in1) const {
	if (IsTriangle()) {
		if ((id[0] == in0 && id[1] == in1) ||
			(id[1] == in0 && id[2] == in1) ||
			(id[2] == in0 && id[0] == in1))
			return true;
		else
			return false;
	} else {
		if ((id[0] == in0 && id[1] == in1) ||
			(id[1] == in0 && id[2] == in1) ||
			(id[2] == in0 && id[3] == in1) ||
			(id[3] == in0 && id[0] == in1))
			return true;
		else
			return false;
	}
}

void Panel::RedirectTriangles() {
	int shift = 0;
	if (id[0] == id[1])
		shift = -1;
	else if (id[1] == id[2])
		shift = -2;
	else if (id[2] == id[3])
		shift = 1;
	else
		return;
	ShiftNodes(shift);
}

void Panel::ShiftNodes(int shift) {
	int id_0 = id[0];
	int id_1 = id[1];
	int id_2 = id[2];
	int id_3 = id[3];
	if (shift == 1) {
		id[1] = id_0;
		id[2] = id_1;
		id[3] = id_2;
		id[0] = id_3;
	} else if (shift == -1) { 
		id[0] = id_1;
		id[1] = id_2;
		id[2] = id_3;
		id[3] = id_0;
	} else if (shift == -2) { 
		id[0] = id_2;
		id[1] = id_3;
		id[2] = id_0;
		id[3] = id_1;
	} else
		throw t_("ShiftNodes value not implemented");
}

bool Surface::SameOrderPanel(int ip0, int ip1, int in0, int in1) {
	bool first0in0 = panels[ip0].FirstNodeIs0(in0, in1);
	bool first1in0 = panels[ip1].FirstNodeIs0(in0, in1);
	
	return first0in0 != first1in0;
}

String Surface::Heal(bool basic, Function <bool(String, int pos)> Status) {
	String ret;
	
	if (basic) {
		Status(t_("Removing duplicated panels (pass 1)"), 25);
		numDupPan = RemoveDuplicatedPanels(panels);
		
		Status(t_("Removing duplicated points"), 50);
		numDupP = RemoveDuplicatedPointsAndRenumber(panels, nodes);
		if (numDupP > 0) 
			ret << "\n" << Format(t_("Removed %d duplicated points"), numDupP);	
	
		Status(t_("Removing duplicated panels (pass 2)"), 75);
		numDupPan += RemoveDuplicatedPanels(panels);	// Second time after duplicated points
		if (numDupPan > 0) 
			ret << "\n" << Format(t_("Removed %d duplicated panels"), numDupPan);
	} else {	
		Status(t_("Detecting triangles and wrong panels"), 40);
		DetectTriBiP(panels, numTriangles, numBiQuads, numMonoQuads);
		if (numTriangles > 0)
			ret << "\n" << Format(t_("Fixed %d triangles"), numTriangles);
		if (numBiQuads > 0)
			ret << "\n" << Format(t_("Removed %d 2 points quads"), numBiQuads);
		if (numMonoQuads > 0)
			ret << "\n" << Format(t_("Removed %d 1 points quads"), numMonoQuads);
		
		Status(t_("Removing tiny panels"), 45);
		RemoveTinyPanels(panels);
		
		Status(t_("Removing duplicated panels (pass 1)"), 55);
		numDupPan = RemoveDuplicatedPanels(panels);
		
		Status(t_("Fixing skewed panels"), 60);
		numSkewed = FixSkewed();
		if (numSkewed > 0) 
			ret << "\n" << Format(t_("Fixed %d skewed panels"), numSkewed);
	
		Status(t_("Removing duplicated points"), 65);
		numDupP = RemoveDuplicatedPointsAndRenumber(panels, nodes);
		if (numDupP > 0) 
			ret << "\n" << Format(t_("Removed %d duplicated points"), numDupP);	
	
		Status(t_("Removing duplicated panels (pass 2)"), 70);
		numDupPan += RemoveDuplicatedPanels(panels);	// Second time after duplicated points
		if (numDupPan > 0) 
			ret << "\n" << Format(t_("Removed %d duplicated panels"), numDupPan);
	
		Status(t_("Analysing water tightness"), 75);
		double zTolerance = -0.1;
		AnalyseSegments(zTolerance);
		ret << "\n" << Format(t_("%d segments, %d water level, %d water leak and %d multipanel"), 
									segments.GetCount(), segWaterlevel.GetCount(), 
									segTo1panel.GetCount(), segTo3panel.GetCount());
/*		
		Status(t_("Reorienting panels water side"), 80);
		if (!ReorientPanels0(true))
			ret << "\n" << t_("Failed to reorient panels to water side");
		else if (numUnprocessed > 0)
			ret << "\n" << Format(t_("%d panels not reoriented. Body contains separated surfaces"), numUnprocessed);
*/		
		healing = true;
	}
	return ret;
}

void Surface::Orient() {
	GetSegments();
	ReorientPanels0(side);
	side = !side;
}		
		
void Surface::Image(int axis) {
	for (int i = 0; i < nodes.GetCount(); ++i) {
		Point3D &node = nodes[i];
		if (axis == 0)
			node.x = -node.x;
		else if (axis == 1)
			node.y = -node.y;
		else
			node.z = -node.z;
	}
	for (int i = 0; i < panels.GetCount(); ++i) 
		ReorientPanel(i);
}
	
const VolumeEnvelope &Surface::GetEnvelope() {
	env.maxX = env.maxY = env.maxZ = -DBL_MAX; 
	env.minX = env.minY = env.minZ = DBL_MAX;
	for (int i = 0; i < nodes.GetCount(); ++i) {
		env.maxX = max(env.maxX, nodes[i].x);
		env.minX = min(env.minX, nodes[i].x);
		env.maxY = max(env.maxY, nodes[i].y);
		env.minY = min(env.minY, nodes[i].y);
		env.maxZ = max(env.maxZ, nodes[i].z);
		env.minZ = min(env.minZ, nodes[i].z);
	}
	return env;
}

void Surface::JointTriangularPanels(int ip0, int ip1, int inode0, int inode1) {
	Panel &pan = panels[ip0];
	int iip0 = -1, iip1 = -1;
	for (int i = 0; i < 4; ++i) {
		if (pan.id[i] == inode0 || pan.id[i] == inode1)
			;
		else {
			iip0 = pan.id[i];
			break;
		}
	}
	if (iip0 < 0)
		return;		// Error?
	Panel &pan1 = panels[ip1];
	for (int i = 0; i < 4; ++i) {
		if (pan1.id[i] == inode0 || pan1.id[i] == inode1)
			;
		else {
			iip1 = pan.id[i];
			break;
		}
	}
	if (iip1 < 0)
		return;		// Error?
	pan.id[0] = iip0;
	pan.id[1] = inode0;
	pan.id[2] = iip1;
	pan.id[3] = inode1;
	FixSkewed(ip0);
	GetPanelParams(pan);
	if (pan.normal0.Angle(pan1.normal0) > 0.1*M_PI) {
		pan.Swap();
		GetPanelParams(pan);	
	}
	panels.Remove(ip1);
}

void Surface::GetPanelParams(Panel &panel) const {
	panel.RedirectTriangles();
	
	const Point3D &p0 = nodes[panel.id[0]];
	const Point3D &p1 = nodes[panel.id[1]];
	const Point3D &p2 = nodes[panel.id[2]];
	const Point3D &p3 = nodes[panel.id[3]];
	
	panel.surface0  = Area(p0, p1, p2);
	panel.centroid0 = Centroid(p0, p1, p2);
	panel.normal0   = Normal(p0, p1, p2);
	if (!panel.IsTriangle()) {
		panel.surface1  = Area(p2, p3, p0);
		panel.centroid1 = Centroid(p2, p3, p0);
		panel.normal1   = Normal(p2, p3, p0);
		double surf = panel.surface0 + panel.surface1;
		panel.centroidPaint.x = (panel.centroid0.x*panel.surface0 + panel.centroid1.x*panel.surface1)/surf;
		panel.centroidPaint.y = (panel.centroid0.y*panel.surface0 + panel.centroid1.y*panel.surface1)/surf;
		panel.centroidPaint.z = (panel.centroid0.z*panel.surface0 + panel.centroid1.z*panel.surface1)/surf;
		panel.normalPaint.x = (panel.normal0.x*panel.surface0 + panel.normal1.x*panel.surface1)/surf;
		panel.normalPaint.y = (panel.normal0.y*panel.surface0 + panel.normal1.y*panel.surface1)/surf;
		panel.normalPaint.z = (panel.normal0.z*panel.surface0 + panel.normal1.z*panel.surface1)/surf;
		panel.normalPaint.Normalize();
	} else {
		panel.surface1 = 0;
		panel.centroidPaint = panel.centroid1 = panel.centroid0;
		panel.normalPaint = panel.normal1 = panel.normal0;
	}
}

String Surface::CheckErrors() const {
	for (int ip = 0; ip < panels.GetCount(); ++ip) {
		const Panel &panel = panels[ip];
		for (int i = 0; i < 4; ++i) {
			if (panel.id[i] >= nodes.GetCount())
				return Format(t_("Node %d in panel %d [%d] does not exist"), panel.id[i]+1, ip+1, i+1);
		}
	}
	if (IsEmpty())
		return t_("Model is empty");
	return Null;
}
		
void Surface::GetPanelParams() {
	for (int ip = 0; ip < panels.GetCount(); ++ip) {
		Panel &panel = panels[ip];
		GetPanelParams(panel);
	}	
}

double Surface::GetArea() {
	surface = 0;
	if (panels.size() == 0) {
		avgFacetSideLen = 0;
		return 0;
	}
	for (int ip = 0; ip < panels.size(); ++ip) 
		surface += panels[ip].surface0 + panels[ip].surface1;
	avgFacetSideLen = sqrt(surface/panels.size());
	return surface;
}

double Surface::GetAreaXProjection(bool positive, bool negative) const {
	double area = 0;
	
	for (int ip = 0; ip < panels.size(); ++ip) {
		const Panel &panel = panels[ip];
		
		bool add0;
		if (panel.normal0.x > 0)
			add0 = positive;
		else if (panel.normal0.x < 0)
			add0 = negative;
		if (add0)
			area += -panel.surface0*panel.normal0.x;
		
		bool add1 = false;
		if (panel.normal1.x > 0)
			add1 = positive;
		else if (panel.normal1.x < 0)
			add1 = negative;
		if (add1)
			area += -panel.surface1*panel.normal1.x;
	}
	return area;
}

double Surface::GetAreaYProjection(bool positive, bool negative) const {
	double area = 0;
	
	for (int ip = 0; ip < panels.GetCount(); ++ip) {
		const Panel &panel = panels[ip];
		
		bool add0 = false;
		if (panel.normal0.y > 0)
			add0 = positive;
		else if (panel.normal0.y < 0)
			add0 = negative;
		if (add0)
			area += -panel.surface0*panel.normal0.y;
		
		bool add1;
		if (panel.normal1.y > 0)
			add1 = positive;
		else if (panel.normal1.y < 0)
			add1 = negative;
		if (add1)
			area += -panel.surface1*panel.normal1.y;
	}
	return area;
}

double Surface::GetAreaZProjection(bool positive, bool negative) const {
	double area = 0;
	
	for (int ip = 0; ip < panels.GetCount(); ++ip) {
		const Panel &panel = panels[ip];
		
		bool add0;
		if (panel.normal0.z > 0)
			add0 = positive;
		else if (panel.normal0.z < 0)
			add0 = negative;
		if (add0)
			area += -panel.surface0*panel.normal0.z;
		
		bool add1;
		if (panel.normal1.z > 0)
			add1 = positive;
		else if (panel.normal1.z < 0)
			add1 = negative;
		if (add1)
			area += -panel.surface1*panel.normal1.z;
	}
	return area;
}

Pointf Surface::GetAreaZProjectionCG() const {
	double area = 0;
	Pointf ret(0, 0);
	
	for (int ip = 0; ip < panels.GetCount(); ++ip) {
		const Panel &panel = panels[ip];
		
		double area0 = -panel.surface0*panel.normal0.z;
		area += area0;
		ret.x += area0*panel.centroid0.x;
		ret.y += area0*panel.centroid0.y;
		
		double area1 = -panel.surface1*panel.normal1.z; 
		area += area1;
		ret.x += area1*panel.centroid1.x;
		ret.y += area1*panel.centroid1.y;
	}
	if (area == 0)
		return Null;
	
	ret.x /= area;
	ret.y /= area;
	
	return ret;
}
	
	
void Surface::GetVolume() {
	volumex = volumey = volumez = 0;
	
	for (int ip = 0; ip < panels.GetCount(); ++ip) {
		const Panel &panel = panels[ip];
		
		volumex += panel.surface0*panel.normal0.x*panel.centroid0.x;
		volumey += panel.surface0*panel.normal0.y*panel.centroid0.y;
		volumez += panel.surface0*panel.normal0.z*panel.centroid0.z;
		
		if (!panel.IsTriangle()) {
			volumex += panel.surface1*panel.normal1.x*panel.centroid1.x;
			volumey += panel.surface1*panel.normal1.y*panel.centroid1.y;
			volumez += panel.surface1*panel.normal1.z*panel.centroid1.z;
		}
	}
	volume = avg(volumex, volumey, volumez);
}

int Surface::VolumeMatch(double ratioWarning, double ratioError) const {
	if (volumex == 0 || volumey == 0 || volumez == 0)
		return 0;
	if ((volumex == 0 && !Between(volume/volumex, 1-ratioError, 1+ratioError)) ||
		(volumey == 0 && !Between(volume/volumey, 1-ratioError, 1+ratioError)) ||
		(volumez == 0 && !Between(volume/volumez, 1-ratioError, 1+ratioError)))
		return -2;
	if (!Between(volume/volumex, 1-ratioWarning, 1+ratioWarning) ||
		!Between(volume/volumey, 1-ratioWarning, 1+ratioWarning) ||
		!Between(volume/volumez, 1-ratioWarning, 1+ratioWarning))
		return -1;
	return 0;
}
	
Point3D Surface::GetCenterOfBuoyancy() const {
	if (panels.size() == 0)
		return Null;
	
	double xb = 0, yb = 0, zb = 0;
	
	for (int ip = 0; ip < panels.size(); ++ip) {
		const Panel &panel = panels[ip];
		
		xb += panel.surface0*panel.normal0.x*sqr(panel.centroid0.x);
		yb += panel.surface0*panel.normal0.y*sqr(panel.centroid0.y);
		zb += panel.surface0*panel.normal0.z*sqr(panel.centroid0.z);
		
		if (!panel.IsTriangle()) {
			xb += panel.surface1*panel.normal1.x*sqr(panel.centroid1.x);
			yb += panel.surface1*panel.normal1.y*sqr(panel.centroid1.y);
			zb += panel.surface1*panel.normal1.z*sqr(panel.centroid1.z);
		}
	}
	
	if (volumex > 0 && volumey > 0 && volumez > 0) {
		xb /= 2*volumex;
		yb /= 2*volumey;
		zb /= 2*volumez;
		return Point3D(xb, yb, zb);
	} else
		return Null;
}

static void CalcDet(const Matrix3d &A, Vector3d &diag, Vector3d &offd, double &vol) {
	double d = A.determinant();  	// vol of tiny parallelepiped= d * dr * ds * dt (the 3 partials of my tetral triple integral equation)
	vol += d;                   	// add vol of current tetra (note it could be negative - that's ok we need that sometimes)
	for (int j = 0; j < 3; j++) {
		int j1 = (j + 1) % 3;
		int j2 = (j + 2) % 3;
		diag[j] += (A(0, j) * A(1, j) + A(1, j) * A(2, j) + A(2, j) * A(0, j) +
					A(0, j) * A(0, j) + A(1, j) * A(1, j) + A(2, j) * A(2, j)) *d; 
		offd(j) += (A(0, j1) * A(1, j2) + A(1, j1) * A(2, j2) + A(2, j1) * A(0, j2) +
					A(0, j1) * A(2, j2) + A(1, j1) * A(0, j2) + A(2, j1) * A(1, j2) +
					A(0, j1) * A(0, j2) * 2 + A(1, j1) * A(1, j2) * 2 + A(2, j1) * A(2, j2) * 2) *d; 
	}	
}

// From https://github.com/melax/sandbox
// Unitary matrix. It needs to be multiplied by the mass
void Surface::GetInertia33(Matrix3d &inertia, const Point3D &cg, bool refine) const {
	double vol = 0;                  
	Vector3d diag(0, 0, 0);             // accumulate matrix main diagonal integrals [x*x, y*y, z*z]
	Vector3d offd(0, 0, 0);             // accumulate matrix off-diagonal  integrals [y*z, x*z, x*y]
	
	for (int ip = 0; ip < panels.size(); ++ip) {
		const Panel &panel = panels[ip];
		Matrix3d A;
		
		Point3D r0 = nodes[panel.id[0]] - cg;
		Point3D r1 = nodes[panel.id[1]] - cg;
		Point3D r2 = nodes[panel.id[2]] - cg;
		Point3D r3 = nodes[panel.id[3]] - cg;
		
		A.row(0) << r0.x, r0.y, r0.z;
		A.row(1) << r1.x, r1.y, r1.z;
		A.row(2) << r2.x, r2.y, r2.z;
		
		CalcDet(A, diag, offd, vol);
		
		A.row(1) << r2.x, r2.y, r2.z;
		A.row(2) << r3.x, r3.y, r3.z;
		
		CalcDet(A, diag, offd, vol);
	}
	vol /= 6;
	diag /= vol*60;  
	offd /= vol*120;
	inertia << 	diag[1] + diag[2], -offd[2], 		  -offd[1],
				-offd[2], 		   diag[0] + diag[2], -offd[0],
				-offd[1], 		   -offd[0], 		  diag[0] + diag[1];
				
	if (refine) {
		double mx = inertia.cwiseAbs().maxCoeff();	
		for (int i = 0; i < inertia.size(); ++i) {
			double v = inertia.array()(i);
			if (abs(mx/v) > 1E5)
				inertia.array()(i) = 0;
		}
	}
}

void Surface::GetInertia66(MatrixXd &inertia, const Point3D &cg, bool refine) const {
	inertia = MatrixXd::Zero(6,6);	
	Matrix3d inertia3;
	GetInertia33(inertia3, cg, refine);
	inertia.bottomRightCorner<3,3>() = inertia3;
	inertia(0, 0) = inertia(1, 1) = inertia(2, 2) = 1;
	
	Point3D c = clone(cg);
	
	if (refine) {
		double mx = max(max(abs(c.x), abs(c.y)), abs(c.z));
		if (abs(c.x) < 1E-10 || abs(mx/c.x) > 1E5)
			c.x = 0;
		if (abs(c.y) < 1E-10 || abs(mx/c.y) > 1E5)
			c.y = 0;
		if (abs(c.z) < 1E-10 || abs(mx/c.z) > 1E5)
			c.z = 0;		
	} 
	inertia(1, 5) = inertia(5, 1) =  c.x;
	inertia(2, 4) = inertia(4, 2) = -c.x;
	inertia(2, 3) = inertia(3, 2) =  c.y;
	inertia(0, 5) = inertia(5, 0) = -c.y;
	inertia(0, 4) = inertia(4, 0) =  c.z;
	inertia(1, 3) = inertia(3, 1) = -c.z;
}	

void Surface::TranslateInertia33(Matrix3d &inertia, double m, const Value3D &delta) {
	inertia(0, 0) += m*(sqr(delta.y) + sqr(delta.z));
	inertia(1, 1) += m*(sqr(delta.x) + sqr(delta.z));
	inertia(2, 2) += m*(sqr(delta.x) + sqr(delta.y));
	double mxy = m*delta.x*delta.y;
	double mxz = m*delta.x*delta.z;
	double myz = m*delta.y*delta.z;
	inertia(0, 1) -= mxy;
	inertia(1, 0) -= mxy;
	inertia(0, 2) -= mxz;
	inertia(2, 0) -= mxz;
	inertia(1, 2) -= myz;
	inertia(2, 1) -= myz;
}

void Surface::TranslateInertia66(MatrixXd &inertia, const Value3D &delta) {
	double m = inertia(0, 0);
	Matrix3d inertia3 = inertia.bottomRightCorner<3,3>();
	TranslateInertia33(inertia3, m, delta);
	inertia.bottomRightCorner<3,3>() = inertia3;
	double mdx = m*delta.x;
	double mdy = m*delta.y;
	double mdz = m*delta.z;
	inertia(1, 5) += mdx;
	inertia(5, 1) += mdx;
	inertia(2, 4) -= mdx;
	inertia(4, 2) -= mdx;
	inertia(2, 3) += mdy;
	inertia(3, 2) += mdy;
	inertia(0, 5) -= mdy;
	inertia(5, 0) -= mdy;
	inertia(0, 4) += mdz;
	inertia(4, 0) += mdz;
	inertia(1, 3) -= mdz;
	inertia(3, 1) -= mdz;
}

Force6D Surface::GetHydrostaticForce(const Point3D &c0, double rho, double g) const {
	Force6D f = GetHydrostaticForceNormalized(c0);	
	
	f *= rho*g; 
	
	return f;
}
	
static void AddTrianglePressure(Force6D &f, const Point3D &centroid, const Point3D &normal, 
	double surface, const Point3D &c0, const Point3D &p1, const Point3D &p2, const Point3D &p3) {
	Direction3D f0;
	double p;
	
	p = centroid.z*surface*.75;
	f0.Set(p*normal.x, p*normal.y, p*normal.z);
	f.AddLinear(f0, centroid, c0);	
	
	p = p1.z*surface*.25/3;
	f0.Set(p*normal.x, p*normal.y, p*normal.z);
	f.AddLinear(f0, p1, c0);	

	p = p2.z*surface*.25/3;
	f0.Set(p*normal.x, p*normal.y, p*normal.z);
	f.AddLinear(f0, p2, c0);
	
	p = p3.z*surface*.25/3;
	f0.Set(p*normal.x, p*normal.y, p*normal.z);
	f.AddLinear(f0, p3, c0);
}

Force6D Surface::GetHydrostaticForceNormalized(const Point3D &c0) const {
	Force6D f;
	
	f.Reset();				
	
	for (int ip = 0; ip < panels.size(); ++ip) {	
		const Panel &panel = panels[ip];

		const Point3D &p0 = nodes[panel.id[0]];
		const Point3D &p1 = nodes[panel.id[1]];
		const Point3D &p2 = nodes[panel.id[2]];
		const Point3D &p3 = nodes[panel.id[3]];
	
		AddTrianglePressure(f, panel.centroid0, panel.normal0, panel.surface0, c0, p0, p1, p2);
		if (!panel.IsTriangle())
			AddTrianglePressure(f, panel.centroid1, panel.normal1, panel.surface1, c0, p2, p3, p0);
	}	
	return f;
}		

static void AddTriangleDynPressure(Force6D &f, const Point3D &centroid, const Point3D &normal, 
	double surface, const Point3D &c0, const Point3D &p0, const Point3D &p1, const Point3D &p2,
	double etcentroid, double et0, double et1, double et2, 
	Function<double(double x, double y)> GetZSurf,
	Function<double(double x, double y, double z, double et)> GetPress) {
		
	Direction3D f0;
	double p;
	
	if (GetZSurf) {		// Clips the triangle if it has vertexes out of the water
		Vector<const Point3D *> pup, pdown;
		Vector<double> etup, etdown;
		if (p0.z > et0) {
			pup << &p0;
			etup << et0;
		} else {
			pdown << &p0;
			etdown << et0;
		}
		if (p1.z > et1) {
			pup << &p1;
			etup << et1;
		} else {
			pdown << &p1;
			etdown << et1;
		}
		if (p2.z > et2) {
			pup << &p2;
			etup << et2;
		} else {
			pdown << &p2;
			etdown << et2;
		}
		if (pup.size() == 3)
			return;
		else if (pup.size() == 2) {
			Point3D p00 = WeightedMean(*pdown[0], pup[0]->z - etup[0], *pup[0], etdown[0] - pdown[0]->z);
			Point3D p01 = WeightedMean(*pdown[0], pup[1]->z - etup[1], *pup[1], etdown[0] - pdown[0]->z);
			Point3D centroid = Centroid(*pdown[0], p00, p01);
			double surface =  Area(*pdown[0], p00, p01);
			AddTriangleDynPressure(f, centroid, normal, surface, c0, *pdown[0], p00, p01, 
					GetZSurf(centroid.x, centroid.y), etdown[0], p00.z, p01.z, Null, GetPress);		// Inside the water... don't clip
			return;
		} else if (pup.size() == 1) {
			Point3D p00 = WeightedMean(*pdown[0], pup[0]->z - etup[0], *pup[0], etdown[0] - pdown[0]->z);
			Point3D p01 = WeightedMean(*pdown[1], pup[0]->z - etup[0], *pup[0], etdown[1] - pdown[1]->z);
			Point3D centroid1 = Centroid(*pdown[0], p00, p01);
			Point3D centroid2 = Centroid(*pdown[0], *pdown[1], p01);
			double surface1 =  Area(*pdown[0], p00, p01);
			double surface2 =  Area(*pdown[0], *pdown[1], p01);
			AddTriangleDynPressure(f, centroid1, normal, surface1, c0, *pdown[0], p00, p01, 		// Both triangles inside the water... don't clip
					GetZSurf(centroid1.x, centroid1.y), etdown[0], p00.z, p01.z, Null, GetPress);
			AddTriangleDynPressure(f, centroid2, normal, surface2, c0, *pdown[0], *pdown[1], p01, 
					GetZSurf(centroid2.x, centroid2.y), etdown[0], etdown[1], p01.z, Null, GetPress);
			return;	
		}
	}
	
	p = GetPress(centroid.x, centroid.y, centroid.z, etcentroid)*surface*.75;
	f0.Set(p*normal.x, p*normal.y, p*normal.z);
	f.AddLinear(f0, centroid, c0);	
	
	p = GetPress(p0.x, p0.y, p0.z, et0)*surface*.25/3;
	f0.Set(p*normal.x, p*normal.y, p*normal.z);
	f.AddLinear(f0, p0, c0);	

	p = GetPress(p1.x, p1.y, p1.z, et1)*surface*.25/3;
	f0.Set(p*normal.x, p*normal.y, p*normal.z);
	f.AddLinear(f0, p1, c0);
	
	p = GetPress(p2.x, p2.y, p2.z, et2)*surface*.25/3;
	f0.Set(p*normal.x, p*normal.y, p*normal.z);
	f.AddLinear(f0, p2, c0);
}

Force6D Surface::GetHydrodynamicForce(const Point3D &c0, bool clip,
						Function<double(double x, double y)> GetZSurf,
						Function<double(double x, double y, double z, double et)> GetPress) const {
	Force6D f;

	f.Reset();				

	VectorXd et(nodes.size());
	for (int ip = 0; ip < nodes.size(); ++ip) 
		et(ip) = GetZSurf(nodes[ip].x, nodes[ip].y);
	
	if (!clip)
		GetZSurf = Null;
		
	for (int ip = 0; ip < panels.size(); ++ip) {	
		const Panel &panel = panels[ip];
		
		int id0 = panel.id[0];
		int id1 = panel.id[1];
		int id2 = panel.id[2];
		int id3 = panel.id[3];
		
		const Point3D &p0 = nodes[id0];
		const Point3D &p1 = nodes[id1];
		const Point3D &p2 = nodes[id2];
		const Point3D &p3 = nodes[id3];
		
		AddTriangleDynPressure(f, panel.centroid0, panel.normal0, panel.surface0, c0, 
					p0, p1, p2, 
					GetZSurf(panel.centroid0.x, panel.centroid0.y), et(id0), et(id1), et(id2), 
						GetZSurf, GetPress);
		if (!panel.IsTriangle())
			AddTriangleDynPressure(f, panel.centroid1, panel.normal1, panel.surface1, c0, 
					p2, p3, p0, 
					GetZSurf(panel.centroid1.x, panel.centroid1.y), et(id2), et(id3), et(id0), 
						GetZSurf, GetPress);
	}
	return f;
}

Force6D Surface::GetHydrostaticForceCB(const Point3D &c0, const Point3D &cb, double rho, double g) const {
	Force6D f = GetHydrostaticForceCBNormalized(c0, cb);	
	
	f *= rho*g; 
	
	return f;
}

Force6D Surface::GetHydrostaticForceCBNormalized(const Point3D &c0, const Point3D &cb) const {
	Force6D f;
	
	f.Reset();
	
	if (volume < EPS_LEN)
		return f;
	
	f.AddLinear(Direction3D(0, 0, volume), cb, c0);		
	
	return f;
}	

Force6D Surface::GetMassForce(const Point3D &c0, const Point3D &cg, const double mass, const double g) {
	Force6D f;
	
	f.Reset();
	
	if (mass == 0)
		return f;
		
	f.AddLinear(Direction3D(0, 0, -mass*g), cg, c0);
	
	return f;
}													

double Surface::GetWaterPlaneArea() const {
	double ret = 0;
	
	for (int ip = 0; ip < panels.size(); ++ip) {	
		const Panel &panel = panels[ip];

		double momentz0 = panel.normal0.z*panel.surface0;	// n3·dS
		double momentz1 = panel.normal1.z*panel.surface1;
		ret -= (momentz0 + momentz1);
	}
	return ret;
}

void Surface::GetHydrostaticStiffness(MatrixXd &c, const Point3D &c0, const Point3D &cg, 
				const Point3D &cb, double rho, double g, double mass) {
	if (volume < EPS_VOL) {
		c.resize(0, 0);
		return;	
	}	
	c.setConstant(6, 6, 0);
	
	if (IsNull(mass))
		mass = rho*volume;
		
	for (int ip = 0; ip < panels.size(); ++ip) {	
		const Panel &panel = panels[ip];

		double momentz0 = panel.normal0.z*panel.surface0;	// n3·dS
		double momentz1 = panel.normal1.z*panel.surface1;
		double x0 = panel.centroid0.x - c0.x;				// (x - x0)
		double y0 = panel.centroid0.y - c0.y;				// (y - y0)
		double x1 = panel.centroid1.x - c0.x;	
		double y1 = panel.centroid1.y - c0.y;
/*33*/	c(2, 2) += (momentz0 + momentz1);
/*34*/  c(2, 3) += (y0*momentz0 + y1*momentz1);
/*35*/  c(2, 4) += (x0*momentz0 + x1*momentz1);
/*44*/  c(3, 3) += (y0*y0*momentz0 + y1*y1*momentz1);
/*45*/  c(3, 4) += (x0*y0*momentz0 + x1*y1*momentz1);
/*55*/  c(4, 4) += (x0*x0*momentz0 + x1*x1*momentz1);
	}
	double rho_g = rho*g;
	
/*33*/c(2, 2) = -rho_g*  c(2, 2);									
/*34*/c(2, 3) = -rho_g*  c(2, 3);
/*35*/c(2, 4) =  rho_g*  c(2, 4);
/*44*/c(3, 3) =  rho_g*(-c(3, 3) + volume*(cb.z - c0.z)) - mass*g*(cg.z - c0.z);
/*45*/c(3, 4) =  rho_g*  c(3, 4);
/*46*/c(3, 5) = 			-rho_g*volume*(cb.x - c0.x)  + mass*g*(cg.x - c0.x);
/*55*/c(4, 4) =  rho_g*(-c(4, 4) + volume*(cb.z - c0.z)) - mass*g*(cg.z - c0.z);
/*56*/c(4, 5) = 			-rho_g*volume*(cb.y - c0.y)  + mass*g*(cg.y - c0.y);
			 
	c(3, 2) = c(2, 3);
	c(4, 2) = c(2, 4);
	c(4, 3) = c(3, 4);
}

inline static void CheckAddSegZero(Vector<Segment3D> &seg, const Point3D &p0, const Point3D &p1, 
			const Point3D &p2, const Point3D &p3) {
	if (p0 != p1 && Between(p0.z, EPS_LEN) && Between(p1.z, EPS_LEN))
		seg << Segment3D(p0, p1);
	if (p1 != p2 && Between(p1.z, EPS_LEN) && Between(p2.z, EPS_LEN))
		seg << Segment3D(p1, p2);
	if (p2 != p3 && Between(p2.z, EPS_LEN) && Between(p3.z, EPS_LEN))
		seg << Segment3D(p2, p3);
	if (p3 != p0 && Between(p3.z, EPS_LEN) && Between(p0.z, EPS_LEN))
		seg << Segment3D(p3, p0);
}

void Surface::CutZ(const Surface &orig, int factor) {
	nodes = clone(orig.nodes);
	panels.Clear();
	
	segWaterlevel.Clear();
	factor *= -1;
	
	for (int ip = 0; ip < orig.panels.GetCount(); ++ip) {
		const int &id0 = orig.panels[ip].id[0];
		const int &id1 = orig.panels[ip].id[1];
		const int &id2 = orig.panels[ip].id[2];
		const int &id3 = orig.panels[ip].id[3];
		const Point3D &p0 = nodes[id0];
		const Point3D &p1 = nodes[id1];
		const Point3D &p2 = nodes[id2];
		const Point3D &p3 = nodes[id3];	
		
		CheckAddSegZero(segWaterlevel, p0, p1, p2, p3);

		if ((p0.z)*factor <= EPS_LEN && (p1.z)*factor <= EPS_LEN && 
				 (p2.z)*factor <= EPS_LEN && (p3.z)*factor <= EPS_LEN) {
			if (!((p0.z)*factor >= -EPS_LEN && (p1.z)*factor >= -EPS_LEN && // Rejects waterplane
				(p2.z)*factor >= -EPS_LEN && (p3.z)*factor >= -EPS_LEN))		     
				panels << Panel(orig.panels[ip]);			// Gets the panels that comply
		} else if ((p0.z)*factor >= -EPS_LEN && (p1.z)*factor >= -EPS_LEN && 
			(p2.z)*factor >= -EPS_LEN && (p3.z)*factor >= -EPS_LEN) 
			;											// Refuses the panels that don't
		else {											// Process the intermediate
			const int *origPanelid = orig.panels[ip].id;
			Vector<int> nodeFrom, nodeTo;
			Segment3D segWL;
			segWL.from = segWL.to = Null;
			const int ids[] = {0, 1, 2, 3, 0};
			for (int i = 0; i < 4; ++i) {
				if (origPanelid[ids[i]] == origPanelid[ids[i+1]])
					;
				else {
					const Point3D &from = nodes[origPanelid[ids[i]]];
					const Point3D &to   = nodes[origPanelid[ids[i+1]]];
					if (abs(from.z) <= EPS_LEN && abs(to.z) <= EPS_LEN) {
						segWL.from = from;
						segWL.to = to;	
					} else if ((from.z)*factor <= 0 && (to.z)*factor <= 0) {
						nodeFrom << origPanelid[ids[i]];
						nodeTo << origPanelid[ids[i+1]];
					} else if ((from.z)*factor >= 0 && (to.z)*factor >= 0) 
						;
					else {
						Segment3D seg(from, to);
						Point3D inter = seg.IntersectionPlaneZ(0);
						if (!IsNull(inter)) {
							if ((from.z)*factor < 0) {
								nodeFrom << origPanelid[ids[i]];
								nodes << inter;
								nodeTo << nodes.GetCount() - 1;
							} else {
								nodeTo << origPanelid[ids[i+1]];
								nodes << inter;
								nodeFrom << nodes.GetCount() - 1;
							}
						}
						if (IsNull(segWL.from))
							segWL.from = inter;
						else if (IsNull(segWL.to))
							segWL.to = inter;
					}
				}
			}
			if (!IsNull(segWL))
				segWaterlevel << segWL;
			
			int pos = -1, nFrom, nTo;
			for (int i = 0; i < nodeFrom.GetCount(); ++i) {
				int i_1 = i + 1;
				if (i_1 >= nodeFrom.GetCount())
					i_1 = 0;
				if (nodeTo[i] != nodeFrom[i_1]) {
					pos = i+1;
					nFrom = nodeTo[i];
					nTo = nodeFrom[i_1];
					break;
				}
			}
			if (pos == nodeTo.GetCount()) {
				nodeFrom << nFrom;
				nodeTo << nTo;
			} else if (pos >= 0) {
				nodeFrom.Insert(pos, nFrom);		
				nodeTo.Insert(pos, nTo);
			}
			
			Panel panel;
			if (nodeFrom.GetCount() == 3) {
				panel.id[0] = nodeFrom[0];
				panel.id[1] = nodeFrom[1];
				panel.id[2] = nodeFrom[2];
				panel.id[3] = nodeFrom[2];
			} else if (nodeFrom.GetCount() == 4) {
				panel.id[0] = nodeFrom[0];
				panel.id[1] = nodeFrom[1];
				panel.id[2] = nodeFrom[2];
				panel.id[3] = nodeFrom[3];
			} else if (nodeFrom.GetCount() == 5) {
				panel.id[0] = nodeFrom[0];
				panel.id[1] = nodeFrom[1];
				panel.id[2] = nodeFrom[2];
				panel.id[3] = nodeFrom[3];
				Panel panel2;
				panel2.id[0] = nodeFrom[0];
				panel2.id[1] = nodeFrom[3];
				panel2.id[2] = nodeFrom[4];
				panel2.id[3] = nodeFrom[4];
				//TriangleToQuad(panel2); 
				panels << panel2;
			}
			//TriangleToQuad(panel);
			panels << panel;
		}
	}
	DeleteVoidSegments(segWaterlevel);
	DeleteDuplicatedSegments(segWaterlevel);
	for (Point3D &node : nodes) 
		node.z = min(node.z, 0.);	// No tolerance -> max z is 0
}

void Surface::CutX(const Surface &orig, int factor) {
	nodes = clone(orig.nodes);
	panels.Clear();
	
	factor *= -1;
	
	for (int ip = 0; ip < orig.panels.GetCount(); ++ip) {
		const int &id0 = orig.panels[ip].id[0];
		const int &id1 = orig.panels[ip].id[1];
		const int &id2 = orig.panels[ip].id[2];
		const int &id3 = orig.panels[ip].id[3];
		const Point3D &p0 = nodes[id0];
		const Point3D &p1 = nodes[id1];
		const Point3D &p2 = nodes[id2];
		const Point3D &p3 = nodes[id3];	
		
		if ((p0.x)*factor >= 0 && (p1.x)*factor >= 0 && (p2.x)*factor >= 0 && (p3.x)*factor >= 0) 
			;
		else if ((p0.x)*factor <= 0 && (p1.x)*factor <= 0 && (p2.x)*factor <= 0 && (p3.x)*factor <= 0) 
			panels << Panel(orig.panels[ip]);
		else {
			const int *origPanelid = orig.panels[ip].id;
			Vector<int> nodeFrom, nodeTo;

			const int ids[] = {0, 1, 2, 3, 0};
			for (int i = 0; i < 4; ++i) {
				if (origPanelid[ids[i]] == origPanelid[ids[i+1]])
					;
				else {
					const Point3D &from = nodes[origPanelid[ids[i]]];
					const Point3D &to   = nodes[origPanelid[ids[i+1]]];
					Segment3D seg(from, to);
					if (abs(from.x) <= EPS_LEN && abs(to.x) <= EPS_LEN) {
						nodeFrom << origPanelid[ids[i]];
						nodeTo << origPanelid[ids[i+1]];
					} else if ((from.x)*factor <= 0 && (to.x)*factor <= 0) {
						nodeFrom << origPanelid[ids[i]];
						nodeTo << origPanelid[ids[i+1]];
					} else if ((from.x)*factor >= 0 && (to.x)*factor >= 0) 
						;
					else {
						Point3D inter = seg.IntersectionPlaneX(0);
						if (!IsNull(inter)) {
							if ((from.x)*factor < 0) {
								nodeFrom << origPanelid[ids[i]];
								nodes << inter;
								nodeTo << nodes.GetCount() - 1;
							} else {
								nodeTo << origPanelid[ids[i+1]];
								nodes << inter;
								nodeFrom << nodes.GetCount() - 1;
							}
						}
					}
				}
			}
			
			int pos = -1, nFrom, nTo;
			for (int i = 0; i < nodeFrom.GetCount(); ++i) {
				int i_1 = i + 1;
				if (i_1 >= nodeFrom.GetCount())
					i_1 = 0;
				if (nodeTo[i] != nodeFrom[i_1]) {
					pos = i+1;
					nFrom = nodeTo[i];
					nTo = nodeFrom[i_1];
					break;
				}
			}
			if (pos == nodeTo.size()) {
				nodeFrom << nFrom;
				nodeTo << nTo;
			} else if (pos >= 0) {
				nodeFrom.Insert(pos, nFrom);		
				nodeTo.Insert(pos, nTo);
			}
			
			Panel panel;
			if (nodeFrom.size() == 3) {
				panel.id[0] = nodeFrom[0];
				panel.id[1] = nodeFrom[1];
				panel.id[2] = nodeFrom[2];
				panel.id[3] = nodeFrom[2];
			} else if (nodeFrom.size() == 4) {
				panel.id[0] = nodeFrom[0];
				panel.id[1] = nodeFrom[1];
				panel.id[2] = nodeFrom[2];
				panel.id[3] = nodeFrom[3];
			} else if (nodeFrom.size() == 5) {
				panel.id[0] = nodeFrom[0];
				panel.id[1] = nodeFrom[1];
				panel.id[2] = nodeFrom[2];
				panel.id[3] = nodeFrom[3];
				Panel panel2;
				panel2.id[0] = nodeFrom[0];
				panel2.id[1] = nodeFrom[3];
				panel2.id[2] = nodeFrom[4];
				panel2.id[3] = nodeFrom[4];
				//TriangleToQuad(panel2); 
				panels << panel2;
			}
			//TriangleToQuad(panel);
			panels << panel;
		}
	}
}

void Surface::CutY(const Surface &orig, int factor) {
	nodes = clone(orig.nodes);
	panels.Clear();
	
	factor *= -1;
	
	for (int ip = 0; ip < orig.panels.size(); ++ip) {
		const int &id0 = orig.panels[ip].id[0];
		const int &id1 = orig.panels[ip].id[1];
		const int &id2 = orig.panels[ip].id[2];
		const int &id3 = orig.panels[ip].id[3];
		const Point3D &p0 = nodes[id0];
		const Point3D &p1 = nodes[id1];
		const Point3D &p2 = nodes[id2];
		const Point3D &p3 = nodes[id3];	
		
		if ((p0.y)*factor >= 0 && (p1.y)*factor >= 0 && (p2.y)*factor >= 0 && (p3.y)*factor >= 0) 
			;
		else if ((p0.y)*factor <= 0 && (p1.y)*factor <= 0 && (p2.y)*factor <= 0 && (p3.y)*factor <= 0) 
			panels << Panel(orig.panels[ip]);
		else {
			const int *origPanelid = orig.panels[ip].id;
			Vector<int> nodeFrom, nodeTo;

			const int ids[] = {0, 1, 2, 3, 0};
			for (int i = 0; i < 4; ++i) {
				if (origPanelid[ids[i]] == origPanelid[ids[i+1]])
					;
				else {
					const Point3D &from = nodes[origPanelid[ids[i]]];
					const Point3D &to   = nodes[origPanelid[ids[i+1]]];
					if ((from.y)*factor <= 0 && (to.y)*factor <= 0) {
						nodeFrom << origPanelid[ids[i]];
						nodeTo << origPanelid[ids[i+1]];
					} else if ((from.y)*factor >= 0 && (to.y)*factor >= 0) 
						;
					else {
						Segment3D seg(from, to);
						Point3D inter = seg.IntersectionPlaneY(0);
						if (!IsNull(inter)) {
							if ((from.y)*factor < 0) {
								nodeFrom << origPanelid[ids[i]];
								nodes << inter;
								nodeTo << nodes.size() - 1;
							} else {
								nodeTo << origPanelid[ids[i+1]];
								nodes << inter;
								nodeFrom << nodes.size() - 1;
							}
						}
					}
				}
			}
			
			int pos = -1, nFrom, nTo;
			for (int i = 0; i < nodeFrom.size(); ++i) {
				int i_1 = i + 1;
				if (i_1 >= nodeFrom.size())
					i_1 = 0;
				if (nodeTo[i] != nodeFrom[i_1]) {
					pos = i+1;
					nFrom = nodeTo[i];
					nTo = nodeFrom[i_1];
					break;
				}
			}
			if (pos == nodeTo.size()) {
				nodeFrom << nFrom;
				nodeTo << nTo;
			} else if (pos >= 0) {
				nodeFrom.Insert(pos, nFrom);		
				nodeTo.Insert(pos, nTo);
			}
			
			Panel panel;
			if (nodeFrom.size() == 3) {
				panel.id[0] = nodeFrom[0];
				panel.id[1] = nodeFrom[1];
				panel.id[2] = nodeFrom[2];
				panel.id[3] = nodeFrom[2];
			} else if (nodeFrom.size() == 4) {
				panel.id[0] = nodeFrom[0];
				panel.id[1] = nodeFrom[1];
				panel.id[2] = nodeFrom[2];
				panel.id[3] = nodeFrom[3];
			} else if (nodeFrom.size() == 5) {
				panel.id[0] = nodeFrom[0];
				panel.id[1] = nodeFrom[1];
				panel.id[2] = nodeFrom[2];
				panel.id[3] = nodeFrom[3];
				Panel panel2;
				panel2.id[0] = nodeFrom[0];
				panel2.id[1] = nodeFrom[3];
				panel2.id[2] = nodeFrom[4];
				panel2.id[3] = nodeFrom[4];
				//TriangleToQuad(panel2); 
				panels << panel2;
			}
			//TriangleToQuad(panel);
			panels << panel;
		}
	}
}

void Surface::Join(const Surface &orig) {
	int num = nodes.size();
	int numOrig = orig.nodes.size();
	nodes.SetCount(num + numOrig);
	for (int i = 0; i < numOrig; ++i)
		nodes[num+i] = orig.nodes[i];
	
	int numPan = panels.size();
	int numPanOrig = orig.panels.size();
	panels.SetCount(numPan + numPanOrig);
	for (int i = 0; i < numPanOrig; ++i) {
		Panel &pan = panels[numPan+i];
		const Panel &panOrig = orig.panels[i];
		
		for (int ii = 0; ii < 4; ++ii)
			pan.id[ii] = panOrig.id[ii] + num;
		
		GetPanelParams(pan);
	}
	
	Surface::RemoveDuplicatedPanels(panels);
	Surface::RemoveDuplicatedPointsAndRenumber(panels, nodes);
	Surface::RemoveDuplicatedPanels(panels);
	
	//pos.Set(0,0,0);
	//angle.Set(0,0,0);
}
	
void Surface::Translate(double dx, double dy, double dz) {
	for (int i = 0; i < nodes.size(); ++i) 
		nodes[i].Translate(dx, dy, dz); 
	
	for (int i = 0; i < skewed.size(); ++i) 
		skewed[i].Translate(dx, dy, dz);
	
	for (int i = 0; i < segTo1panel.size(); ++i) 
		segTo1panel[i].Translate(dx, dy, dz);
	for (int i = 0; i < segTo3panel.size(); ++i) 
		segTo3panel[i].Translate(dx, dy, dz);
	
	//pos += Point3D(dx, dy, dz);
}

void Surface::Rotate(double a_x, double a_y, double a_z, double c_x, double c_y, double c_z) {
	Affine3d quat;
	GetTransform(quat, a_x, a_y, a_z, c_x, c_y, c_z);
	
	for (int i = 0; i < nodes.size(); ++i) 
		nodes[i].TransRot(quat);

	for (int i = 0; i < skewed.size(); ++i) 
		skewed[i].TransRot(quat);
	
	for (int i = 0; i < segTo1panel.size(); ++i) 
		segTo1panel[i].TransRot(quat);
	for (int i = 0; i < segTo3panel.size(); ++i) 
		segTo3panel[i].TransRot(quat);
	
	//angle += Point3D(a_x, a_y, a_z);
}

void Surface::TransRot(double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz) {
	Affine3d quat;
	GetTransform(quat, dx, dy, dz, ax, ay, az, cx, cy, cz);
	
	for (int i = 0; i < nodes.size(); ++i) 
		nodes[i].TransRot(quat);

	for (int i = 0; i < skewed.size(); ++i) 
		skewed[i].TransRot(quat);
	
	for (int i = 0; i < segTo1panel.size(); ++i) 
		segTo1panel[i].TransRot(quat);
	for (int i = 0; i < segTo3panel.size(); ++i) 
		segTo3panel[i].TransRot(quat);
}


bool Surface::TranslateArchimede(double mass, double rho, double &dz, Surface &under) {
	if (IsEmpty())
		return false;
	Surface base;
	
	if (env.minZ  > 0)
		dz = env.minZ;

	auto Residual = [&](const double dz)->double {
		base = clone(*this);
		base.Translate(0, 0, dz);
		under.CutZ(base, -1);
		under.GetPanelParams();
		under.GetVolume();
		if (under.volume == 0 || under.volume == volume)
			return Null;		
		return under.volume*rho - mass;		// ∑ Fheave = 0
	};

	int nIter = 0;	

	VectorXd x(1);
	x[0] = dz;
	if (SolveNonLinearEquations(x, [&](const VectorXd &x, VectorXd &residual)->int {
		nIter++;
		residual[0] = Residual(x[0]);		// ∑ Fheave = 0
		if (IsNull(residual[0]))
			residual[0] = mass;
		if (abs(residual[0]) < 0.1)
			return -1;
		return 0;
	}))
		dz = x[0];
	else {
		// Slow, but more robust...
		double ndz, mn = mass*10;
		for (double dz2 = 0; dz2 < 50; dz2 += 0.5) {
			double res = Residual(dz+dz2);
			if (IsNull(res))
				break;
			res = abs(res);
			if (res < mn) {
				mn = res;
				ndz = dz2;
			}
		}
		for (double dz2 = -0.5; dz2 > -50; dz2 -= 0.5) {
			double res = Residual(dz+dz2);
			if (IsNull(res))
				break;
			res = abs(res);
			if (res < mn) {
				mn = res;
				ndz = dz2;
			}
		}
		for (double dz2 = ndz-0.7; dz2 <= ndz+0.7; dz2 += 0.1) {
			double res = Residual(dz+dz2);
			if (IsNull(res))
				return false;
			res = abs(res);
			if (res < mn) {
				mn = res;
				ndz = dz2;
			}
		}
		for (double dz2 = ndz-0.07; dz2 <= ndz+0.07; dz2 += 0.01) {
			double res = Residual(dz+dz2);
			if (IsNull(res))
				return false;
			res = abs(res);
			if (res < mn) {
				mn = res;
				ndz = dz2;
			}
		}
		dz += ndz;
	}
	
	Translate(0, 0, dz);
	return true;
}

bool Surface::Archimede(double mass, Point3D &cg, const Point3D &c0, double rho, double g, double &dz, double &droll, double &dpitch, Surface &under) {
	int maxIter = 50;
	Surface base;
	Point3D basecg;
	
	int nIter = 0;	
	
	VectorXd x(2);
	x[0] = droll;
	x[1] = dpitch;
	try {
		if (!SolveNonLinearEquations(x, [&](const VectorXd &x, VectorXd &residual)->int {
			nIter++;
			
			double droll = x[0], dpitch = x[1];
			
			base = clone(*this);
			basecg = clone(cg);
	
			base.Rotate(ToRad(droll), ToRad(dpitch), 0, c0.x, c0.y, c0.z);
			basecg.Rotate(ToRad(droll), ToRad(dpitch), 0, c0.x, c0.y, c0.z);
			
			if (!base.TranslateArchimede(mass, rho, dz, under))
				throw Exc("");
			Point3D cb = under.GetCenterOfBuoyancy();
			basecg.Translate(0, 0, dz);
			
			if (dz < 0.01 && Distance(cb, basecg) < 0.01)
				return -1;
		
			Force6D fcb;
			Force6D fcg = GetMassForce(c0, basecg, mass, g);
			double rho;
			if (under.volume > 0) {
				rho = mass/under.volume;
				fcb = under.GetHydrostaticForceCB(c0, cb, rho, g);
			} else
				fcb.Reset();
		
			residual[0] = fcb.r.x + fcg.r.x;		// ∑ Froll = 0
			residual[1] = fcb.r.y + fcg.r.y;		// ∑ Fpitch = 0
			
			if (abs(residual[0]) < 0.01 && abs(residual[1]) < 0.01)
				return -1;
			return 0;
			}))
			return false;
	} catch (...) {
		return false;	
	}
	droll = x[0];
	dpitch = x[1];
	
	TransRot(0, 0, dz, ToRad(droll), ToRad(dpitch), 0, c0.x, c0.y, c0.z);
	cg.TransRot(0, 0, dz, ToRad(droll), ToRad(dpitch), 0, c0.x, c0.y, c0.z);
	return true;
}

void Surface::Scale(double rx, double ry, double rz, const Point3D &c0) {
	for (auto &n : nodes) 
		n.Translate(rx*(n.x -c0.x), ry*(n.y -c0.y), rz*(n.z -c0.z)); 
	
	for (auto &n : skewed) 
		n.Scale(rx, ry, rz, c0); 
	
	for (auto &n : segTo1panel) 
		n.Scale(rx, ry, rz, c0); 
	for (auto &n : segTo3panel) 
		n.Scale(rx, ry, rz, c0); 
}

void Surface::DeployXSymmetry() {
	int nnodes = nodes.GetCount();
	for (int i = 0; i < nnodes; ++i) {
		Point3D 	  &dest = nodes.Add();
		const Point3D &orig = nodes[i];
		dest.x = -orig.x;
		dest.y =  orig.y;
		dest.z =  orig.z;
	}
	int npanels = panels.GetCount();
	for (int i = 0; i < npanels; ++i) {
		Panel 		&dest = panels.Add();
		const Panel &orig = panels[i];
		dest.id[0] = orig.id[3] + nnodes;
		dest.id[1] = orig.id[2] + nnodes;
		dest.id[2] = orig.id[1] + nnodes;
		dest.id[3] = orig.id[0] + nnodes;
	}
}

void Surface::DeployYSymmetry() {
	int nnodes = nodes.GetCount();
	for (int i = 0; i < nnodes; ++i) {
		Point3D 	  &dest = nodes.Add();
		const Point3D &orig = nodes[i];
		dest.x =  orig.x;
		dest.y = -orig.y;
		dest.z =  orig.z;
	}
	int npanels = panels.GetCount();
	for (int i = 0; i < npanels; ++i) {
		Panel 		&dest = panels.Add();
		const Panel &orig = panels[i];
		dest.id[0] = orig.id[3] + nnodes;
		dest.id[1] = orig.id[2] + nnodes;
		dest.id[2] = orig.id[1] + nnodes;
		dest.id[3] = orig.id[0] + nnodes;
	}
}

void VolumeEnvelope::MixEnvelope(VolumeEnvelope &env) {
	maxX = maxNotNull(env.maxX, maxX);
	minX = minNotNull(env.minX, minX);
	maxY = maxNotNull(env.maxY, maxY);
	minY = minNotNull(env.minY, minY);
	maxZ = maxNotNull(env.maxZ, maxZ);
	minZ = minNotNull(env.minZ, minZ);
}

void Surface::AddNode(Point3D &p) {
	double similThres = EPS_LEN;
	for (int i = 0; i < nodes.GetCount(); ++i) {
		if (nodes[i].IsSimilar(p, similThres))
			return;
	}
	nodes << p;
}

int Surface::FindNode(Point3D &p) {
	for (int i = 0; i < nodes.GetCount(); ++i) {
		if (nodes[i] == p)
			return i;
	}
	return -1;
}
	
void Surface::AddFlatPanel(double lenX, double lenY, double panelWidth) {
	int numX = int(round(lenX/panelWidth));
	ASSERT(numX > 0);
	double widthX = lenX/numX;	
	
	int numY = int(round(lenY/panelWidth));
	ASSERT(numY > 0);
	double widthY = lenY/numY;	
	
	Array<PanelPoints> pans;
	pans.SetCount(numX*numY);
	int n = 0;
	for (int i = 0; i < numX; ++i) {
		for (int j = 0; j < numY; ++j) {
			pans[n].data[0].x = widthX*i;		pans[n].data[0].y = widthY*j;		pans[n].data[0].z = 0; 
			pans[n].data[1].x = widthX*(i+1);	pans[n].data[1].y = widthY*j;		pans[n].data[1].z = 0;
			pans[n].data[2].x = widthX*(i+1);	pans[n].data[2].y = widthY*(j+1);	pans[n].data[2].z = 0;
			pans[n].data[3].x = widthX*i;		pans[n].data[3].y = widthY*(j+1);	pans[n].data[3].z = 0;
			n++;
		}
	}
	SetPanelPoints(pans);
}

void Surface::AddRevolution(Vector<Pointf> &points, double panelWidth) {
	if (points.GetCount() < 2)
		throw Exc(t_("Point nimver has to be higher than 2"));
	
	for (int i = points.GetCount()-2; i >= 0; --i) {
		double len = sqrt(sqr(points[i].x-points[i+1].x) + sqr(points[i].y-points[i+1].y)); 
		int num = int(round(len/panelWidth));
		if (num > 1) {
			double x0 = points[i].x;
			double lenx = points[i+1].x - points[i].x;
			double y0 = points[i].y;
			double leny = points[i+1].y - points[i].y;
			for (int in = num-1; in >= 1; --in) {
				Pointf p(x0 + lenx*in/num, y0 + leny*in/num);
				points.Insert(i+1, p);
			}
		}
	}

	double maxx = 0;
	for (Pointf &p : points)
		maxx = max(maxx, p.x);
	
	int numSlices = int(round(2*M_PI*maxx/panelWidth));
	if (Odd(numSlices))
		numSlices++;
	
	if (numSlices < 3)
		throw Exc(t_("Panel width too large"));
		
	Array<PanelPoints> pans;
	pans.SetCount(numSlices*(points.GetCount()-1));
	int n = 0;
	for (int i = 0; i < points.GetCount()-1; ++i) {
		if (points[i].x == 0) {
			for (int j = 0; j < numSlices; j += 2) {
				pans[n].data[0].x = 0;	
				pans[n].data[0].y = 0;
				pans[n].data[0].z = points[i].y;

				pans[n].data[1].x = points[i+1].x*cos((2*M_PI*j)/numSlices);	
				pans[n].data[1].y = points[i+1].x*sin((2*M_PI*j)/numSlices);
				pans[n].data[1].z = points[i+1].y;
	
				pans[n].data[2].x = points[i+1].x*cos((2*M_PI*(j+1))/numSlices);	
				pans[n].data[2].y = points[i+1].x*sin((2*M_PI*(j+1))/numSlices);
				pans[n].data[2].z = points[i+1].y;
											
				pans[n].data[3].x = points[i+1].x*cos((2*M_PI*(j+2))/numSlices);	
				pans[n].data[3].y = points[i+1].x*sin((2*M_PI*(j+2))/numSlices);
				pans[n].data[3].z = points[i+1].y;
				
				n++;
			}
		} else if (points[i+1].x == 0) {
			for (int j = 0; j < numSlices; j += 2) {
				pans[n].data[0].x = points[i].x*cos((2*M_PI*j)/numSlices);	
				pans[n].data[0].y = points[i].x*sin((2*M_PI*j)/numSlices);
				pans[n].data[0].z = points[i].y;
	
				pans[n].data[1].x = points[i].x*cos((2*M_PI*(j+1))/numSlices);	
				pans[n].data[1].y = points[i].x*sin((2*M_PI*(j+1))/numSlices);
				pans[n].data[1].z = points[i].y;
											
				pans[n].data[2].x = points[i].x*cos((2*M_PI*(j+2))/numSlices);	
				pans[n].data[2].y = points[i].x*sin((2*M_PI*(j+2))/numSlices);
				pans[n].data[2].z = points[i].y;

				pans[n].data[3].x = 0;	
				pans[n].data[3].y = 0;
				pans[n].data[3].z = points[i+1].y;
								
				n++;
			}
		} else {
			for (int j = 0; j < numSlices; ++j) {
				pans[n].data[0].x = points[i].x*cos((2*M_PI*j)/numSlices);	
				pans[n].data[0].y = points[i].x*sin((2*M_PI*j)/numSlices);
				pans[n].data[0].z = points[i].y;
				
				pans[n].data[1].x = points[i].x*cos((2*M_PI*(j+1))/numSlices);	
				pans[n].data[1].y = points[i].x*sin((2*M_PI*(j+1))/numSlices);
				pans[n].data[1].z = points[i].y;
	
				pans[n].data[2].x = points[i+1].x*cos((2*M_PI*(j+1))/numSlices);	
				pans[n].data[2].y = points[i+1].x*sin((2*M_PI*(j+1))/numSlices);
				pans[n].data[2].z = points[i+1].y;
	
				pans[n].data[3].x = points[i+1].x*cos((2*M_PI*j)/numSlices);	
				pans[n].data[3].y = points[i+1].x*sin((2*M_PI*j)/numSlices);
				pans[n].data[3].z = points[i+1].y;
				
				n++;
			}
		}
	}
	pans.SetCount(n);
	SetPanelPoints(pans);
}

void Surface::SetPanelPoints(Array<PanelPoints> &pans) {
	for (int i = 0; i < pans.GetCount(); ++i) {
		PanelPoints &pan = pans[i];
		for (int j = 0; j < 4; ++j) {
			Point3D p(pan.data[j].x, pan.data[j].y, pan.data[j].z);
			AddNode(p);
		}
	}
	for (int i = 0; i < pans.GetCount(); ++i) {
		PanelPoints &pan = pans[i];
		Panel &panel = panels.Add();
		for (int j = 0; j < 4; ++j) {
			Point3D p(pan.data[j].x, pan.data[j].y, pan.data[j].z);
			int id = FindNode(p);
			if (id < 0)
				throw Exc("Node not found in SetPanelPoints()");
			panel.id[j] = id;
		}
	}
}

bool Surface::FindMatchingPanels(const Array<PanelPoints> &pans, double x, double y, 
								 double width, int &idpan1, int &idpan2) {
	idpan1 = idpan2 = -1;
	Vector<Point3D> pt;
	pt << Point3D(x, 		 y, 		0);
	pt << Point3D(x, 		 y + width, 0);
	pt << Point3D(x + width, y + width, 0);
	pt << Point3D(x + width, y, 0);
	for (int ip = 0; ip < pans.GetCount(); ++ip) {	
		int numMatchingPoints = 0;
		for (int i = 0; i < 3; ++i) {
			for (int ipt = 0; ipt < 4; ++ipt) 
				if (pans[ip].data[i] == pt[ipt])
					numMatchingPoints++;
		}
		if (numMatchingPoints == 3) {
			if (idpan1 < 0)
				idpan1 = ip;
			else {
				idpan2 = ip;
				return true;
			}
		}
	}
	return false;
}


Vector<double> GetPolyAngles(const Array<Pointf> &bound) {
	Vector<double> angles(bound.size());
	int i;
	for (i = 0; i < angles.size()-1; ++i) 
		angles[i] = ToDeg(Angle(bound[i], bound[i+1]));
	angles[i] = ToDeg(Angle(bound[i], bound[0]));
	return angles;
}

void Surface::AddPolygonalPanel2(Array<Pointf> &bound, double width, bool adjustSize) {
	ASSERT(bound.GetCount() >= 2);
	
	if (bound[0] != bound[bound.size()-1])
		bound << bound[0];
	
	Array<Pointf> points = clone(bound);
	
	if (adjustSize) {
		for (int i = points.GetCount()-2; i >= 0; --i) {
			double len = sqrt(sqr(points[i].x-points[i+1].x) + sqr(points[i].y-points[i+1].y)); 
			int num = int(len/width);
			if (num > 1) {
				double x0 = points[i].x;
				double lenx = points[i+1].x - points[i].x;
				double y0 = points[i].y;
				double leny = points[i+1].y - points[i].y;
				for (int in = num-1; in >= 1; --in) {
					Pointf p(x0 + lenx*in/num, y0 + leny*in/num);
					points.Insert(i+1, p);
				}
			}
		}
	}
	
	double minX = std::numeric_limits<double>::max(), minY = std::numeric_limits<double>::max(), 
		   maxX = std::numeric_limits<double>::lowest(), maxY = std::numeric_limits<double>::lowest();
	for (const auto &p : points) {
		if (p.x < minX)
			minX = p.x;
		if (p.y < minY)
			minY = p.y;
		if (p.x > maxX)
			maxX = p.x;
		if (p.y > maxY)
			maxY = p.y;
	}
	Array<Pointf> poly;
	for (double x = minX; x < maxX; x += width) {
		for (double y = minY; y < maxY; y += width) {	
			Pointf pt(x, y);
			bool addPoint = true;
			for (int i = 0; i < points.size(); ++i)
				if (Distance(points[i], pt) < 0.8*width) {
					addPoint = false;
					break;
				}
			if (addPoint && ContainsPoint(points, pt) != CMP_OUT)
				poly << pt;
		}
	}
	for (const auto &p : points) 
		poly << p;

	Delaunay del;
	del.Build(poly);

	Array<PanelPoints> pans;
	for (int i = 0; i < del.GetCount(); ++i) {
		const Delaunay::Triangle &tri = del[i];
		if (tri[0] < 0 || tri[1] < 0 || tri[2] < 0)  
			continue;

		const Pointf &p0 = poly[tri[0]];
		const Pointf &p1 = poly[tri[1]];
		const Pointf &p2 = poly[tri[2]];

		int idp0 = Find(bound, p0);
		int idp1 = Find(bound, p1);
		int idp2 = Find(bound, p2);
		
		bool t01 = idp0 >= 0 && idp1 >= 0 && abs(idp0 - idp1) == 1;	// In the bound and adjacent
		if (!t01)
			t01 = ContainsPoint(bound, Middle(p0, p1)) != CMP_OUT;

		bool t12 = idp1 >= 0 && idp2 >= 0 && abs(idp1 - idp2) == 1;
		if (!t12)
			t12 = ContainsPoint(bound, Middle(p1, p2)) != CMP_OUT;
			
		bool t20 = idp2 >= 0 && idp0 >= 0 && abs(idp2 - idp0) == 1;
		if (!t20)
			t20 = ContainsPoint(bound, Middle(p2, p0)) != CMP_OUT;		
		
		if (t01 && t12 && t20) {
			PanelPoints &pan = pans.Add();
			pan.data[0].x = poly[tri[0]].x;		pan.data[0].y = poly[tri[0]].y;		pan.data[0].z = 0;
			pan.data[1].x = poly[tri[1]].x;		pan.data[1].y = poly[tri[1]].y;		pan.data[1].z = 0;
			pan.data[2].x = poly[tri[2]].x;		pan.data[2].y = poly[tri[2]].y;		pan.data[2].z = 0;
			pan.data[3].x = poly[tri[2]].x;		pan.data[3].y = poly[tri[2]].y;		pan.data[3].z = 0;
		}
	}
	// Convert triangles to quads. To be improved
	for (double x = minX; x < maxX; x += width) {
		for (double y = minY; y < maxY; y += width) {	
			int idpan1, idpan2;
			if (FindMatchingPanels(pans, x, y, width, idpan1, idpan2)) {
				pans[idpan1].data[0] = Point3D(x, 		  y, 		 0);
				pans[idpan1].data[1] = Point3D(x + width, y, 		 0);
				pans[idpan1].data[2] = Point3D(x + width, y + width, 0);
				pans[idpan1].data[3] = Point3D(x, y + width, 0);
				pans.Remove(idpan2);
			}
		}
	}
	SetPanelPoints(pans);
}

void Surface::AddPolygonalPanel(Vector<Pointf> &bound, double panelWidth, bool adjustSize) {
	ASSERT(bound.GetCount() >= 2);
	
	if (bound[0] != bound[bound.size()-1])
		bound << bound[0];
	
	if (adjustSize) {
		for (int i = bound.GetCount()-2; i >= 0; --i) {
			double len = sqrt(sqr(bound[i].x-bound[i+1].x) + sqr(bound[i].y-bound[i+1].y)); 
			int num = int(round(len/panelWidth));
			if (num > 1) {
				double x0 = bound[i].x;
				double lenx = bound[i+1].x - bound[i].x;
				double y0 = bound[i].y;
				double leny = bound[i+1].y - bound[i].y;
				for (int in = num-1; in >= 1; --in) {
					Pointf p(x0 + lenx*in/num, y0 + leny*in/num);
					bound.Insert(i+1, p);
				}
			}
		}
	}
	bound.Remove(bound.size()-1);
	
	double avgx = 0, avgy = 0;
	for (Pointf &p : bound) {
		avgx += p.x;
		avgy += p.y;
	}
	Pointf avgp(avgx/bound.GetCount(), avgy/bound.GetCount());

	Array<Pointf> delp;
	delp.SetCount(bound.GetCount());
	for (int i = 0; i < bound.GetCount(); ++i) 
		delp[i] = bound[i]; 
	delp << avgp;
	
	Delaunay del;
	
	int maxnum = -1;
	while (true) {
		del.Build(delp);
		double avglen = 0, maxlen = 0;
		int maxid, maxid3, num = 0;
		for (int i = 0; i < del.GetCount(); ++i) {
			const Delaunay::Triangle &tri = del[i];
			if (tri[0] < 0 || tri[1] < 0 || tri[2] < 0)  
				continue;

			double len;
			len = sqrt(sqr(delp[tri[0]].x - delp[tri[1]].x) + sqr(delp[tri[0]].y - delp[tri[1]].y));
			avglen += len;	num++;
			if (maxlen < len) {
				maxlen = len;	maxid = i;	maxid3 = 0;
			}
			len = sqrt(sqr(delp[tri[1]].x - delp[tri[2]].x) + sqr(delp[tri[1]].y - delp[tri[2]].y));
			avglen += len;	num++;
			if (maxlen < len) {
				maxlen = len;	maxid = i;	maxid3 = 1;
			}			
			len = sqrt(sqr(delp[tri[2]].x - delp[tri[0]].x) + sqr(delp[tri[2]].y - delp[tri[0]].y));
			avglen += len;	num++;
			if (maxlen < len) {
				maxlen = len;	maxid = i;	maxid3 = 2;
			}
		}
		if (num == maxnum)
			break;
		maxnum = num;
		avglen /= num;
		if (avglen < panelWidth) 
			break;
		
		int maxid33 = maxid3 == 2 ? 0 : maxid3+1;
		
		delp << Pointf(Avg(delp[del[maxid][maxid3]].x, delp[del[maxid][maxid33]].x), Avg(delp[del[maxid][maxid3]].y, delp[del[maxid][maxid33]].y));	
	}
	
	Array<PanelPoints> pans;
	for (int i = 0; i < del.GetCount(); ++i) {
		const Delaunay::Triangle &tri = del[i];
		if (tri[0] < 0 || tri[1] < 0 || tri[2] < 0)  
			continue;

		PanelPoints &pan = pans.Add();
		pan.data[0].x = delp[tri[0]].x;		pan.data[0].y = delp[tri[0]].y;		pan.data[0].z = 0;
		pan.data[1].x = delp[tri[1]].x;		pan.data[1].y = delp[tri[1]].y;		pan.data[1].z = 0;
		pan.data[2].x = delp[tri[2]].x;		pan.data[2].y = delp[tri[2]].y;		pan.data[2].z = 0;
		pan.data[3].x = delp[tri[2]].x;		pan.data[3].y = delp[tri[2]].y;		pan.data[3].z = 0;
	}
	SetPanelPoints(pans);
}

Vector<Point3D> Surface::GetClosedPolygons(Vector<Segment3D> &segs) {
	Vector<Point3D> ret;
	
	if (segs.IsEmpty())
		return ret;
	
	ret << segs[0].from;
	ret << segs[0].to;
	segs.Remove(0);
	while (true) {
		const Point3D &last = ret[ret.size()-1];
		bool found = false;
		for (int i = 0; i < segs.size(); ++i) {
			if (segs[i].from == last) {
				ret << segs[i].to;
				segs.Remove(i);
				found = true;
				break;
			} else if (segs[i].to == last) {
				ret << segs[i].from;
				segs.Remove(i);
				found = true;
				break;
			}
		}
		if (!found) 
			return ret;
		if (ret[0] == ret[ret.size()-1])
			return ret;		// Polygon closed
		if (segs.IsEmpty()) {
			ret.Clear();	// Polygon unclosed
			return ret;
		}
	}
}
		
Array<Pointf> Surface::Point3dto2D(const Vector<Point3D> &bound) {
	Array<Pointf> ret;
	for (const auto &d: bound)
		ret << Pointf(d.x, d.y);
	return ret;
}

int Find(Vector<Segment3D> &segs, const Point3D &from, const Point3D &to) {
	for (int is = 0; is < segs.size(); ++is) {
		if (segs[is].from == from && segs[is].to == to)
			return is;
		if (segs[is].from == to && segs[is].to == from)
			return is;
	}
	return -1;
}

void DeleteVoidSegments(Vector<Segment3D> &segs) {
	for (int i = segs.size()-1; i >= 0; --i) {
		const Segment3D &seg = segs[i];
		if (seg.from == seg.to)
			segs.Remove(i);
	}
}	

void DeleteDuplicatedSegments(Vector<Segment3D> &segs) {
	for (int i = 0; i < segs.size(); ++i) {
		const Segment3D &seg0 = segs[i];
		for (int j = segs.size()-1; j > i; --j) {
			const Segment3D &seg = segs[j];	
			if (seg0.from == seg.from && seg0.to == seg.to)
				segs.Remove(j);
			else if (seg0.from == seg.to && seg0.to == seg.from)
				segs.Remove(j);
		}
	}
}

bool Surface::GetDryPanels(const Surface &orig, bool onlywaterplane) {
	nodes = clone(orig.nodes);
	panels.Clear();
	
	for (const auto &pan : orig.panels) {
		const int &id0 = pan.id[0];
		const int &id1 = pan.id[1];
		const int &id2 = pan.id[2];
		const int &id3 = pan.id[3];
		const Point3D &p0 = nodes[id0];
		const Point3D &p1 = nodes[id1];
		const Point3D &p2 = nodes[id2];
		const Point3D &p3 = nodes[id3];	
		
		if (p0.z >= -EPS_LEN && p1.z >= -EPS_LEN && p2.z >= -EPS_LEN && p3.z >= -EPS_LEN) { 
			if (!onlywaterplane || (p0.z <= EPS_LEN && p1.z <= EPS_LEN && p2.z <= EPS_LEN && p3.z <= EPS_LEN))
				panels << clone(pan);
		}
	}
	Heal(true);	
	
	return !panels.IsEmpty();
}

Vector<Segment3D> Surface::GetWaterLineSegments(const Surface &orig) {
	Vector<Segment3D> ret;

	for (const auto &pan : orig.panels) {
		const int &id0 = pan.id[0];
		const int &id1 = pan.id[1];
		const int &id2 = pan.id[2];
		const int &id3 = pan.id[3];
		const Point3D &p0 = orig.nodes[id0];
		const Point3D &p1 = orig.nodes[id1];
		const Point3D &p2 = orig.nodes[id2];
		const Point3D &p3 = orig.nodes[id3];	
		
		if (p0 != p1 && p0.z >= -EPS_LEN && p1.z >= -EPS_LEN && Find(ret, p0, p1) < 0)
			ret << Segment3D(p0, p1);
		if (p1 != p2 && p1.z >= -EPS_LEN && p2.z >= -EPS_LEN && Find(ret, p1, p2) < 0)
			ret << Segment3D(p1, p2);
		if (p2 != p3 && p2.z >= -EPS_LEN && p3.z >= -EPS_LEN && Find(ret, p2, p3) < 0)
			ret << Segment3D(p2, p3);
		if (p3 != p0 && p3.z >= -EPS_LEN && p0.z >= -EPS_LEN && Find(ret, p3, p0) < 0)
			ret << Segment3D(p3, p0);
	}
	return ret;
}

void Surface::AddWaterSurface(Surface &surf, const Surface &under, char c) {
	if (c == 'f') {				// Takes the underwater limit from under and fills inside it
		if (surf.surface == 0)
			return;

		Vector<Segment3D> segs = GetWaterLineSegments(under);
		if (segs.IsEmpty())
			throw Exc(t_("There is no water piercing in this mesh"));
		
		surf.GetSegments();
		double panelWidth = surf.GetAvgLenSegment();
		
		while (!segs.IsEmpty()) {
			Vector<Point3D> bound = GetClosedPolygons(segs);
			if (bound.IsEmpty())
				break;
			Array<Pointf> bound2D = Point3dto2D(bound);
			if (bound2D.size() > 2)
				AddPolygonalPanel2(bound2D, panelWidth*1.2, false);
		}
	} else if (c == 'r') {		// Copies only the underwater side
		if (under.panels.IsEmpty())
			throw Exc(t_("There is no submerged mesh"));
		
		panels = clone(under.panels);
		nodes = clone(under.nodes);
	} else if (c == 'e') { 		// Copies only the dry and waterline side
		if (!GetDryPanels(surf, false))
			throw Exc(t_("There is no mesh in and above the water surface"));		
	} else if (c == 'w') { 		// Copies only the waterline side
		if (!GetDryPanels(surf, true))
			throw Exc(t_("There is no mesh in the water surface"));		
	}
}

char Surface::IsWaterPlaneMesh() const {
	bool waterplane = false, outwaterplane = false;
	for (const auto &pan : panels) {
		const int &id0 = pan.id[0];
		const int &id1 = pan.id[1];
		const int &id2 = pan.id[2];
		const int &id3 = pan.id[3];
		const Point3D &p0 = nodes[id0];
		const Point3D &p1 = nodes[id1];
		const Point3D &p2 = nodes[id2];
		const Point3D &p3 = nodes[id3];	
		
		int numwaterplane = 0, num = 0;
		if (p0 != p1) {
			num++;
			if (p0.z >= -EPS_LEN && p1.z >= -EPS_LEN && p0.z <= EPS_LEN && p1.z <= EPS_LEN)
				numwaterplane++;
		}
		if (p1 != p2) {
			num++;
			if (p1.z >= -EPS_LEN && p2.z >= -EPS_LEN && p1.z <= EPS_LEN && p2.z <= EPS_LEN)
				numwaterplane++;
		}
		if (p2 != p3) {
			num++;
			if (p2.z >= -EPS_LEN && p3.z >= -EPS_LEN && p2.z <= EPS_LEN && p3.z <= EPS_LEN)
				numwaterplane++;
		}
		if (p3 != p0) {
			num++;
			if (p3.z >= -EPS_LEN && p0.z >= -EPS_LEN && p3.z <= EPS_LEN && p0.z <= EPS_LEN)
				numwaterplane++;
		}
		if (num == numwaterplane)
			waterplane = true;
		else
			outwaterplane = true;
	}	
	if (waterplane && !outwaterplane)
		return 'y';
	else if (!waterplane && outwaterplane)
		return 'n';
	else
		return 'x';	
}

ForceVector &ForceVector::TransRot(double dx, double dy, double dz, double ax, double ay, double az, double cx, double cy, double cz) {
	Affine3d aff;
	GetTransform(aff, dx, dy, dz, ax, ay, az, cx, cy, cz);
	TransRot(aff);
	return *this;
}

ForceVector &ForceVector::TransRot(const Affine3d &aff) {
	point.TransRot(aff);
	force.t.TransRot(aff);
	return *this;
}

// Just force, no moment
void Force6D::AddLinear(const Direction3D &dir, const Point3D &point, const Point3D &c0) {
	Direction3D R = point - c0;
	Direction3D M = R%dir;
	t += dir;
	r += M;							// No moment added, just the generated by the leverage arm of the force
}
	
void Force6D::Add(const Force6D &force, const Point3D &point, const Point3D &c0) {
	Direction3D R = point - c0;
	Direction3D M = R%force.t;
	t += force.t;
	r += M + force.r;
}

void Force6D::Add(const ForceVector &fv, const Point3D &c0) {
	Direction3D R = fv.point - c0;	// Leverage arm
	Direction3D M = R%fv.force.t;	// Moment generated by the leverage arm of the force
	t += fv.force.t;
	r += M + fv.force.r;
}

VectorXd C6ToVector(const double *c) {
	VectorXd v(6);
	std::copy(c, c+6, v.data());
	return v;
}

VectorXd C6ToVector(const float *c) {
	VectorXd v(6);
	for (int i = 0; i < 6; ++i)
		v[i] = c[i];
	return v;
}

void Vector6ToC(const VectorXd &v, double *c) {
	ASSERT(v.size() == 6);
	std::copy(v.data(), v.data()+6, c);
}

void Vector6ToC(const VectorXd &v, float *c) {
	ASSERT(v.size() == 6);
	for (int i = 0; i < 6; ++i)
		c[i] = float(v[i]);
}

}
