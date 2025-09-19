// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors   
#include <Core/Core.h>
#include <Surface/Surface.h>


#include <Functions4U/EnableWarnings.h>

namespace Upp {
using namespace Eigen;

Color BrightenColor(const Color &color, int intensity) {
    float factor = intensity / 255.0f;		// Normalize intensity to the range [0.0, 1.0]

    int r = min(255, static_cast<int>(color.GetR() * factor + 255 * (1 - factor)));
    int g = min(255, static_cast<int>(color.GetG() * factor + 255 * (1 - factor)));
    int b = min(255, static_cast<int>(color.GetB() * factor + 255 * (1 - factor)));
    
    return Color(r, g, b);
}

Color DarkenColor(const Color &color, int intensity) {
    float factor = intensity / 255.0f;		// Normalize intensity to the range [0.0, 1.0]

    int r = static_cast<int>(color.GetR() * factor);
    int g = static_cast<int>(color.GetG() * factor);
    int b = static_cast<int>(color.GetB() * factor);
    
    return Color(r, g, b);
}

Color BlendColor(const Color &object, const Color &light, int intensity) {
    float factor = intensity / 255.0f;

    int r = min(255, static_cast<int>(object.GetR() * (1 - factor) + light.GetR() * factor));
    int g = min(255, static_cast<int>(object.GetG() * (1 - factor) + light.GetG() * factor));
    int b = min(255, static_cast<int>(object.GetB() * (1 - factor) + light.GetB() * factor));
    
    return Color(r, g, b);
}

void SurfaceView::Render(const Affine3d &quat) {
	nodesRot.SetCount(nodes0.size());
	std::atomic<int> ii(0);
	CoDo([&] {
		for(int in = ii++; in < nodes0.size(); in = ii++)
			nodesRot[in] = quat * nodes0[in];
	});

	colors.SetCount(items.size());
	
	ii = 0;
	CoDo([&] {
		for (int ip = ii++; ip < items.size(); ip = ii++) {	// Get the colours
			ItemView &pv = items[ip];
		
			if (pv.numIds == 1)
				pv.centroid = nodesRot[pv.id[0]];
			else if (pv.numIds == 2) {
				const Point3D &p0 = nodesRot[pv.id[0]];
				const Point3D &p1 = nodesRot[pv.id[1]];
		
				pv.centroid = Middle(p0, p1);
			} else {	
				const Point3D &p0 = nodesRot[pv.id[0]];
				const Point3D &p1 = nodesRot[pv.id[1]];
				const Point3D &p2 = nodesRot[pv.id[2]];
		
				pv.normal = Normal(p0, p1, p2);				// To simplify, triangles and quads have same normal and centroid
				pv.centroid = Centroid(p0, p1, p2);
				
		 		int c = (int) abs(lightDir.dot(pv.normal) * 255.0);
			 	if (showColor == SHOW_BRIGHTER)
			 		colors[ip] = BrightenColor(pv.color, c);
			 	else if (showColor == SHOW_DARKER)
			 		colors[ip] = DarkenColor(pv.color, c);
			 	else if (showColor == SHOW_FLAT)
			 		colors[ip] = pv.color;
			 	else
			 		colors[ip] = Color(c, c, c);
			}
			if (pv.overunder == 'o')
				pv.centroid.z = UNDER_ALL;		// Over all (yes, it is opposite)
			else if (pv.overunder == 'u')
				pv.centroid.z = OVER_ALL;		// Under all	
			}
	});
	if (sort)
	    order = GetCoSortOrderX(items, [](const ItemView& p1, const ItemView& p2) {	// Sort by Z depth
	        return p1.centroid.z > p2.centroid.z;		
	    });
    
    GetEnvelope2D();
}

bool SurfaceView::GetEnvelope2D() {
	if (nodesRot.IsEmpty())
		return false;
	
	env2D.minX = env2D.minY = std::numeric_limits<double>::max();
	env2D.maxX = env2D.maxY = std::numeric_limits<double>::lowest();
	
	for (const Vector3d &p : nodesRot) {
		env2D.minX = min(p[0], env2D.minX);
		env2D.minY = min(p[1], env2D.minY);
		env2D.maxX = max(p[0], env2D.maxX);
		env2D.maxY = max(p[1], env2D.maxY);
	}
	return true;
}
	
Image SurfaceView::GetImage(Size sz, double scale, double dx, double dy, const Affine3d &rot) {
	ImageBuffer ib(sz);	
	BufferPainter bp(ib, MODE_ANTIALIASED);	
	
	bp.LineCap(LINECAP_SQUARE);
	bp.LineJoin(LINEJOIN_MITER);
	
	Render(rot);
	Paint(bp, sz, scale, dx, dy);

	return ib;				       
}

void SurfaceView::Clear() {
	nodes0.Clear();
	nodesRot.Clear();
	items.Clear();
	colors.Clear();
	order.Clear();
	surfs.Clear();
	strings.Clear();
}

const SurfaceView &SurfaceView::ZoomToFit(Size sz, double &scale, Pointf &pos) const { 
	if (env2D.IsNull2D())
		return *this;
	
	double scaleX = sz.cx/(env2D.maxX - env2D.minX);
	double scaleY = sz.cy/(env2D.maxY - env2D.minY);
	double cx = avg(env2D.maxX, env2D.minX);
	double cy = avg(env2D.maxY, env2D.minY);
	
	scale = scaleX < scaleY ? scaleX : scaleY;

	pos.x = -cx*scale;
	pos.y =  cy*scale;

	return *this;
}
			
void SurfaceView::SelectPoint(Point pmouse, Size sz, double scale, double dx, double dy, int &idBody, int &idSubBody, bool select) {
	Pointf point;
	
	dx += sz.cx/2;
	dy = sz.cy/2 - dy;
	
	point.x = (pmouse.x - dx)/scale;
	point.y = (dy - pmouse.y)/scale;

	auto DoPaint = [&](const ItemView &p) {
		if (p.numIds == 3) {
			const Point3D &n0 = nodesRot[p.id[0]];
			const Point3D &n1 = nodesRot[p.id[1]];
			const Point3D &n2 = nodesRot[p.id[2]];
			if (ContainsPoint<double>(Pointf(n0.x, -n0.y), Pointf(n1.x, -n1.y), Pointf(n2.x, -n2.y), point)) {
				idBody = p.idBody;
				idSubBody = p.idSubBody;
				Surface *s = surfs.Get(idBody);
				if (s->IsValid()) {
					if (select)
						s->AddSelPanel(idSubBody);
					else
						s->RemoveSelPanel(idSubBody);
				}
				return;
			}
		} else if (p.numIds == 4) {
			const Point3D &n0 = nodesRot[p.id[0]];
			const Point3D &n1 = nodesRot[p.id[1]];
			const Point3D &n2 = nodesRot[p.id[2]];
			const Point3D &n3 = nodesRot[p.id[3]];
			if (ContainsPoint<double>(Pointf(n0.x, -n0.y), Pointf(n1.x, -n1.y), Pointf(n2.x, -n2.y), Pointf(n3.x, -n3.y), point)) {
				idBody = p.idBody;
				idSubBody = p.idSubBody;
				Surface *s = surfs.Get(idBody);
				if (s->IsValid()) {
					if (select)
						s->AddSelPanel(idSubBody);
					else
						s->RemoveSelPanel(idSubBody);
				}
				return;
			}
		}
	};
	
	if (sort) {
		for (int io = order.size()-1; io >= 0; --io)
			DoPaint(items[order[io]]);
	} else {
		for (int io = 0; io < items.size(); ++io) 
			DoPaint(items[io]);
	}
	
	if (deselectIfClickOutside) {
		for(int i = 0; i < surfs.size(); i++) {	// Clear all selected panels
			if (surfs[i]->IsValid()) 
				surfs[i]->ClearSelPanels();
		}
	}
	idBody = idSubBody = -1;
}

SurfaceView &SurfaceView::PaintSurface(Surface &surf, Color color, Color meshColor, double thick, int idBody, bool showNormals, double normalLen) {
	surfs.Add(idBody, &surf);
	
	Point3D clightDir = clone(lightDir).Normalize();
	
	int nnodes = nodes0.size();
	nodes0.SetCountR(nnodes + surf.nodes.size());
	for (int in = 0; in < surf.nodes.size(); ++in) {
		const Vector3D &n = surf.nodes[in];
		Vector3d &nn = nodes0[in + nnodes];
		nn[0] = n.x;
		nn[1] = n.y;
		nn[2] = n.z;
	}
	Vector<bool> selPanels(surf.panels.size(), false);
	for (int ip : surf.GetSelPanels())
		selPanels[ip] = true;
	
	int nitems = items.size();
	items.SetCountR(nitems + surf.panels.size());
	bool paintArrow2 = nitems + surf.panels.size() < 30000;
	for (int ip = 0; ip < surf.panels.size(); ++ip) {
		const Panel &p = surf.panels[ip];
		ItemView &pv = items[nitems + ip];
		
		pv.numIds = p.IsTriangle() ? 3 : 4;
		for (unsigned i = 0; i < pv.numIds; ++i)
			pv.id[i] = p.id[i] + nnodes;
		pv.meshColor = meshColor;
		if (!selPanels[ip]) {
			pv.color = color;
			pv.thickness = thick;
		} else {
			pv.color = Red();
			pv.thickness = 4*thick;
		}
		pv.idBody = idBody;
		pv.idSubBody = ip;
		if (showNormals) {
			const Point3D &p0 = nodes0[pv.id[0]];
			const Point3D &p1 = nodes0[pv.id[1]];
			const Point3D &p2 = nodes0[pv.id[2]];
	
			pv.normal = Normal(p0, p1, p2);		// To simplify, triangles and quads have same normal and centroid
			
			if (pv.numIds == 3)
				pv.centroid = Centroid(p0, p1, p2);
			else {
				const Point3D &p3 = nodes0[pv.id[3]];
				
				Point3D c0 = Centroid(p0, p1, p2);
				Point3D c1 = Centroid(p2, p3, p0);
				double area = p.surface0 + p.surface1;
				pv.centroid.x = (c0.x*p.surface0 + c1.x*p.surface1)/area;
				pv.centroid.y = (c0.y*p.surface0 + c1.y*p.surface1)/area;
				pv.centroid.z = (c0.z*p.surface0 + c1.z*p.surface1)/area;
			}
			Segment3D seg(pv.centroid, pv.normal, normalLen);	
			if (paintArrow2)		
				PaintArrow2(seg.from, seg.to, Blue(), -1);
			else
				PaintLine(seg.from, seg.to, Blue(), lineThickness, -1);
		}
	}
	return *this;
}

SurfaceView &SurfaceView::PaintText(double x, double y, double z, const char *str, double where) {
	Vector3d &node = nodes0.Add();
	node.x() = x;
	node.y() = y;
	node.z() = z;
	
	ItemView &pv = items.Add();
	
	pv.numIds = 1;
	pv.id[0] = nodes0.size()-1;
	
	strings << str;
	pv.idString = strings.size()-1;
		
	if (where == SurfaceView::OVER_ALL)
		pv.overunder = 'o';
	else if (where == SurfaceView::UNDER_ALL)
		pv.overunder = 'u';
	else
		pv.overunder = '\0';
	return *this;
}

SurfaceView &SurfaceView::PaintLine(const Point3D &p0, const Point3D &p1, const Color &color, double thick, double lenDelta) {
	ASSERT(!IsNull(lenDelta));
	
	double len = Distance(p0, p1);
	Vector<Point3D> points;
	points << p0;
	if (sort && lenDelta > SurfaceView::UNDER_ALL && lenDelta < SurfaceView::OVER_ALL) {
		int num;
		if (lenDelta > 0)
			num = int(len/lenDelta);
		else
			num = max(1, -int(lenDelta));
		double dx = (p1.x - p0.x)/num;
		double dy = (p1.y - p0.y)/num;
		double dz = (p1.z - p0.z)/num;
		for (int i = 1; i < num; ++i)
			points << Point3D(p0.x + dx*i, p0.y + dy*i, p0.z + dz*i);
		
	}
	points << p1;	

	int nnodes = nodes0.size();
	nodes0.SetCountR(nnodes + points.size());
	for (int ip = 0; ip < points.size(); ++ip)
		nodes0[nnodes + ip] = points[ip];
	int nitems = items.size();
	items.SetCountR(nitems + points.size()-1);
	for (int ip = 0; ip < points.size()-1; ++ip) {
		ItemView &pv = items[nitems + ip];
		
		pv.numIds = 2;
		pv.id[0] = nnodes + ip;
		pv.id[1] = nnodes + ip+1;
		pv.meshColor = color;
		pv.thickness = thick;
		if (lenDelta == SurfaceView::OVER_ALL)
			pv.overunder = 'o';
		else if (lenDelta == SurfaceView::UNDER_ALL)
			pv.overunder = 'u';
		else
			pv.overunder = '\0';
	}
	return *this;
}

SurfaceView &SurfaceView::PaintLine(double x0, double y0, double z0, double x1, double y1, double z1, const Color &color, double thick, double lenDelta) {
	Point3D p0(x0, y0, z0);
	Point3D p1(x1, y1, z1);
	if (!IsNull(p0) && !IsNull(p1))
		PaintLine(p0, p1, color, thick, lenDelta);
	return *this;
}

SurfaceView &SurfaceView::PaintLine(const Segment3D &p, const Color &color, double thick, double lenDelta) {
	if (!IsNull(p.from) && !IsNull(p.to))
		PaintLine(p.from, p.to, color, thick, lenDelta);
	return *this;
}

SurfaceView &SurfaceView::PaintSegments(const Vector<Segment3D>& segs, const Color &color, double lenDelta) {
	for (int i = 0; i < segs.size(); ++i) 
		PaintLine(segs[i], color, lineThickness, lenDelta);
	return *this;
}

SurfaceView &SurfaceView::PaintLines(const Vector<Point3D>& line, const Color &color, double thick, double lenDelta) {
	if (IsNull(thick))
		thick = lineThickness*3;
	for (int i = 0; i < line.size()-1; ++i)
		PaintLine(line[i], line[i+1], color, thick, lenDelta);
	return *this;
}

SurfaceView &SurfaceView::PaintLines(const Vector<double>& x, const Vector<double>& y, const Vector<double>& z, const Color &color, double thick, double lenDelta) {
	ASSERT(x.size() == y.size() && y.size() == z.size());
	if (IsNull(thick))
		thick = lineThickness*3;
	for (int i = 0; i < x.size()-1; ++i)
		PaintLine(x[i], y[i], z[i], x[i+1], y[i+1], z[i+1], color, thick, lenDelta);
	return *this;
}

SurfaceView &SurfaceView::PaintMesh(const Point3D &p0, const Point3D &p1, const Point3D &p2, const Point3D &p3, const Color &linCol) {
	PaintLine(p0, p1, linCol, lineThickness);
	PaintLine(p1, p2, linCol, lineThickness);
	PaintLine(p2, p3, linCol, lineThickness);
	PaintLine(p3, p0, linCol, lineThickness);
	return *this;
}

SurfaceView &SurfaceView::PaintSegments(const Surface &surf, const Color &linCol, double lenDelta) {
	for (int is = 0; is < surf.segments.size(); ++is) {
		const LineSegment &seg = surf.segments[is];
		
		const Point3D &p0 = surf.nodes[seg.idNod0];
		const Point3D &p1 = surf.nodes[seg.idNod1];
		PaintLine(p0, p1, linCol, lineThickness, lenDelta);
	}
	return *this;
}

SurfaceView &SurfaceView::PaintCuboid(const Point3D &p0, const Point3D &p1, const Color &color, double lenDelta) {
	PaintLine(p0.x, p0.y, p0.z, p0.x, p0.y, p1.z, color, lineThickness, lenDelta);
	PaintLine(p0.x, p0.y, p1.z, p0.x, p1.y, p1.z, color, lineThickness, lenDelta);
	PaintLine(p0.x, p1.y, p1.z, p0.x, p1.y, p0.z, color, lineThickness, lenDelta);
	PaintLine(p0.x, p1.y, p0.z, p0.x, p0.y, p0.z, color, lineThickness, lenDelta);
	
	PaintLine(p0.x, p0.y, p0.z, p1.x, p0.y, p0.z, color, lineThickness, lenDelta);
	PaintLine(p1.x, p0.y, p0.z, p1.x, p0.y, p1.z, color, lineThickness, lenDelta);
	PaintLine(p1.x, p0.y, p1.z, p1.x, p1.y, p1.z, color, lineThickness, lenDelta);
	PaintLine(p1.x, p1.y, p1.z, p1.x, p1.y, p0.z, color, lineThickness, lenDelta);
	
	PaintLine(p1.x, p1.y, p0.z, p1.x, p0.y, p0.z, color, lineThickness, lenDelta);
	PaintLine(p1.x, p0.y, p0.z, p1.x, p0.y, p1.z, color, lineThickness, lenDelta);
	PaintLine(p1.x, p0.y, p1.z, p0.x, p0.y, p1.z, color, lineThickness, lenDelta);
	PaintLine(p0.x, p0.y, p1.z, p0.x, p1.y, p1.z, color, lineThickness, lenDelta);
	
	PaintLine(p0.x, p1.y, p1.z, p1.x, p1.y, p1.z, color, lineThickness, lenDelta);
	PaintLine(p1.x, p1.y, p1.z, p1.x, p1.y, p0.z, color, lineThickness, lenDelta);
	PaintLine(p1.x, p1.y, p0.z, p0.x, p1.y, p0.z, color, lineThickness, lenDelta);
	PaintLine(p0.x, p1.y, p0.z, p0.x, p0.y, p0.z, color, lineThickness, lenDelta);
		
	return *this;
}

SurfaceView &SurfaceView::PaintGrid(const Point3D &p0, const Point3D &p1, const Color &color, double lenDelta) {
	static int numtiles = 20;
	double minx, maxx, miny, maxy, minz, maxz;
	if (p0.x > p1.x) {
		maxx = p0.x;
		minx = p1.x;
	} else {
		minx = p0.x;
		maxx = p1.x;
	}
	if (p0.y > p1.y) {
		maxy = p0.y;
		miny = p1.y;
	} else {
		miny = p0.y;
		maxy = p1.y;
	}
	if (p0.z > p1.z) {
		maxz = p0.z;
		minz = p1.z;
	} else {
		minz = p0.z;
		maxz = p1.z;
	}
	double dx = maxx - minx,
		   dy = maxy - miny,
		   dz = maxz - minz;
	if (dx < 0.0001) {
		double d = max(dz, dy)/numtiles;
		for (double z = minz; z <= maxz; z += d)	
			PaintLine(minx, miny, z, minx, maxy, z, color, lineThickness, lenDelta);
		for (double y = miny; y <= maxy; y += d)
			PaintLine(minx, y, minz, minx, y, maxz, color, lineThickness, lenDelta);	
	} else if (dy < 0.0001) {
		double d = max(dx, dz)/numtiles;
		for (double x = minx; x <= maxx; x += d)	
			PaintLine(x, miny, minz, x, miny, maxz, color, lineThickness, lenDelta);
		for (double z = minz; z <= maxz; z += d)
			PaintLine(minx, miny, z, maxx, miny, z, color, lineThickness, lenDelta);	
	} else if (dz < 0.0001) {
		double d = max(dx, dy)/numtiles;
		for (double x = minx; x <= maxx; x += d)	
			PaintLine(x, miny, minz, x, maxy, minz, color, lineThickness, lenDelta);
		for (double y = miny; y <= maxy; y += d)
			PaintLine(minx, y, minz, maxx, y, minz, color, lineThickness, lenDelta);	
	}
	
	return *this;
}

SurfaceView &SurfaceView::PaintAxis(const Point3D &p, double len, double lenDelta) {
	return PaintAxis(p.x, p.y, p.z, len, lenDelta);
}

SurfaceView &SurfaceView::PaintAxis(double x, double y, double z, double len, double lenDelta) {
	PaintArrow(x, y, z, x+len, y	 , z,     LtRed(),   lenDelta);
	PaintArrow(x, y, z, x	 , y+len , z,     LtGreen(), lenDelta);
	PaintArrow(x, y, z, x	 , y	 , z+len, LtBlue(),  lenDelta);
	return *this;
}

SurfaceView &SurfaceView::PaintDoubleAxis(const Point3D &p, double len, const Color &color, double lenDelta) {
	return PaintDoubleAxis(p.x, p.y, p.z, len, color, lenDelta);
}

SurfaceView &SurfaceView::PaintDoubleAxis(double x, double y, double z, double len, const Color &color, double lenDelta) {
	PaintLine(x-len/2, y	  , z	   , x+len/2, y	     , z	  , color, lineThickness, lenDelta);
	PaintLine(x		 , y-len/2, z	   , x		, y+len/2, z	  , color, lineThickness, lenDelta);
	PaintLine(x		 , y	  , z-len/2, x		, y	     , z+len/2, color, lineThickness, lenDelta);
	return *this;
}

SurfaceView &SurfaceView::PaintCube(const Point3D &p, double side, const Color &color, double lenDelta) {
	return PaintCube(p.x, p.y, p.z, side, color, lenDelta);
}

SurfaceView &SurfaceView::PaintCube(double x, double y, double z, double side, const Color &color, double lenDelta) {
	return PaintCuboid(Point3D(x-side/2., y-side/2., z-side/2.), Point3D(x+side/2., y+side/2., z+side/2.), color, lenDelta);
}

SurfaceView &SurfaceView::PaintArrow(double x0, double y0, double z0, double x1, double y1, double z1, const Color &color, double lenDelta) {
	return PaintArrow(Point3D(x0, y0, z0), Point3D(x1, y1, z1), color, lenDelta);
}

SurfaceView &SurfaceView::PaintArrow(const Point3D &p0, const Point3D &p1, const Color &color, double lenDelta) {
	Segment3D seg(p0, p1);
	Vector3D vector = seg.Direction().Normalize();
	double len = seg.Length();
	double lenArr = 0.8*len;
	Point3D pointTri = p0 + vector*lenArr;
	double zangle = atan2(vector.x, vector.y);
	double nangle = zangle + M_PI/2;
	
	Point3D parr1(pointTri.x + 0.1*len*sin(nangle), pointTri.y + 0.1*len*cos(nangle), pointTri.z); 
	Point3D parr2(pointTri.x - 0.1*len*sin(nangle), pointTri.y - 0.1*len*cos(nangle), pointTri.z); 
	
	PaintLine(p0,   pointTri, color, lineThickness, lenDelta);
	PaintLine(p1,   parr1, 	  color, lineThickness, IsNull(lenDelta) ? Null : lenDelta*0.1);
	PaintLine(p1,   parr2,    color, lineThickness, IsNull(lenDelta) ? Null : lenDelta*0.1);
	PaintLine(parr1,parr2,    color, lineThickness, IsNull(lenDelta) ? Null : lenDelta*0.1);
	
	return *this;
}

SurfaceView &SurfaceView::PaintArrow2(const Point3D &p0, const Point3D &p1, const Color &color, double lenDelta) {
	Segment3D seg(p0, p1);
	Vector3D vector = seg.Direction().Normalize();
	double len = seg.Length();
	double lenArr = 0.8*len;
	Point3D pointTri = p0 + vector*lenArr;
	double zangle = atan2(vector.x, vector.y);
	double nangle = zangle + M_PI/2;
	
	Point3D parr1(pointTri.x + 0.1*len*sin(nangle), pointTri.y + 0.1*len*cos(nangle), pointTri.z); 
	Point3D parr2(pointTri.x - 0.1*len*sin(nangle), pointTri.y - 0.1*len*cos(nangle), pointTri.z); 
	
	PaintLine(p0, p1, 	 color, lineThickness, lenDelta);
	PaintLine(p1, parr1, color, lineThickness, lenDelta*0.1);
	PaintLine(p1, parr2, color, lineThickness, lenDelta*0.1);
	
	return *this;
}

}