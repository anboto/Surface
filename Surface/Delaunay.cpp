#include <Core/Core.h>
#include "Surface.h"
#include <Geom/Geom.h>
#include <Surface/predicates/predicates.h>

namespace Upp {

static void InitializePredicates() {
	static bool initialized = false;
	if (!initialized) {
		initialized = true;
		exactinit();
	}
}

int Orientation(const Pointf& a, const Pointf& b, const Pointf& c) {
	InitializePredicates();
	
    double pa[2] = { a.x, a.y };
    double pb[2] = { b.x, b.y };
    double pc[2] = { c.x, c.y };
    double v = orient2d(pa, pb, pc);
    return (v > 0) - (v < 0); // +1 (CCW), 0, -1 (CW)
}

bool InCircle(const Pointf& a, const Pointf& b, const Pointf& c, const Pointf& d) {
	InitializePredicates();
	
    double pa[2] = { a.x, a.y };
    double pb[2] = { b.x, b.y };
    double pc[2] = { c.x, c.y };
    double pd[2] = { d.x, d.y };

    double o = orient2d(pa, pb, pc);
    double v;

    if(o > 0)
        v = incircle(pa, pb, pc, pd);
    else if(o < 0)
        v = incircle(pa, pc, pb, pd);
    else
        return false;

    if(v > 0) 
    	return true;
    if(v < 0) 
    	return false;

    // exact cocircular → reject flip
    return false;
}

inline void Delaunay2::Link(int ta, int ia, int tb, int ib) {
	triangles[ta].SetNext(ia, tb, ib);
	triangles[tb].SetNext(ib, ta, ia);
}

bool Delaunay2::IsBadTriangle(const Pointf& A, const Pointf& B, const Pointf& C, double ratio) {
    double l0 = Squared(A - B);
    double l1 = Squared(B - C);
    double l2 = Squared(C - A);

    double mx = max(l0, max(l1, l2));
    double mn = min(l0, min(l1, l2));

    return mn/mx < ratio;	
}

bool Delaunay2::IsBadTriangle(const Triangle& t, double ratio) const {
    if(!t.IsProper())
		return false;

	return IsBadTriangle(At(t, 0), At(t, 1), At(t, 2), ratio);
}

void Delaunay2::Build(const Vector<Pointf>& p, double e) {
	InitializePredicates();
	
	epsilon = e;
	points <<= p;
	
	order = GetSortOrder(points, [](const Pointf& a, const Pointf& b) {return a.x < b.x || (a.x == b.x && a.y < b.y);});
	
	tihull = -1;
	int npoints = p.GetCount();

	triangles.Clear();
	if (order.IsEmpty())
		return;
	const Pointf *p0 = &points[order[0]];
	int xi = 0;
	do
		if(++xi >= points.GetCount())
			return;
	while (IsNear(*p0, points[order[xi]]));

	// pass 1: create pair of improper triangles
	CreatePair(order[0], order[xi]);
	while(++xi < npoints)
		AddHull(order[xi]);
	
	int clean = 0;
	do {
		int old_clean = clean;
		clean = triangles.GetCount();
		for (int i = clean; --i >= old_clean;)
			if (triangles[i].IsProper()) {
				Triangle& t = triangles[i];
				for (int x = 0; x < 3; x++) {
					int j = t.Next(x);
					Triangle& u = triangles[j];
					if (u.IsProper()) {
						int x1 = x + 1, 
							x2 = x + 2;
						if (x1 >= 3) 
							x1 -= 3;
						if (x2 >= 3) 
							x2 -= 3;
						Pointf A = At(t, x);
						Pointf B = At(t, x1);
						Pointf C = At(t, x2);
						int y = t.NextIndex(x);
						Pointf D = At(u, y);
						
						if (InCircle(A, B, C, D)) { // not locally Delaunay, flip
							int y1 = y + 1, 
								y2 = y + 2;
							if (y1 >= 3) 
								y1 -= 3;
							if (y2 >= 3) 
								y2 -= 3;

							Triangle ot = t;
							Triangle ou = u;
							
							t.Set(ot[x1], ou[y], ot[x]);
							u.Set(ou[y1], ot[x], ou[y]);
							
							Link(i, 0, j, 0);
							Link(i, 1, ot.Next(x2), ot.NextIndex(x2));
							Link(i, 2, ou.Next(y1), ou.NextIndex(y1));
							Link(j, 1, ou.Next(y2), ou.NextIndex(y2));
							Link(j, 2, ot.Next(x1), ot.NextIndex(x1));
							
							clean = i;
						}
					}
				}
			}
	} while(clean < triangles.GetCount());
}

void Delaunay2::CreatePair(int i, int j) {
	int ia = triangles.GetCount(), ib = ia + 1;
	triangles.Add().Set(-1, i, j);
	triangles.Add().Set(-1, j, i);
	Link(ia, 0, ib, 0);
	Link(ia, 1, ib, 2);
	Link(ia, 2, ib, 1);

	tihull = ia;
}

void Delaunay2::AddHull(int i) {
	Pointf newpt = points[i];
	int hi = tihull;
	int vf = -1, vl = -1;
	bool was_out = true, fix_out = false;
	int im = -1;
	double nd2 = 1e300;
	do {
		const Triangle& t = triangles[hi];
		Pointf t1 = At(t, 1), t2 = At(t, 2);//, tm = (t1 + t2)/2;
		if (t1.x == newpt.x && t1.y == newpt.y)
			return; // too close
		double d2 = Squared(t1 - newpt);
		if (d2 < nd2) {
			im = hi;
			nd2 = d2;
		}
		if (Orientation(t1, t2, newpt) > 0) {
			if (was_out)
				vf = hi;
			if (!fix_out)
				vl = hi;
			was_out = false;
		} else {
			was_out = true;
			if (vl >= 0)
				fix_out = true;
		}
		hi = t.Next(1);
	} while (hi != tihull);
	
	if (vf < 0) { // collinear, extend fan
		Triangle& tm = triangles[im];
		int in = tm.Next(2);

		int j = tm[1];

		int ia = triangles.GetCount(), ib = ia + 1;
		triangles.Add().Set(-1, i, j);
		triangles.Add().Set(-1, j, i);
		Link(ia, 0, ib, 0);
		Link(ia, 2, ib, 1);
		Link(ia, 1, im, 2);
		Link(ib, 2, in, 1);
	} else {
		Triangle& tf = triangles[vf];
		Triangle& tl = triangles[vl];
		int xfn = tf.Next(2), xln = tl.Next(1);

		int xf = triangles.GetCount(), xl = xf + 1;
		
		triangles.Add().Set(-1, tf[1], i);
		triangles.Add().Set(-1, i, tl[2]);

		tihull = xf;
		tf[0] = i;
		int f = vf;
		while (true) {
		    triangles[f][0] = i;
		    if(f == vl)
		        break;
		
		    int nf = triangles[f].Next(1);
		
		    // SAFETY: stop if hull loops or degenerates
		    if(nf == vf || nf < 0)
		        break;
		
		    f = nf;
		}

		Link(xf, 0, vf, 2);
		Link(xl, 0, vl, 1);

		Link(xf, 2, xfn, 1);
		Link(xl, 1, xln, 2);

		Link(xf, 1, xl, 2);
	}
}

Vector<int> Delaunay2::GetTouchingTriangles(int ti) const {
    Vector<int> out;

    if (ti < 0 || ti >= triangles.GetCount())
        return out;

    const Triangle& t = triangles[ti];
    if (!t.IsProper())
        return out;

    for (int e = 0; e < 3; e++) {
        int nb = t.Next(e);
        if (nb >= 0 && nb < triangles.GetCount())
            out.Add(nb);
    }
    return out;
}

}
