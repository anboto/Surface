// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2026, the Anboto author and contributors
#include <Core/Core.h>
#include "Surface.h"
#include <STEM4U/Utility.h>

namespace Upp {

// Imperfect comparison
bool Surface::CompareDistance(const Surface &b, double eps, Value3D &distance) const {
	if (b.nodes.size() != nodes.size() || b.panels.size() != panels.size() || b.segments.size() != segments.size() || b.lines.size() != lines.size())
		return false;
	
	Point3D centroid = Point3D::Zero(), centroidb = Point3D::Zero();
	for (int i = 0; i < nodes.size(); ++i) {
		centroid  += nodes[i];
		centroidb += b.nodes[i];
	}
	centroid  /= nodes.size();
	centroidb /= b.nodes.size();
	distance = centroidb - centroid;
	
	Surface a2 = clone(*this), b2 = clone(b);
	b2.Translate(-distance);
	
	auto GetBasicPanelParams = [](Surface &surf) {
		for (int i = 0; i < surf.panels.size(); ++i) {
			Panel &p = surf.panels[i];
			const Point3D &p0 = surf.nodes[p.id[0]];
			const Point3D &p1 = surf.nodes[p.id[1]];
			const Point3D &p2 = surf.nodes[p.id[2]];
			const Point3D &p3 = surf.nodes[p.id[3]];
			
			p.surface0  = Area(p0, p1, p2);
			p.centroid0 = Centroid(p0, p1, p2);
			if (!p.IsTriangle()) {
				p.surface1  = Area(p2, p3, p0);
				p.centroid1 = Centroid(p2, p3, p0);
				double surf = p.surface0 + p.surface1;
				if (surf == 0)
					p.centroidPaint = (p.centroid0 + p.centroid1)/surf;
				else if (p.surface0 < 1e-10)
					p.centroidPaint = clone(p.centroid1);
				else if (p.surface1 < 1e-10)
					p.centroidPaint = clone(p.centroid0);
				else
					p.centroidPaint = (p.centroid0*p.surface0 + p.centroid1*p.surface1)/surf;
			} else {
				p.surface1 = 0;
				p.centroidPaint = p.centroid0;
			}
		}		
	};
	
	GetBasicPanelParams(a2);
	GetBasicPanelParams(b2);
	Vector<int> idAvailable_b2;
	Arange(idAvailable_b2, 0, a2.panels.size()-1, 1);
	
	double eps2 = sqr(eps);
	
	for (int ip = 0; ip < a2.panels.size(); ++ip) {		// Compare centroids and panel surface
		double surf = a2.panels[ip].surface0 + a2.panels[ip].surface1;
		int idFound = -1;
		for (int i = 0; i < idAvailable_b2.size(); ++i) {
			int id = idAvailable_b2[i];
			if (a2.panels[ip].centroidPaint.CompareDelta(b2.panels[id].centroidPaint, eps) && 
				abs(surf - (b2.panels[id].surface0 + b2.panels[id].surface1)) <= eps2) {
				idFound = i;
				break;
			}
		}
		if (idFound < 0)
			return false;
		idAvailable_b2.Remove(idFound);
	}			
	return true;	
}

}