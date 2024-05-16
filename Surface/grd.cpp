// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2022, the Anboto author and contributors
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>

namespace Upp {
using namespace Eigen;


void LoadGRD(String fileName, Surface &surf, bool &y0z, bool &x0z) {
	FileInLine in(fileName);
	if (!in.IsOpen()) 
		throw Exc(Format(t_("Impossible to open file '%s'"), fileName));
	
	try {
		String line;
		LineParser f(in);	
		f.IsSeparator = [](int c)->int {return c == '\t' || c == ' ';};
		
		surf.Clear();
		
		f.GetLine(3);
		y0z =  f.GetInt(0) == 1;
		x0z =  f.GetInt(1) == 1;

		f.GetLine();
		int numPanels = f.GetInt(0);
		while(true) {
			f.GetLine_discard_empty();
			if (f.IsEof())
				break;		
			
			Panel &panel = surf.panels.Add();
			
			panel.id[0] = FindAdd(surf.nodes, Point3D(f.GetDouble(0), f.GetDouble(1), f.GetDouble(2)));
			
			f.GetLine_discard_empty();
			panel.id[1] = FindAdd(surf.nodes, Point3D(f.GetDouble(0), f.GetDouble(1), f.GetDouble(2)));
			
			f.GetLine_discard_empty();
			panel.id[2] = FindAdd(surf.nodes, Point3D(f.GetDouble(0), f.GetDouble(1), f.GetDouble(2)));
			
			f.GetLine_discard_empty();
			panel.id[3] = FindAdd(surf.nodes, Point3D(f.GetDouble(0), f.GetDouble(1), f.GetDouble(2)));			
		}
		if (surf.panels.size() != numPanels)
			throw Exc(Format(t_("The number of panels in the header (%d) does not match the number of panels in the file (%d)"), numPanels, surf.panels.size()));
	} catch (Exc e) {
		throw Exc(t_("Parsing error: ") + e);
	}
}

void SaveGRD(String fileName, Surface &surf, double g, bool y0z, bool x0z) {
	FileOut out(fileName);
	if (!out.IsOpen())
		throw Exc(Format(t_("Impossible to open '%s'\n"), fileName));	
		
	const Vector<Panel> &panels = surf.panels;
	const Vector<Point3D> &nodes = surf.nodes;
	
	out << Format("mesh '%s' saved by U++ Anboto", RemoveAccents(GetFileTitle(fileName)))
		<< Format("\n1.0   %.2f", g)
		<< Format("\n%d     %d", y0z ? 1 : 0, x0z ? 1 : 0)
		<< "\n" << panels.size();

	for (const Panel &panel : panels) {
		for (int i = 0; i < 4; ++i) {
			const Point3D &p = nodes[panel.id[i]];
			out << Format("\n%6.3f %6.3f %6.3f", p.x, p.y, p.z);
		}
	}
}

}
