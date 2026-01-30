// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>


namespace Upp {
using namespace Eigen;


void LoadVTK(String fileName, Surface &surf, bool &y0z) {
	FileInLine in(fileName);
	if (!in.IsOpen()) 
		throw Exc(Format(t_("Impossible to open file '%s'"), fileName));
	
	try {
		String line;
		LineParser f(in);	
		f.IsSeparator = [](int c)->int {return c == '\t' || c == ' ';};
		
		surf.Clear();
		
		f.GetLine();
		
		f.GetLine();
		y0z = ToUpper(f.GetText(1)) == "YES";
		
		String type = Trim(f.GetLine());
		if (type != "ASCII")
			throw Exc(Format(t_("Unsupported type %s"), type));
		
		String dataset = Trim(f.GetLine());
		if (dataset != "DATASET UNSTRUCTURED_GRID")
			throw Exc(Format(t_("Unsupported type %s"), dataset));
		
		f.GetLine();
		if (f.GetText(0) != "POINTS")
			throw Exc(Format(t_("Label %s should have to be POINTS"), f.GetText(0)));
		
		while(true) {
			f.GetLine_discard_empty();
			if (f.IsEof())
				break;		
			
			if (f.GetText(0) == "CELLS")
				break;
			
			Point3D &p = surf.nodes.Add();
			
			p.x = f.GetDouble(0);
			p.y = f.GetDouble(1);
			p.z = f.GetDouble(2);
		}
		while(true) {
			f.GetLine_discard_empty();
			if (f.IsEof())
				break;		
			
			if (IsNull(f.GetInt_nothrow(0)))
				break;		
		
			Panel &panel = surf.panels.Add();
			
			int num = f.size();
			if (num == 4) {
				panel.id[0] = panel.id[3] = f.GetInt(1);
				panel.id[1] = f.GetInt(2);
				panel.id[2] = f.GetInt(3);
			} else if (num == 5) {
				panel.id[0] = f.GetInt(1);
				panel.id[1] = f.GetInt(2);
				panel.id[2] = f.GetInt(3);
				panel.id[3] = f.GetInt(4);
			} else
				throw Exc(in.Str() + ". "  + t_("Wrong number of elements"));
			
			for (int id = 0; id < 4; ++id)
				if (panel.id[id] < 0 || panel.id[id] >= surf.nodes.size())
					throw Exc(in.Str() + ". "  + t_("Wrong panel id"));
		}
	} catch (Exc e) {
		throw Exc(t_("Parsing error: ") + e);
	}
}

String StrDiffrac(double num) {
	int exponent;
	double mantissa;
	
	if (num == 0) {
		exponent = 0;
		mantissa = 0;
	} else {
		exponent = (int)floor(log10(fabs(num)));
	    mantissa = num/pow(10, exponent);
		if (fabs(mantissa) >= 10.0) {
        	mantissa /= 10;
        	exponent++;
    	}
	}
	String ret;		// To reproduce this format 4.421725E+002
	ret << (mantissa >= 0 ? " " : "-") << Format("%.6f", abs(mantissa)) << "E" << (exponent >= 0 ? "+" : "-") << Format("%03d", abs(exponent));
	return ret;
}

void SaveVTK(String fileName, Surface &surf, bool y0z) {
	FileOut out(fileName);
	if (!out.IsOpen())
		throw Exc(Format(t_("Impossible to open '%s'\n"), fileName));	
		
	const Vector<Panel> &panels = surf.panels;
	const Vector<Point3D> &nodes = surf.nodes;
	
	out << 	"# vtk DataFile Version 2.0\n"
	    << 	Format("SYMMETRY: %s STATICDEF NO\n", y0z ? "YES" : "NO")
	    << 	"ASCII\n"
			"DATASET UNSTRUCTURED_GRID\n";
			
	out <<	Format("POINTS  %6d   float\n", nodes.size());

	for (const Point3D &p : nodes)
		out << Format("  %s  %s  %s\n", StrDiffrac(p.x), StrDiffrac(p.y), StrDiffrac(p.z));
	
	int num = 0;
	for (const Panel &panel : panels)
		num += panel.IsTriangle() ? 4 : 5;
	
	out <<	Format("CELLS%6d %6d\n", panels.size(), num);
	
	for (const Panel &panel : panels) {
		if (panel.IsTriangle())
			out << Format(" %7d %7d %7d %7d\n", 	3, panel.id[0], panel.id[1], panel.id[2]);
		else
			out << Format(" %7d %7d %7d %7d %7d\n", 4, panel.id[0], panel.id[1], panel.id[2], panel.id[3]);
	}
	
	out <<	Format("CELL_TYPES   %d\n", panels.size());
	
	for (const Panel &panel : panels)
		out << "   9\n";
	
	out << "\n";
	
	out <<  Format("CELL_DATA  %d\n", panels.size())
		<<  "SCALARS module int\n"
			"LOOKUP_TABLE default\n";
			
	for (int id = 0; id < panels.size(); ++id)
		out << Format("   %d\n", surf.IsWaterPlanePanel(id) ? 2 : 0);
	
	out <<  "SCALARS hull int\n"
			"LOOKUP_TABLE default\n";
			
	for (const Panel &panel : panels)
		out << "   0\n";
}

}
