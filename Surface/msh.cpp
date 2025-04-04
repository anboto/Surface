// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>

#include <Functions4U/EnableWarnings.h>

namespace Upp {
using namespace Eigen;


void LoadGMSH(String fileName, Surface &surf) {
	FileInLine in(fileName);
	if (!in.IsOpen()) 
		throw Exc(Format(t_("Impossible to open file '%s'"), fileName));
	
	try {
		String line;
		LineParser f(in);	
		f.IsSeparator = [](int c)->int {return c == '\t' || c == ' ';};
		
		surf.Clear();
		
		Index<int> nodeIds;
		bool nodes = false, panels = false;
		while(true) {
			f.GetLine();
			if (f.IsEof())
				break;		
			if (f.GetText(0) == "$Nodes") {
				nodes = true;
				f.GetLine();
			} else if (f.GetText(0) == "$Elements") {
				panels = true;
				f.GetLine();
			} else if (f.GetText(0).StartsWith("$"))
				panels = nodes = false;
			else {
				if (nodes) {
					int num = f.GetInt(3);
					for (int i = 0; i < num; ++i) {
						f.GetLine();
						nodeIds << f.GetInt(0);
					}
					for (int i = 0; i < num; ++i) {
						f.GetLine();
						surf.nodes << Point3D(f.GetDouble(0), f.GetDouble(1), f.GetDouble(2));
					}
				}
				if (panels) {
					int num = f.GetInt(3);
					for (int i = 0; i < num; ++i) {
						f.GetLine();
						if (f.size() < 4)
							;
						else {
							Panel &panel = surf.panels.Add();
							panel.id[0] = nodeIds.Find(f.GetInt(1));
							panel.id[1] = nodeIds.Find(f.GetInt(2));	
							panel.id[2] = nodeIds.Find(f.GetInt(3));	
							if (f.size() < 5)
								panel.id[3] = panel.id[0];
							else
								panel.id[3] = nodeIds.Find(f.GetInt(4));
						}
					}
				}
			}
		}
	} catch (Exc e) {
		throw Exc(t_("Parsing error: ") + e);
	}
	double mx = 0;
	for (int i = 0; i < surf.nodes.size(); ++i) {
		const auto &p = surf.nodes[i];
		if (abs(p.x) > mx)
			mx = abs(p.x);
		if (abs(p.y) > mx)
			mx = abs(p.y);
		if (abs(p.z) > mx)
			mx = abs(p.z);
	}
	if (mx > 500) {		// It's guessed to be in mm. Convert to m
		for (int i = 0; i < surf.nodes.size(); ++i) {
			auto &p = surf.nodes[i];
			p.x /= 1000;
			p.y /= 1000;
			p.z /= 1000;
		}
	}
}
	
void LoadTDynMsh(String fileName, Surface &surf) {
	FileInLine in(fileName);
	if (!in.IsOpen()) 
		throw Exc(Format(t_("Impossible to open file '%s'"), fileName));
	
	try {
		String line;
		LineParser f(in);	
		f.IsSeparator = [](int c)->int {return c == '\t' || c == ' ' || c == '!';};
		
		surf.Clear();
	
		f.GetLine(2);
		
		while(true) {
			f.GetLine();
			if (f.IsEof())
				break;	
			if (f.GetText(0) == "End")
				break;
			Point3D &node = surf.nodes.Add();
			node.x = f.GetDouble(1);
			node.y = f.GetDouble(2);
			node.z = f.GetDouble(3); 
		}
		while(true) {
			f.GetLine();
			if (f.IsEof()) 
				break;
			if (f.size() > 0 && f.GetText(0) == "Elements")
				break;
		}
		while(true) {
			f.GetLine();
			if (f.IsEof())
				break;
			if (f.GetText(0) == "End")	
				break;
			Panel &panel = surf.panels.Add();
			panel.id[0] = f.GetInt(1)-1;
			panel.id[1] = f.GetInt(2)-1;	
			panel.id[2] = f.GetInt(3)-1;	
			if (f.size() < 5)
				panel.id[3] = panel.id[0];
			else
				panel.id[3] = f.GetInt(4)-1;	
		}	
	} catch (Exc e) {
		throw Exc(t_("Parsing error: ") + e);
	}
	double mx = 0;
	for (int i = 0; i < surf.nodes.size(); ++i) {
		const auto &p = surf.nodes[i];
		if (abs(p.x) > mx)
			mx = abs(p.x);
		if (abs(p.y) > mx)
			mx = abs(p.y);
		if (abs(p.z) > mx)
			mx = abs(p.z);
	}
	if (mx > 500) {		// It's guessed to be in mm. Convert to m
		for (int i = 0; i < surf.nodes.size(); ++i) {
			auto &p = surf.nodes[i];
			p.x /= 1000;
			p.y /= 1000;
			p.z /= 1000;
		}
	}
}

void LoadMesh(String fileName, Surface &surf, double &mass, Point3D &cg) {
	SurfaceMass sm;
	
	if (!LoadFromJsonFile(sm, fileName))
		throw Exc(t_("Parsing error"));
	
	surf = pick(sm.surface);
	mass = sm.mass;
	cg = pick(sm.cg);
}

void SaveMesh(String fileName, const Surface &surf, double mass, const Point3D &cg) {
	SurfaceMass sm;
	
	sm.surface = clone(surf);
	sm.mass = mass;
	sm.cg = clone(cg);
	
	if (!StoreAsJsonFile(sm, fileName, true))
		throw Exc(t_("Parsing error"));
}

}
