// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Geom/Geom.h>

#include <Functions4U/EnableWarnings.h>

namespace Upp {
using namespace Eigen;


void LoadOBJ(String fileName, Surface &surf) {
	FileInLine in(fileName);
	if (!in.IsOpen()) 
		throw Exc(Format(t_("Impossible to open file '%s'"), fileName));
	
	try {
		String line;
		LineParser f(in);	
		f.IsSeparator = [](int c)->int {return c == '\t' || c == ' ';};
		
		surf.Clear();
		
		while(true) {
			f.GetLine_discard_empty();
			if (f.IsEof())
				break;		

			if (f.GetText(0) == "#")
				continue;
			else if (f.GetText(0) == "v") {
				Point3D &p = surf.nodes.Add();
				p.x = f.GetDouble(1);
				p.y = f.GetDouble(2);
				p.z = f.GetDouble(3);
			} else if (f.GetText(0) == "f") {
				if (f.size() <= 5) {
					Panel &p = surf.panels.Add();
					int id1 = f.GetInt(1);
					if (id1 > 0)
						p.id[0] = id1 - 1;
					else
						p.id[0] = surf.nodes.size() +  id1;
					int id2 = f.GetInt(2);
					if (id2 > 0)
						p.id[1] = id2 - 1;
					else
						p.id[1] = surf.nodes.size() +  id2;
					int id3 = f.GetInt(3);
					if (id3 > 0)
						p.id[2] = id3 - 1;
					else
						p.id[2] = surf.nodes.size() +  id3;
					if (f.size() == 5) {
						int id4 = f.GetInt(4);
						if (id4 > 0)
							p.id[3] = id4 - 1;
						else
							p.id[3] = surf.nodes.size() +  id4;
					} else
						p.id[3] = p.id[0];
				}
			}
		}
		String error = surf.CheckNodeIds();
		if (!error.IsEmpty())
			throw Exc(error);
	} catch (Exc e) {
		throw Exc(t_("Parsing error: ") + e);
	}
}

}