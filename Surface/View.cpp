// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors   
#include <Core/Core.h>
#include <Surface/Surface.h>
#include <Draw/Draw.h>
#include <Painter/Painter.h>
#include <ScatterDraw/DataSource.h>

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

void Surface::Render(Surface &mesh, Vector<Color> &colors, Color color, const Point3D &lightDir, double ax, double ay, double az, double cx, double cy, double cz, SurfaceShowColor toShowColor) const {
	Point3D clightDir = clone(lightDir);
	clightDir.Normalize();
	RenderObject(mesh, colors, color, clightDir, ax, ay, az, cx, cy, cz, toShowColor);			       			       
}

void Surface::RenderObject(Surface &surf, Vector<Color> &colors, Color color, const Point3D &lightDir, 
			   double ax, double ay, double az, double _c_x, double _c_y, double _c_z, SurfaceShowColor toShowColor) const {
	surf = clone(*this);								// Orientates the surface
	surf.Rotate(ax, -(ay + M_PI), az + M_PI, _c_x, _c_y, _c_z);
	
	colors.Clear();
	colors.SetCount(surf.panels.size());
	
	for (int i = 0; i < surf.panels.size(); ++i) {		// Get the colours
		Panel &panel = surf.panels[i];
		
		const Point3D &p0 = surf.nodes[panel.id[0]];
		const Point3D &p1 = surf.nodes[panel.id[1]];
		const Point3D &p2 = surf.nodes[panel.id[2]];
	
		panel.normalPaint = Normal(p0, p1, p2);
		panel.centroidPaint = Centroid(p0, p1, p2);
		if (!panel.IsTriangle()) {
			const Point3D &p3 = surf.nodes[panel.id[3]];
			panel.normalPaint = avg(panel.normalPaint, Normal(p2, p3, p0));
			panel.centroidPaint = avg(panel.centroidPaint, Centroid(p2, p3, p0));
		}
	 	int c = (int) abs(lightDir.dot(panel.normalPaint) * 255.0);
	 	if (toShowColor == SHOW_BRIGHTER)
	 		colors[i] = BrightenColor(color, c);
	 	else if (toShowColor == SHOW_DARKER)
	 		colors[i] = DarkenColor(color, c);
	 	else if (toShowColor == SHOW_FLAT)
	 		colors[i] = color;
	 	else
	 		colors[i] = Color(c, c, c);
	}
    Vector<int> ids = GetSortOrderX(surf.panels, [](const Panel& p1, const Panel& p2) {	// Sort by Z depth
        return p1.centroidPaint.z > p2.centroidPaint.z;
    });
    surf.panels = ApplyIndex(surf.panels, ids);
    colors = ApplyIndex(colors, ids);
}

void Surface::GetRenderDimensions(double &minX, double &maxX, double &minY, double &maxY) const {
	minX = minY = std::numeric_limits<double>::max();
	maxX = maxY = std::numeric_limits<double>::lowest();
	
	for (const Point3D &p : nodes) {
		minX = min(p.x, minX);
		minY = min(p.y, minY);
		maxX = max(p.x, maxX);
		maxY = max(p.y, maxY);
	}
}
	
Image Surface::GetImage(double scale, int width, int height, Color color, Color back, const Point3D &lightDir, 
			   int dx, int dy, double ax, double ay, double az, double cx, double cy, double cz, bool painter, SurfaceShowMesh toShowMesh, SurfaceShowColor toShowColor) const {
	int mode = MODE_ANTIALIASED;
	
	ImageBuffer ib(Size(width, height));	
	BufferPainter bp(ib, mode);	
	
	bp.LineCap(LINECAP_SQUARE);
	bp.LineJoin(LINEJOIN_MITER);
	
	Surface render;
	Vector<Color> colors;
	Render(render, colors, color, lightDir, ax, ay, az, cx, cy, cz, toShowColor);
	Paint(bp, render, colors, scale, width, height, back, lightDir, dx, dy, painter, toShowMesh);

	return ib;				       
}

}
