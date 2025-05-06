#include <CtrlLib/CtrlLib.h>
#include <Surface/Surface.h>
#include <Controls4U/Controls4U.h>


using namespace Upp;

#define LAYOUTFILE <examples/Surface_viewer/main.lay>
#include <CtrlCore/lay.h>


struct SurfaceViewer : TopWindow {
	Surface surf,
			render;
	UVector<Color> colors;
		
	//Color lightColor{255, 0, 0};
	
	WithPanelLayout<StaticRect> panel;
	SplitterFrame sf;
	
	SurfaceViewer() {
		panel.file <<= GetDataFile("teapot.obj");

		Sizeable().Zoomable().CenterScreen().SetRect(0, 0, 800, 800);
		CtrlLayout(panel);
		AddFrame(sf.Right(panel, 600));
		
		auto ViewRefresh = [=] { 
			panel.valrx.SetLabel(AsString(~panel.rx));
			panel.valry.SetLabel(AsString(~panel.ry));
			panel.valrz.SetLabel(AsString(~panel.rz));
			panel.valtx.SetLabel(AsString(~panel.tx));
			panel.valty.SetLabel(AsString(~panel.ty));
			panel.valscale.SetLabel(AsString(~panel.scale));
			
			Refresh();
		};
		
		auto FullRefresh = [=] { 
			surf.Render(render, colors, ~panel.color, Point3D(panel.lightX, panel.lightY, panel.lightZ), ToRad(double(~panel.rx)), ToRad(double(~panel.ry)), ToRad(double(~panel.rz)), 
						~panel.cx, ~panel.cy, ~panel.cz, (Surface::SurfaceShowColor)(int)panel.opShowColor.GetData());
			ViewRefresh();
		};
		
		auto ZoomToFit = [=] { 
			double minX, maxX, minY, maxY;
			render.GetRenderDimensions(minX, maxX, minY, maxY);
			
			Size sz = GetSize();
			double scaleX = sz.cx/(maxX - minX);
			double scaleY = sz.cy/(maxY - minY);
			double cx = avg(maxX, minX);
			double cy = avg(maxY, minY);
			
			if (scaleX < scaleY) {
				panel.scale <<= int(scaleX);
				panel.tx <<= -int(cx*scaleX);
				panel.ty <<= int(cy*scaleX);
			} else { 
				panel.scale <<= int(scaleY);
				panel.tx <<= -int(cx*scaleY);
				panel.ty <<= int(cy*scaleY);
			}
				
			ViewRefresh();
		};
				
		panel.scale.MinMax(1, 500).Step(1) << ViewRefresh;
		panel.butZoomToFit << ZoomToFit;
		
		panel.rx.MinMax(-180, 180).Step(1) << FullRefresh;
		panel.ry.MinMax(-180, 180).Step(1) << FullRefresh;
		panel.rz.MinMax(-180, 180).Step(1) << FullRefresh;

		panel.tx.MinMax(-1000, 1000).Step(1) << ViewRefresh;
		panel.ty.MinMax(-1000, 1000).Step(1) << ViewRefresh;

		panel.scale <<= 50;
		
		panel.rx <<= -45;
		panel.ry <<= 0;
		panel.rz <<= 45;

		panel.tx <<= 0;
		panel.ty <<= 0;
		
		panel.cx <<= 0;
		panel.cy <<= 0;
		panel.cz <<= 0;
		
		panel.lightX = 0;
		panel.lightY = 0;
		panel.lightZ = -1;
		
		panel.lightX << FullRefresh;
		panel.lightY << FullRefresh;
		panel.lightZ << FullRefresh;
		
		panel.color <<= Color{52, 229, 52};
		panel.backcolor <<= Color{52, 180, 235};
		
		panel.color << FullRefresh;
		panel.backcolor << FullRefresh;
		
		panel.opPainter << ViewRefresh;
		panel.opShowMesh << ViewRefresh;
		
		panel.opShowMesh.Add(Surface::SHOW_MESH, "SHOW_MESH").Add(Surface::SHOW_VISIBLE_MESH, "SHOW_VISIBLE_MESH")
						.Add(Surface::SHOW_FACES, "SHOW_FACES").Add(Surface::SHOW_MESH_FACES, "SHOW_MESH_FACES");
		
		panel.opShowMesh.SetData(3);
		
		panel.opShowColor << FullRefresh;
		
		panel.opShowColor.Add(Surface::SHOW_DARKER, "SHOW_DARKER").Add(Surface::SHOW_BRIGHTER, "SHOW_BRIGHTER")
						 .Add(Surface::SHOW_FLAT, "SHOW_FLAT");
		
		panel.opShowColor.SetData(0);		
		
		panel.butLoad.WhenAction = [=] {
			String file = ~panel.file;
			String ext = GetFileExt(file);
			bool y0z = false, x0z = false;
			
			surf.Clear();
			
			try {
				if (ext == ".obj")
					LoadOBJ(file, surf);
				else if (ext == ".stl")
					LoadStl(file, surf);
				else if (ext == ".grd")
					LoadGRD(file, surf, y0z, x0z);
				else if (ext == ".gmsh")
					LoadGMSH(file, surf);
				else if (ext == ".msh")	
					LoadTDynMsh(file, surf);
				else
					Exclamation("Unknown extension");
			} catch (Exc e) {
				Exclamation(e);
			}
			
			FullRefresh();
		};
		
		panel.butLoad.WhenAction();
		
		FullRefresh();
	}
	void Paint(Draw& w) override {
		Size sz = GetSize();
		w.DrawRect(sz, ~panel.backcolor);
		Surface::Paint(w, render, colors, ~panel.scale, sz.cx, sz.cy, ~panel.backcolor, Point3D(panel.lightX, panel.lightY, panel.lightZ), 
			   ~panel.tx, ~panel.ty, panel.opPainter, (Surface::SurfaceShowMesh)(int)panel.opShowMesh.GetData());
	}
};

GUI_APP_MAIN
{
	SurfaceViewer().Run();
}
