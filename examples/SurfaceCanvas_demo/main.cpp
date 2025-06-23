#include <CtrlLib/CtrlLib.h>
#include <SurfaceCanvas/SurfaceCanvas.h>
#include <Controls4U/Controls4U.h>
#include <PdfDraw/PdfDraw.h>

using namespace Upp;

#define LAYOUTFILE <examples/SurfaceCanvas_demo/main.lay>
#include <CtrlCore/lay.h>


struct SurfaceViewer : TopWindow {
	Surface surf;
	SurfaceCanvas view;
		
	//Color lightColor{255, 0, 0};
	
	WithPanelLayout<StaticRect> panel;
	Splitter splitter;

	void ViewRefresh() { 
		view.SetShowMesh((SurfaceView::ShowMesh)(int)panel.opShowMesh.GetData());
			
		Refresh();
	}
	
	void RenderRefresh() { 
		view.SetBackgroundColor(~panel.backcolor).SetLineThickness(~panel.thickness);
		view.SetShowColor((SurfaceView::ShowColor)(int)panel.opShowColor.GetData())
			.SetLightDir(Point3D(panel.lightX, panel.lightY, panel.lightZ));
		view.SetCentre(Point3D(~panel.cx, ~panel.cy, ~panel.cz));
		
		view.Render();
		ViewRefresh();
	}
	
	void FullRefresh(bool fit) {
		view.Clear();
		
		view.PaintSurface(surf, ~panel.color, ~panel.meshColor, ~panel.thickness, 0, ~panel.opNormals, surf.avgFacetSideLen);
		view.PaintArrow(0, 0, 0, surf.env.LenRef()/4., 0, 0, LtRed(), -20);
		view.PaintArrow(0, 0, 0, 0, surf.env.LenRef()/4., 0, LtGreen(), -20);
		view.PaintArrow(0, 0, 0, 0, 0, surf.env.LenRef()/4., LtBlue(), -20);
		
		if (fit) {
			view.SetRotation(Value3D(ToRad(-45), 0, ToRad(45)));
			view.ZoomToFit();
		}
		RenderRefresh();
		panel.numItems.SetLabel(FormatInt(view.GetNumItems()));
		panel.numNodes.SetLabel(FormatInt(view.GetNumNodes()));
	}

	bool Load() {
		String file = ~panel.fileLoad;
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
				throw Exc("Unknown extension");
		} catch (Exc e) {
			Exclamation(DeQtf(e));
			return false;
		}
		
		surf.GetPanelParams();
		surf.GetArea();
		surf.GetEnvelope();
		view.SetRotationXYZ();
		FullRefresh(true);
		
		return true;
	}
	
	SurfaceViewer() {
		panel.fileLoad <<= GetDataFile("UMaine.grd");
		panel.fileLoad.WhenChange << [=]{return Load();};

		Sizeable().Zoomable().CenterScreen().SetRect(0, 0, 800, 800);
		CtrlLayout(panel);
		Add(splitter.SizePos());
		splitter.Horz(view.SizePos(), panel.SizePos());
		splitter.SetPos(6000, 0);
		
		// FullRefresh
		panel.cx 		<< [=]{FullRefresh(false);};					panel.cx <<= 0;
		panel.cy 		<< [=]{FullRefresh(false);};					panel.cy <<= 0;
		panel.cz 		<< [=]{FullRefresh(false);};					panel.cz <<= 0;			
		
		panel.color 	<< [=]{FullRefresh(false);};					panel.color <<= Color{52, 229, 52};
		panel.meshColor << [=]{FullRefresh(false);};					panel.meshColor <<= Color{70, 70, 70};
		
		panel.thickness.SetInc(1).Min(1) << [=]{FullRefresh(false);};	panel.thickness <<= 1;
		
		panel.opNormals << [=]{FullRefresh(false);};					panel.opNormals = false;
		panel.opDeselect<< [=]{view.SetDeselectIfClickOut(panel.opDeselect); FullRefresh(false);};					panel.opDeselect = true;
		
		// RenderRefresh
		panel.lightX 	<< [=]{RenderRefresh();};						panel.lightX = 0;
		panel.lightY 	<< [=]{RenderRefresh();};						panel.lightY = 0;
		panel.lightZ 	<< [=]{RenderRefresh();};						panel.lightZ = -1;

		panel.backcolor << [=]{RenderRefresh();};						panel.backcolor <<= Color{52, 180, 235};

		panel.opShowColor.Add(SurfaceView::SHOW_DARKER, "SHOW_DARKER").Add(SurfaceView::SHOW_BRIGHTER, "SHOW_BRIGHTER")
						 .Add(SurfaceView::SHOW_FLAT, "SHOW_FLAT").SetData(0);
		panel.opShowColor << [=]{RenderRefresh();};
		
		// ViewRefresh
		panel.opShowMesh.Add(SurfaceView::SHOW_MESH, "SHOW_MESH").Add(SurfaceView::SHOW_VISIBLE_MESH, "SHOW_VISIBLE_MESH")
						.Add(SurfaceView::SHOW_FACES, "SHOW_FACES").Add(SurfaceView::SHOW_MESH_FACES, "SHOW_MESH_FACES").SetData(3);
		panel.opShowMesh << [=]{ViewRefresh();};
		
		
		panel.butLoad << [=]{Load();};
		
		panel.butSave << [=] {
			view.SaveToFile(~panel.fileSave);
		};
		
		panel.fileSave.SelLoad(false);
		
		view.SetCanSelect();
		view.WhenSelect << [=](int&, int&, bool) {FullRefresh(false);};
		
		Load();
	}
};

GUI_APP_MAIN
{
	SurfaceViewer().Run();
}
