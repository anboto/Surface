// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors   
#include <CtrlLib/CtrlLib.h>
#include <plugin/jpg/jpg.h>
#include <plugin/png/png.h>
#include <PdfDraw/PdfDraw.h>
#include <SurfaceCanvas/SurfaceCanvas.h>
#include <Draw/Draw.h>
#include <Painter/Painter.h>
#include <ScatterDraw/DataSource.h>
#include <STEM4U/Utility.h>

#define IMAGECLASS SurfaceCanvasImg
#define IMAGEFILE <SurfaceCanvas/SurfaceCanvas.iml>
#include <Draw/iml.h>

#include <Functions4U/EnableWarnings.h>

namespace Upp {
using namespace Eigen;


void SurfaceCanvas::Render() { 
	SurfaceView::Render(affine);
	Refresh();
}

SurfaceCanvas &SurfaceCanvas::ZoomToFit() { 
	SurfaceView::ZoomToFit(GetSize(), scale, pos);
	Refresh();
	return *this;
}

void SurfaceCanvas::Paint(Draw &w) {
	GuiLock __;
		
	SurfaceView::Paint(w, GetSize(), scale, pos.x, pos.y);
	WhenPaint();
}

SurfaceCanvas &SurfaceCanvas::ScrollPosition(double rx, double ry) {
	Size sz = GetSize();
	pos.x += sz.cx*rx;
	pos.y += sz.cy*ry;
	Refresh();
	return *this;	
}

void SurfaceCanvas::SaveToFile(String fileName) {
	GuiLock __;
	
	if (IsNull(fileName)) {
		FileSel fs;
		
		fs.NoExeIcons();
		fs.Type(Format(t_("%s bitmap file"), "jpeg"), "*.jpg");
		fs.Type(Format(t_("%s bitmap file"), "png"), "*.png");
		fs.Type(Format(t_("%s vector file"), "pdf"), "*.pdf");
		fs.AllFilesType();
		
		if (!defaultFileName.IsEmpty())
			fs = defaultFileName;
		else
			fs = String(t_("Mesh view")) + ".jpg";
		
		String ext = GetFileExt(~fs);
		fs.DefaultExt(ext);
		int idt = 0;
		if (ext == ".jpg" || ext == ".jpeg")
			idt = 0;
		else if (ext == ".png")
			idt = 1;
		else if (ext == ".pdf")
			idt = 2;
		fs.ActiveType(idt);
	
		fs.ActiveDir(GetFileFolder(defaultFileName));
		fs.type.WhenAction = [&] {
			int id = fs.type.GetIndex();
			
			if (id == 0)
				fs.file = ForceExt(GetFileName(~fs), ".jpg");
			else if (id == 1)
				fs.file = ForceExt(GetFileName(~fs), ".png");
			else if (id == 2)
				fs.file = ForceExt(GetFileName(~fs), ".pdf");
		};
	    if(!fs.ExecuteSaveAs(t_("Saving image to file"))) {
	        Exclamation(t_("Image has not been saved"));
	        return;
	    }
	    fileName = defaultFileName = ~fs;
	} else
		defaultFileName = fileName;
	 
	if (GetFileExt(fileName) == ".png") {
		WaitCursor waitcursor;
		PNGEncoder encoder;
		if (!encoder.SaveFile(fileName, GetImage(GetSize(), scale, pos.x, pos.y, affine)))
			Exclamation(t_("Image has not been saved"));
	} else if (GetFileExt(fileName) == ".jpg") {	
		WaitCursor waitcursor;
		JPGEncoder encoder(jpgQuality);
		if (!encoder.SaveFile(fileName, GetImage(GetSize(), scale, pos.x, pos.y, affine)))
			Exclamation(t_("Image has not been saved"));
	} else if (GetFileExt(fileName) == ".pdf") {	
		WaitCursor waitcursor;
		int pdfscale = 3;
		Size sz = pdfscale*GetSize();
		double pscale = pdfscale*scale;
		//double thickness = pdfscale*lineThickness;
		double tx = pdfscale*pos.x;
		double ty = pdfscale*pos.y;
		
		PdfDraw pdf(sz);
		SurfaceView::Paint(pdf, sz, pscale, tx, ty);
		String spdf = pdf.Finish();
			
		if (!SaveFile(fileName, spdf))
			Exclamation(t_("Image has not been saved"));
	} else 
		Exclamation(Format(t_("File format \"%s\" not found"), GetFileExt(fileName)));
}

void SurfaceCanvas::SaveToClipboard() {
	GuiLock __;
	
	Image image = GetImage(GetSize(), scale, pos.x, pos.y, affine);
	if (IsNull(image)) {
		Exclamation(t_("Imposible to get view image"));
		return;
	}
	
	WriteClipboardImage(image);	
}

Image SurfaceCanvas::MouseEvent(int event, Point p, int zdelta, dword keyflags) {
	bool nothingdone = false;
	if (((event & Ctrl::BUTTON) == Ctrl::RIGHT) && ((event & Ctrl::ACTION) == Ctrl::UP)) {
		Disable();		// If window has slow refresh
		MenuBar::Execute(THISBACK(ContextMenu));
		Enable();
	} else if ((event & Ctrl::ACTION) == Ctrl::MOUSEWHEEL) {
		double factor = zdelta/1000.;
		SetScale(GetScale()*(1+factor));
	} else if ((event & Ctrl::BUTTON) == Ctrl::MIDDLE && (event & Ctrl::ACTION) == Ctrl::DOWN) {
		translating = true;
		begin = clone(p);
	} else if ((event & Ctrl::BUTTON) == Ctrl::LEFT && (event & Ctrl::ACTION) == Ctrl::DOWN) {
		SetFocus();
		if (canSelect && keyflags & K_CTRL) {
			selecting = true;
			SelectPoint(p, true);
		} else if (canSelect && keyflags & K_SHIFT) {
			selecting = true;
			SelectPoint(p, false);
		} else {
			rotating = true;
			begin = clone(p);
		}
	} else if((event & Ctrl::ACTION) == Ctrl::MOUSEMOVE) {
		if (canSelect && selecting && keyflags & K_CTRL)
			SelectPoint(p, true);
		else if (canSelect && selecting && keyflags & K_SHIFT)
			SelectPoint(p, false);
		else if (begin != p) {
			if (translating) {
				Pointf ps = GetPosition();
				ps.x += p.x - begin.x;
				ps.y -= p.y - begin.y;
				SetPosition(ps);
				begin = clone(p);
			} else if (rotating) {
				Affine3d rot = TrackballRotation(begin, p, GetSize(), centre);
				affine = rot * affine;
				SetRotation(affine);
				begin = clone(p);
			}
		}
	} else if ((event & Ctrl::ACTION) == Ctrl::UP || (event & Ctrl::ACTION) == Ctrl::MOUSELEAVE) 
		translating = rotating = selecting = false;
	else
		nothingdone = true;
	
	if (!nothingdone)	
		Refresh();
	return Image::Hand();
}

void SurfaceCanvas::ContextMenu(Bar& bar) {
	bar.Add(t_("Fit to data"), SurfaceCanvasImg::ShapeHandles(), [&]{ZoomToFit();})				.Key(K_CTRL_F).Help(t_("Zoom to fit visible all data"));
	bar.Add(t_("Zoom +"), 	   SurfaceCanvasImg::ZoomPlus(),  	 [&]{SetScale(GetScale()*1.1);}).Key(K_ADD).Help(t_("Zoom in (closer)"));
	bar.Add(t_("Zoom -"), 	   SurfaceCanvasImg::ZoomMinus(), 	 [&]{SetScale(GetScale()*0.9);}).Key(K_SUBTRACT).Help(t_("Zoom out (away)"));
	bar.Add(t_("Scroll right"),SurfaceCanvasImg::RightArrow(), 	 [&]{ScrollPosition(-.05, 0);}) .Key(K_RIGHT).Help(t_("Scroll right"));
	bar.Add(t_("Scroll left"), SurfaceCanvasImg::LeftArrow(), 	 [&]{ScrollPosition(.05,  0);}) .Key(K_LEFT).Help(t_("Scroll left"));
	bar.Add(t_("Scroll up"),   SurfaceCanvasImg::UpArrow(), 	 [&]{ScrollPosition(0, -.05);}) .Key(K_UP).Help(t_("Scroll up"));
	bar.Add(t_("Scroll down"), SurfaceCanvasImg::DownArrow(), 	 [&]{ScrollPosition(0, .05);})  .Key(K_DOWN).Help(t_("Scroll down"));
	bar.Add(t_("View X axis"), 			[&]{SetRotationX();});
	bar.Add(t_("View Y axis"), 			[&]{SetRotationY();});
	bar.Add(t_("View Z axis"), 			[&]{SetRotationZ();});
	bar.Add(t_("View isometric XYZ"), 	[&]{SetRotationXYZ();});
	bar.Separator();
	bar.Add(t_("Copy image"),  SurfaceCanvasImg::Copy(), [&]{SaveToClipboard();}).Key(K_CTRL_C).Help(t_("Copy image to clipboard"));
	bar.Add(t_("Save image"),  SurfaceCanvasImg::Save(), [&]{SaveToFile(Null);}).Key(K_CTRL_S).Help(t_("Save image to file"));
}

bool SurfaceCanvas::Key(dword key, int ) {
	if (key == K_CTRL_F)
		ZoomToFit();
	else if (key == K_ADD || key == K_PLUS)
		SetScale(GetScale()*1.1);
	else if (key == K_SUBTRACT || key == K_MINUS)
		SetScale(GetScale()*0.9);
	else if (key == K_LEFT)
		ScrollPosition(.05,  0);
	else if (key == K_RIGHT)
		ScrollPosition(-.05,  0);
	else if (key == K_UP)
		ScrollPosition(0, -.05);
	else if (key == K_DOWN)
		ScrollPosition(0, .05);
	else if (key == K_CTRL_C)
		SaveToClipboard();
	else if (key == K_CTRL_S)
		SaveToFile(Null);
	else
		return false;
	return true;
}

SurfaceCanvas &SurfaceCanvas::SetRotation(const Value3D &rot) {
	if (IsNull(rot))
		return *this;
	
	affine = GetTransformRotation(Value3D(rot.x, -(rot.y + M_PI), rot.z + M_PI), centre);		
	return SetRotation(affine);
}

SurfaceCanvas &SurfaceCanvas::SetCentre(const Point3D &p) {
	if (IsNull(p))
		return *this;
	
	affine =    Translation3d(p)
			  * Translation3d(-centre)
			  * affine
			  * Translation3d(centre)
			  * Translation3d(-p);
	centre = clone(p);		
	return SetRotation(affine);
}

SurfaceCanvas &SurfaceCanvas::SetRotationX() {
	SetRotation(contextX[contextIdX++]);		
	contextIdX %= contextX.size();
	return ZoomToFit();
}

SurfaceCanvas &SurfaceCanvas::SetRotationY() {
	SetRotation(contextY[contextIdY++]);		
	contextIdY %= contextY.size();
	return ZoomToFit();
}

SurfaceCanvas &SurfaceCanvas::SetRotationZ() {
	SetRotation(contextZ[contextIdZ++]);		
	contextIdZ %= contextZ.size();
	return ZoomToFit();
}

SurfaceCanvas &SurfaceCanvas::SetRotationXYZ() {
	SetRotation(contextXYZ[contextIdXYZ++]);		
	contextIdXYZ %= contextXYZ.size();
	return ZoomToFit();
}

void SurfaceCanvas::SelectPoint(Point p, bool select) {
	int idBody, idSubBody;
	SurfaceView::SelectPoint(p, GetSize(), scale, pos.x, pos.y, idBody, idSubBody, select);
	if (lastidBody < 0 || lastidSubBody < 0 || lastidBody != idBody || lastidSubBody != idSubBody) {
		WhenSelect(idBody, idSubBody, select);
		lastidBody = idBody;
		lastidSubBody = idSubBody;
		Render();	
	}
}

}