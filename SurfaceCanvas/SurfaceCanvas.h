// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2025, the Anboto author and contributors  
#ifndef _SurfaceCanvas_SurfaceCanvas_h_
#define _SurfaceCanvas_SurfaceCanvas_h_

#include <Surface/Surface.h>

namespace Upp {

class SurfaceCanvas : public Ctrl, 	
					  public SurfaceView {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef SurfaceCanvas CLASSNAME;

	SurfaceCanvas &ZoomToFit();
		
	SurfaceCanvas &SetScale(double s)			{scale = s;				Render();	return *this;}
	SurfaceCanvas &SetPosition(const Pointf &p)	{pos = clone(p);		Render();	return *this;}
	SurfaceCanvas &SetCentre(const Point3D &p);
	SurfaceCanvas &SetRotation(const Affine3d &p){affine = p;			Render();	return *this;}
	SurfaceCanvas &SetRotation(const Value3D &rot);
	SurfaceCanvas &ScrollPosition(double rx, double ry);
	
	SurfaceCanvas &SetRotationX();
	SurfaceCanvas &SetRotationY();
	SurfaceCanvas &SetRotationZ();
	SurfaceCanvas &SetRotationXYZ();
		
	double GetScale() const						{return scale;}
	const Pointf &GetPosition() const			{return pos;}
	const Affine3d &GetRotation() const			{return affine;}
	
	SurfaceCanvas &SetShowMesh(ShowMesh s)		{SurfaceView::SetShowMesh(s);	return *this;}
	SurfaceCanvas &SetPainter(bool b)			{SurfaceView::SetPainter(b); 	return *this;}
	
	void Render();

	void SaveToFile(String fileName);
	void SaveToClipboard();
	
	virtual void Paint(Draw& w) override;
	
	Function <void()> WhenPaint;

	SurfaceCanvas &SetCanSelect(bool v = true)	{canSelect = v;			return *this;}
	Function <void(int&, int&, bool)> WhenSelect;

	SurfaceCanvas &SetJPGQuality(int quality) 	{jpgQuality = quality; 	return *this;}
	int GetJPGQuality() 				 		{return jpgQuality;}
	
	void Xmlize(XmlIO& xml) override	{Ize(xml);}
	void Jsonize(JsonIO& json) override	{Ize(json);}
	
	void Serialize(Stream& s) override {
		s % defaultFileName
		  % jpgQuality
		;
	}
	
private:
	virtual Image MouseEvent(int event, Point p, int zdelta, dword keyflags) override;
	virtual bool Key(dword key, int count) override;
	
	void ContextMenu(Bar& bar);
	
	void SelectPoint(Point p, bool select);
	
	template <class T>
	void Ize(T& io) { 
		io
			("defaultFileName", defaultFileName)
			("jpgQuality", jpgQuality)
		;
	}
	
	bool rotating = false, translating = false, selecting = false;
	Point begin = Point(0, 0);
	
	int contextIdX = 0, contextIdY = 0, contextIdZ = 0, contextIdXYZ = 0;
	Vector<Value3D> contextX = {Value3D(M_PI/2, M_PI, M_PI/2), 	Value3D(M_PI/2, M_PI, -M_PI/2)};
	Vector<Value3D> contextY = {Value3D(M_PI/2, M_PI, 0), 		Value3D(M_PI/2, M_PI, M_PI)};
	Vector<Value3D> contextZ = {Value3D(0, 0, 0), 				Value3D(M_PI, 0, 0)};
	Vector<Value3D> contextXYZ={Value3D(-M_PI/4, 0, M_PI/4), 	Value3D(-M_PI/4, 0, -M_PI/4), 	Value3D(-M_PI/4, 0, M_PI/4+M_PI), 	Value3D(-M_PI/4, 0, M_PI/4+M_PI/2)};
	
	double scale = 10;
	Pointf pos;
	Affine3d affine = Affine3d::Identity();
	Point3D centre = Point3D::Zero();
	
	bool canSelect = false;
	
	int jpgQuality = 90;
	String defaultFileName;
	
	int lastidBody = -1, lastidSubBody = -1;
};


}

#endif
