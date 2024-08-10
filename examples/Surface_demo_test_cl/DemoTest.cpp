// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 - 2022, the Anboto author and contributors
#include <Core/Core.h>
#include <Functions4U/Functions4U.h>
#include <plugin/lz4/lz4.h>
#include <Eigen/Eigen.h>
#include <Surface/Surface.h>

using namespace Upp;
using namespace Eigen;

#include "Data.brc"

void TestSurfaceX_Calc() {
	UppLog() << "\n\nSurfaceX demo fast calculations";
	
	double rho = 1025, g = 9.81;
	
	RealTimeStop t;
	String strfile = String(Model, Model_length);
	
	String folder = AFX(GetTempDirectory(), "Surface");
	DirectoryCreate(folder);
	String file = AFX(folder, "demo.stl");
	SaveFile(file, strfile);
	
	Surface mesh, under;
	bool isText;
	String header;
	LoadStl(file, mesh, isText, header);

	UppLog() << "\nMesh data";
	mesh.GetPanelParams();
	t.Start();
	mesh.GetArea();
	mesh.GetVolume();
	t.Pause();
	UppLog() << Format("\nSurface: %.3f", mesh.surface);	
	UppLog() << Format("\nVolume:  %.3f", mesh.volume);
	UppLog() << Format("\nSeconds: %.8f", t.Seconds());
	
	UppLog() << "\nUnderwater data";
	under.CutZ(mesh, -1);
	under.GetPanelParams();
	t.Start();
	under.GetArea();
	under.GetVolume();
	t.Pause();
	UppLog() << Format("\nSurface: %.3f", under.surface);	
	UppLog() << Format("\nVolume:  %.3f", under.volume);
	UppLog() << Format("\nSeconds: %.8f", t.Seconds());
	Point3D cb = under.GetCentreOfBuoyancy();
	UppLog() << Format("\nCB:      %s %s %s", FDS(cb.x, 10, true), FDS(cb.y, 10, true), FDS(cb.z, 10, true));

	Point3D c0(0, 0, 0);
	Force6D f = under.GetHydrostaticForce(c0, rho, g);
	UppLog() << "\nHydroF:  " << f;
	
	UppLog() << "\nSurfaceX data";
	
	SurfaceX surfx;
	surfx.Load(mesh);
	double surface, volume;
	t.Start();
	surface = SurfaceX::GetSurface(surfx);
	volume = SurfaceX::GetVolume(surfx);
	t.Pause();
	UppLog() << Format("\nSurface: %.3f", surface);
	UppLog() << Format("\nVolume:  %.3f", volume);
	UppLog() << Format("\nSeconds: %.8f", t.Seconds());
	
	UppLog() << "\nUnderwater X data";
	t.Start();
	surface = SurfaceX::GetSurface(surfx, [](double x, double y, double z) {return z <= 0;});
	volume = SurfaceX::GetVolume(surfx, [](double x, double y, double z) {return z <= 0;});
	t.Pause();
	UppLog() << Format("\nSurface: %.3f", surface);
	UppLog() << Format("\nVolume:  %.3f", volume);
	UppLog() << Format("\nSeconds: %.8f", t.Seconds());
	
	Affine3d quat = GetTransformRotation(Vector3D(1, 2, 3), Point3D(4, 5, 6));
	double x0 = 1, y0 = 2, z0 = 3;
	Point3D npos;
	TransRot(quat, Point3D(x0, y0, z0), npos);
	
	MatrixXd centroids0(2, 3);
	centroids0 << x0, y0, z0, x0, y0, z0;
	
	//MatrixXd centroids = quat * centroids0.colwise().homogeneous();	
}

void TestSurfaceX_TransRot() {
	UppLog() << "\n\nSurfaceX demo transrot static functions";
	
	String strfile = LZ4Decompress(Data, Data_length);
	
	UVector<double> TwrTpTDxi, TwrTpTDyi, TwrTpTDzi, PtfmSurge, PtfmSway, PtfmHeave, PtfmRoll, PtfmPitch, PtfmYaw;
	
	int posid = 0;
	while(posid >= 0) {
		String line = GetLine(strfile, posid);
		if (Trim(line).IsEmpty())
			break;
		UVector<String> split = Split(line, ';');
		int i = 0;
		PtfmSurge << ScanDouble(split[i++]);
		PtfmSway  << ScanDouble(split[i++]);
		PtfmHeave << ScanDouble(split[i++]);
		PtfmRoll  << ScanDouble(split[i++]);
		PtfmPitch << ScanDouble(split[i++]);
		PtfmYaw   << ScanDouble(split[i++]);
		TwrTpTDxi << ScanDouble(split[i++]);
		TwrTpTDyi << ScanDouble(split[i++]);
		TwrTpTDzi << ScanDouble(split[i++]);
	}
	
	double x, y, z, x0 = 0, y0 = 0, z0 = 139;
	
	for (int i = 0; i < PtfmSurge.size(); ++i) {
		SurfaceX::TransRotFast(x, y, z, x0, y0, z0, PtfmSurge[i], PtfmSway[i], PtfmHeave[i], 
							ToRad(PtfmRoll[i]), ToRad(PtfmPitch[i]), ToRad(PtfmYaw[i])); 
		VERIFY(abs(x - TwrTpTDxi[i]) < 0.005 && abs(y - TwrTpTDyi[i]) < 0.005 && abs(z - TwrTpTDzi[i]) < 0.005);
	}

	for (int i = 0; i < PtfmSurge.size(); ++i) {
		Point3D npos;
		TransRot000(Point3D(x0, y0, z0), PtfmSurge[i], PtfmSway[i], PtfmHeave[i], 
							ToRad(PtfmRoll[i]), ToRad(PtfmPitch[i]), ToRad(PtfmYaw[i]), npos); 
		VERIFY(abs(npos.x - TwrTpTDxi[i]) < 0.01 && abs(npos.y - TwrTpTDyi[i]) < 0.01 && abs(npos.z - TwrTpTDzi[i]) < 0.01);
	}
}


void TestMesh() {
	UppLog() << "\n\nMeshing functions demo";
	
	UVector<Pointf> bound = {
		{78.282032,	1.732051},
		{79.282032,	0},
		{78.282032,	-1.732051},
		{77.282032,	-3.464102},
		{76.282032,	-5.196152},
		{75.282032,	-6.928203},
		{74.282032,	-8.660254},
		{72.282032,	-8.660254},
		{70.282032,	-8.660254},
		{68.282032,	-8.660254},
		{66.282032,	-8.660254},
		{64.282032,	-8.660254},
		{63.282032,	-6.928203},
		{62.282032,	-5.196152},
		{61.282032,	-3.464102},
		{60.282032,	-1.732051},
		{59.282032,	0},
		{60.282032,	1.732051},
		{61.282032,	3.464102},
		{62.282032,	5.196152},
		{63.282032,	6.928203},
		{64.282032,	8.660254},
		{66.282032,	8.660254},
		{68.282032,	8.660254},
		{70.282032,	8.660254},
		{72.282032,	8.660254},
		{74.282032,	8.660254},
		{75.282032,	6.928203},
		{76.282032,	5.196152},
		{77.282032,	3.464102},
		{78.282032,	1.732051}};

	Surface s;
	
	for (double msh = 1; msh <= 2; msh += 1) {
		double deltax = (msh - 1)*15;
		
		Surface s1;
		s1.AddPolygonalPanel(bound, msh, true);
		s1.Translate(-70+deltax, 0, 0);
		
		for (auto &p : bound) 	
			p.x -= 70;
	
		Surface s2;
		s2.AddPolygonalPanel(bound, msh, true);
		s2.Translate(deltax, 20, 0);
		
		UVector<Pointf> bound3 = {
			{0,0},
			{10,0},
			{6,5},
			{6,10},
			{10,15},
			{0,15},
			{4,10},
			{4,5},
			{0,0}};
	
		Surface s3;
		s3.AddPolygonalPanel(bound3, msh, true);
		s3.Translate(deltax, -30, 0);	
	
		s << s1 << s2 << s3;
	}
	
	String file = "MeshDemo.stl";
	UppLog() << "\nSaving " << file;
	SaveStlBin(AFX(GetTempDirectory(), file), s);
	
	// Testing inertia translation
	MatrixXd mat0(6, 6), mat1(6, 6), mat2(6, 6), m;
	Point3D cg(100, 100, 15);
	
	Point3D c00(20, 20, 30);
	mat0 <<	37688.79,	0,			0,			0,			-565331,	-3015103,
			0,			37688.79,	0,			565331,		0,			3015103,
			0,			0,			37688.79,	3015103,	-3015103,	0,
			0,			565331,		3015103,	2.5628E8,	-2.412E8,	45226501,
			-565331	,	0,			-3015103,	-2.412E8,	2.5628E8,	45226497,
			-3015103,	3015103,	0,			45226501,	45226497,	4.8995E8;

	Point3D c01(100, 100, 15);
	mat1 <<	37688.79,	0,			0,			0,			0,			0,
			0,			37688.79,	0,			0,			0,			0,
			0,			0,			37688.79,	0,			0,			0,
			0,			0,			0,			6594687,	0,			0,
			0,			0,			0,			0,			6594688,	0,
			0,			0,			0,			0,			0,			7535693;

	Point3D c02(0, 0, 0);
	mat2<< 	37688.79,	0,			0,			0,			565332.4,	-3768879,
			0,			37688.79,	0,			-565332,	0,			3768879,
			0,			0,			37688.79,	3768879	,	-3768879,	0,
			0,			-565332,	3768879,	3.9196E8,	-3.769E8,	-5.653E7,
			565332.4,	0,			-3768879,	-3.769E8,	3.9196E8,	-5.653E7,
			-3768879,	3768879,	0,			-5.653E7,	-5.653E7,	7.6131E8;
	
	m = mat0;
	Surface::TranslateInertia66(m, cg, c00, c01);
	VERIFY(CompareRatio(m.array(), mat1.array(), 0.001, 10000));
	
	m = mat1;
	Surface::TranslateInertia66(m, cg, c01, c02);
	VERIFY(CompareRatio(m.array(), mat2.array(), 0.001, 10000));
	
	m = mat0;
	Surface::TranslateInertia66(m, cg, c00, c02);
	VERIFY(CompareRatio(m.array(), mat2.array(), 0.001, 10000));
}

void TestPoly() {
	UppLog() << "\nTesting 2D and 3D ContainsPoint()";
	
	UVector<Point3D> polygon = {
        Point3D(0, 0, 1),
        Point3D(0, 3.784, 1),
        Point3D(2.663, 3.784, 2.065),
        Point3D(2.663, 0, 2.065)
    };
    
	UppLog() << "\nTesting ContainsPoint() with a 3D polygon";
	VERIFY(ContainsPointRes::POLY_IN  == ContainsPoint(polygon, Point3D(1.332, 1.829, 1.533), 0.1, ToRad(2.)));
    VERIFY(ContainsPointRes::POLY_OUT == ContainsPoint(polygon, Point3D(3.832, 1.892, 2.533), 0.1, ToRad(2.)));
	
    UVector<Pointf> pol_2 = {	// Convex polygon
        Pointf(-1, 0),
        Pointf(2, 0),
        Pointf(1.5, 1),
        Pointf(1.5, 2),
        Pointf(3, 3),
        Pointf(0, 3),
        Pointf(0.5, 2),
        Pointf(0.5, 1)
    };	
    
    VERIFY(5 == Area(pol_2));
    
    Pointf pin_2 = Pointf(1,   2);
	Pointf ppo_2 = Pointf(1.5, 1.00001);	// In point + error
	Pointf pl1_2 = Pointf(1,   0.00001);	// In line + error
	Pointf pl2_2 = Pointf(1.5, 1.50001);	// In line + error
	Pointf pot_2 = Pointf(0,   1.5);
	Pointf cen_2 = Pointf(1,   1.5);		// Centroid
    
    VERIFY(cen_2 == Centroid(pol_2));
    
    UppLog() << "\nTesting ContainsPoint() with a 3D polygon with different angles in the space, and different points inside, in the boundary, and outside";
	VERIFY(ContainsPointRes::POLY_IN   == ContainsPoint(pol_2, pin_2));
	VERIFY(ContainsPointRes::POLY_SECT == ContainsPoint(pol_2, ppo_2));	
	VERIFY(ContainsPointRes::POLY_SECT == ContainsPoint(pol_2, pl1_2));	
	VERIFY(ContainsPointRes::POLY_SECT == ContainsPoint(pol_2, pl2_2));	
	VERIFY(ContainsPointRes::POLY_OUT  == ContainsPoint(pol_2, pot_2));		
		
	Point3D pin, pin_no, ppo, pl1, pl2, pot, cen;
	UVector<Point3D> pol;
	
	auto assign = [&]() {
		pin = Point3D(pin_2.x, pin_2.y, 0);
		pin_no = Point3D(pin_2.x, pin_2.y, .5);		// "Inside", but not in the same plane
		ppo = Point3D(ppo_2.x, ppo_2.y, 0);
		pl1 = Point3D(pl1_2.x, pl1_2.y, 0);
		pl2 = Point3D(pl2_2.x, pl2_2.y, 0);
		pot = Point3D(pot_2.x, pot_2.y, 0);	
		cen = Point3D(cen_2.x, cen_2.y, 0);	
		
		pol = Point2Dto3D_XY(pol_2);
	};
	auto rotate = [&](double roll, double pitch, double yaw) {
		pin.Rotate(roll, pitch, yaw, 0, 0, 0);
		pin_no.Rotate(roll, pitch, yaw, 0, 0, 0);
		ppo.Rotate(roll, pitch, yaw, 0, 0, 0);
		pl1.Rotate(roll, pitch, yaw, 0, 0, 0);
		pl2.Rotate(roll, pitch, yaw, 0, 0, 0);
		pot.Rotate(roll, pitch, yaw, 0, 0, 0);
		cen.Rotate(roll, pitch, yaw, 0, 0, 0);
		Rotate(pol, roll, pitch, yaw, 0, 0, 0);
	};
	auto test = [&]() {
		VERIFY(ContainsPointRes::POLY_IN   == ContainsPoint(pol, pin, 0.1, ToRad(2.)));
		VERIFY(ContainsPointRes::POLY_FAR  == ContainsPoint(pol, pin_no, 0.1, ToRad(2.)));
		VERIFY(ContainsPointRes::POLY_SECT == ContainsPoint(pol, ppo, 0.1, ToRad(2.)));	
		VERIFY(ContainsPointRes::POLY_SECT == ContainsPoint(pol, pl1, 0.1, ToRad(2.)));	
		VERIFY(ContainsPointRes::POLY_SECT == ContainsPoint(pol, pl2, 0.1, ToRad(2.)));	
		VERIFY(ContainsPointRes::POLY_OUT  == ContainsPoint(pol, pot, 0.1, ToRad(2.)));	
		
		VERIFY(EqualRatio(5., Area(pol), .0001));
		Point3D c = Centroid(pol);
		VERIFY(EqualRatio(cen.x, c.x, 0.0001) && EqualRatio(cen.y, c.y, 0.0001) && EqualRatio(cen.z, c.z, 0.0001));
	};
	
	for (double roll = 0; roll <= M_PI; roll += M_PI/4)				// Rotates to check that this works in any position in the space
		for (double pitch = 0; pitch <= M_PI; pitch += M_PI/4)
			for (double yaw = 0; yaw <= M_PI; yaw += M_PI/4) {
				for (int i = 0; i < pol_2.size(); ++i) {
					assign();
					Rotate(pol, i); 			// Circular rotation of the polynomial
					rotate(roll, pitch, yaw);
					test();
				}
				for (int i = 0; i < pol_2.size(); ++i) {
					assign();
					ReverseX(pol);				// Reverse of the polynomial... to check that the order doesn't matter
					Rotate(pol, i); 			// Circular rotation of the polynomial
					rotate(roll, pitch, yaw);
					test();
				}
			}
	
 	UppLog() << "\nTesting Surface::AddPolygonalPanel(), at scale 1 and 100, with different mesh sizes, comparing areas";
	Surface s;
    UVector<double> sizes = {0.05, 0.1, 0.2, 0.5, 1};
    
    for (int n = 0; n < 1; ++n) {
	    for (int i = 0; i < sizes.size(); ++i) {
	        s.Clear();
			s.AddPolygonalPanel(pol_2, sizes[i], true);
			VERIFY(EqualRatio(Area(pol_2), s.GetArea(), 0.00001));		// 5
	    }
	    for (int i = 0; i < sizes.size(); ++i) 
	        sizes[i] *= 100;
	    for (int i = 0; i < pol_2.size(); ++i) 
	        pol_2[i] *= 100;
    }	
}


void TestBasic() {
	UppLog() << "\nTesting basic classes";
	
	{
		UppLog() << "\nTesting force sum 1";
		
		Point3D c0(1, -1, 5);
		
		Vector3D f1(2, 5, 4);
		Point3D p1(3, 6, 7);
	
		Vector3D f2(-1, 5, -2);
		Point3D p2(1, 2, -4);
	
		Force6D f = Force6D::Zero();
		f.Add(f1, p1, c0);
		f.Add(f2, p2, c0);
	
		VERIFY(f == Force6D(1, 10, 2, 57, 5, -1));
	}
	{
		UppLog() << "\nTesting force sum 2";
		
		Point3D c0(2, -3, 1);
		
		Vector3D f1(22, 15, 34);
		Point3D p1(31, 16, -77);
	
		Vector3D f2(-12, -5, 22);
		Point3D p2(11, 234, 14);
	
		Force6D f = Force6D::Zero();
		f.Add(f1, p1, c0);
		f.Add(f2, p2, c0);
	
		VERIFY(f == Force6D(10, 10, 56, 7095, -3056, 2816));
	}
	{
		// Points c0 and p are in the same rigid body
		// If c0 translates trans and rotates rot, where p will be?
		
		Point3D c0(10, 20, -30);
		Value3D trans(2, 1, 6), rot(-M_PI/4, M_PI, M_PI/3);
		{
			Point3D p(23, -12, 45);
			p.TransRot(trans, rot, c0, RotationOrder::XYZ);		// Considering rotation in XYZ order
			VERIFY(p == Point3D(-22.2128129, -35.3858754, -73.6801417));
		}
		{
			Point3D p(23, -12, 45);
			p.TransRot(trans, rot, c0, RotationOrder::ZYX);		// Considering rotation in ZYX order
			VERIFY(p == Point3D(-20.8320147, 24.9444655, -99.6604255));
		}
		{
			// In the same case, if the speed in c0 is vtrans and vrot, what would be the speed at p?
			
			Point3D p(23, -12, 45);
		
			Value3D vtrans(22, 11, -16), vrot(-M_PI/5, M_PI/2, -M_PI/5);
			
			Velocity6D v(vtrans, vrot);
			v.Translate(c0, p);
			VERIFY(v == Velocity6D(Value3D(119.7035315, 49.9557489, -16.3141592), Value3D(-M_PI/5, M_PI/2, -M_PI/5)));
		}
	}
}
	
CONSOLE_APP_MAIN 
{
	StdLogSetup(LOG_COUT|LOG_FILE);
	SetExitCode(0);
	
	UppLog() << "Surface demo and test";
	
	try {
		TestBasic();
		
		TestPoly();
		TestMesh();

		TestSurfaceX_Calc();
		TestSurfaceX_TransRot();
	  
	    UppLog() << "\n\nAll tests passed\n";
   	} catch (Exc e) {
		UppLog() << "\nError: " << e << "\n";  
		SetExitCode(1);
	} catch (...) {
		UppLog() << "\nUnknown Error\n";  
		SetExitCode(1);
	}
	 
	#ifdef flagDEBUG
	UppLog() << "\n";
	Cout() << "\nPress enter key to end";
	ReadStdIn();
	#endif    
}

