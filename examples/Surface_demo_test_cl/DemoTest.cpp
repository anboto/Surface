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
	
	String folder = AppendFileNameX(GetTempDirectory(), "Surface");
	DirectoryCreate(folder);
	String file = AppendFileNameX(folder, "demo.stl");
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
	
	Affine3d quat;
	SurfaceX::GetTransform(quat, 1, 2, 3, 4, 5, 6);
	double x, y, z, x0 = 1, y0 = 2, z0 = 3;
	SurfaceX::TransRot(x, y, z, x0, y0, z0, quat);
	
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
		SurfaceX::TransRot(x, y, z, x0, y0, z0, PtfmSurge[i], PtfmSway[i], PtfmHeave[i], 
							ToRad(PtfmRoll[i]), ToRad(PtfmPitch[i]), ToRad(PtfmYaw[i])); 
		VERIFY(abs(x - TwrTpTDxi[i]) < 0.01 && abs(y - TwrTpTDyi[i]) < 0.01 && abs(z - TwrTpTDzi[i]) < 0.01);
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
		s3.AddPolygonalPanel2(bound3, msh, true);
		s3.Translate(deltax, -30, 0);	
	
		s << s1 << s2 << s3;
	}
	
	String file = "MeshDemo.stl";
	UppLog() << "\nSaving " << file;
	SaveStlBin(AppendFileNameX(GetDesktopFolder(), file), s, 1);
	
}

CONSOLE_APP_MAIN 
{
	StdLogSetup(LOG_COUT|LOG_FILE);
	SetExitCode(0);
	
	UppLog() << "Surface demo and test";
	
	try {
		bool test = CommandLine().size() > 0 && CommandLine()[0] == "-test";
		
		TestMesh();
		
		//TestSurfaceX_Calc();
		//TestSurfaceX_TransRot();
		
	  
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

