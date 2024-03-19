#include"stdafx.h"

vtkSmartPointer<vtkActor> RenderPolydata(vtkSmartPointer<vtkPolyData> polydata,vtkSmartPointer<vtkRenderer> renderer, double R=1.0, double G=1.0, double B=1.0, double A=1.0)
{
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	if (!polydata) return actor;

	vtkSmartPointer<vtkPolyDataNormals> vtkNormal = vtkSmartPointer<vtkPolyDataNormals>::New();
	vtkNormal->SetInputData(polydata);
	vtkNormal->SetComputePointNormals(1);
	vtkNormal->SetComputeCellNormals(0);
	vtkNormal->SetAutoOrientNormals(1);
	vtkNormal->SetSplitting(0);
	vtkNormal->FlipNormalsOn();
	vtkNormal->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(vtkNormal->GetOutput());
	mapper->Update();

	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(R, G, B);
	actor->GetProperty()->SetAmbient(0.5);
	actor->GetProperty()->SetSpecularPower(100);
	actor->GetProperty()->SetSpecular(0.5);
	actor->GetProperty()->SetDiffuse(0.5);
	actor->GetProperty()->SetOpacity(A);
	actor->PickableOff();
	renderer->AddActor(actor);
	return actor;
}

template<typename T>
vtkSmartPointer<vtkActor> RenderSphere(T pt, double radius, vtkSmartPointer<vtkRenderer> renderer, double R = 1.0, double G = 1.0, double B = 1.0, double A = 1.0)
{
	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	sphere->SetCenter(static_cast<double>(pt[0]), static_cast<double>(pt[1]), static_cast<double>(pt[2]));
	sphere->SetRadius(radius);
	sphere->SetPhiResolution(16);
	sphere->SetThetaResolution(16);
	sphere->Update();
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(sphere->GetOutput());
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(R, G, B);
	actor->GetProperty()->SetAmbient(0.5);
	actor->GetProperty()->SetSpecularPower(100);
	actor->GetProperty()->SetSpecular(0.5);
	actor->GetProperty()->SetDiffuse(0.5);
	actor->GetProperty()->SetOpacity(A);
	actor->PickableOff();
	renderer->AddActor(actor);
	return actor;
}

template<typename T>
vtkSmartPointer<vtkActor> RenderLine(T p1, T p2, vtkSmartPointer<vtkRenderer> renderer, double R = 1.0, double G = 1.0, double B = 1.0, double A = 1.0)
{
	vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
	line->SetPoint1(p1[0], p1[1], p1[2]);
	line->SetPoint2(p2[0], p2[1], p2[2]);
	line->Update();
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(line->GetOutput());
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(R, G, B);
	actor->GetProperty()->SetAmbient(0.5);
	actor->GetProperty()->SetSpecularPower(100);
	actor->GetProperty()->SetSpecular(0.5);
	actor->GetProperty()->SetDiffuse(0.5);
	actor->GetProperty()->SetOpacity(A);
	actor->PickableOff();
	renderer->AddActor(actor);
	return actor;
}

template<typename T>
vtkSmartPointer<vtkActor> RenderPlane(T p, T v, vtkSmartPointer<vtkRenderer> renderer, double R = 1.0, double G = 1.0, double B = 1.0, double A = 1.0)
{
	T v1(0, v[2], -v[1]);
	T v2 (-v[1] * v[1] - v[2] * v[2], v[0] * v[1], v[0] * v[2]);
	v1.normalize();
	v2.normalize();
	//p = p -(v1+v2)*sqrt(2);
	vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
	plane->SetPoint1(v1[0], v1[1], v1[2]);
	plane->SetPoint2(v2[0], v2[1], v2[2]);
	plane->SetOrigin(p[0], p[1], p[2]);

	plane->SetResolution(1, 1);
	plane->SetProgressText("Rendering Plane");
	plane->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(plane->GetOutput());
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(R, G, B);
	actor->GetProperty()->SetAmbient(0.5);
	actor->GetProperty()->SetSpecularPower(100);
	actor->GetProperty()->SetSpecular(0.5);
	actor->GetProperty()->SetDiffuse(0.5);
	actor->GetProperty()->SetOpacity(A);
	actor->PickableOff();
	renderer->AddActor(actor);
	return actor;
}