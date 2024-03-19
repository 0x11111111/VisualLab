#include"stdafx.h"
#include"vtkRenderPipeline.h"
#include"meshTransform.h"
#include"simpleRender.h"
void LeftPress(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
	cout<<"Left Press" << endl;
}

void MouseMove(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
	
}

void writePNG(SurfaceMesh& sm)
{
	Tree tree(faces(sm).first, faces(sm).second, sm);
	double x_min = std::numeric_limits<double>::max();
	double x_max = std::numeric_limits<double>::min();
	double z_min = std::numeric_limits<double>::max();
	double z_max = std::numeric_limits<double>::min();
	double y_max = std::numeric_limits<double>::min();
	for (auto v : sm.vertices())
	{
		Point_3 p = sm.point(v);
		x_min = std::min(x_min, p.x());
		x_max = std::max(x_max, p.x());
		z_min = std::min(z_min, p.z());
		z_max = std::max(z_max, p.z());
		y_max = std::max(y_max, p.y());
	}
	double max;
	if ((x_max - x_min) > (z_max - z_min))
		max = x_max - x_min;
	else
		max = z_max - z_min;

	vtkSmartPointer< vtkImageData> image = vtkSmartPointer< vtkImageData>::New();
	image->SetDimensions(1000, 1000, 1);
	image->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
	int dim[3];
	image->GetDimensions(dim);
	cout << dim[0] << " " << dim[1] << " " << dim[2] << endl;
	std::vector<std::vector<double>> depth(dim[0], std::vector<double>(dim[1], 0));
	for (int x = 0; x < dim[0]; x++)
	{
		for (int z = 0; z < dim[1]; z++)
		{
			double x_ = x_min + (x_max - x_min) * x / dim[0];
			double z_ = z_min + (z_max - z_min) * z / dim[1];
			Ray_3 ray_query(Point_3(x_, y_max, z_), Vector_3(0, -1, 0));
			auto intersection = tree.first_intersection(ray_query);
			const Point_3* p;
			if (intersection && boost::get<Point_3>(&(intersection->first)))
			{
				p = boost::get<Point_3>(&(intersection->first));
				depth[x][z] = y_max - p->y();
			}
			else
			{
				depth[x][z] = 0;
			}
		}
	}
	double depth_max = 0;
	for (int x = 0; x < 1000; x++)
	{
		for (int z = 0; z < 1000; z++)
		{
			depth_max = std::max(depth_max, depth[x][z]);
		}
	}
	for (int x = 0; x < 1000; x++)
	{
		for (int z = 0; z < 1000; z++)
		{
			if (depth[x][z] == 0)
			{
				unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x, z, 0));
				pixel[0] = static_cast<int>(depth[x][z] / depth_max * 255); // 修改红色通道的值
				pixel[1] = static_cast<int>(depth[x][z] / depth_max * 255);   // 修改绿色通道的值
				pixel[2] = static_cast<int>(depth[x][z] / depth_max * 255);   // 修改蓝色通道的值
			}
			else
			{
				unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x, z, 0));
				pixel[0] = 255 - static_cast<int>(depth[x][z] / depth_max * 255); // 修改红色通道的值
				pixel[1] = 255 - static_cast<int>(depth[x][z] / depth_max * 255);   // 修改绿色通道的值
				pixel[2] = 255 - static_cast<int>(depth[x][z] / depth_max * 255);   // 修改蓝色通道的值
			}
		}
	}
	vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName("output.png");
	writer->SetInputData(image);
	writer->Write();
}

SurfaceMesh mesh;
void LeftRelease(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
	writePNG(mesh);
}



vtkRenderPipeline* pipeline;

int main()
{
	pipeline = new vtkRenderPipeline();

	CGAL::IO::read_polygon_mesh("data/test.stl",mesh);
	RenderPolydata(CGAL_Surface_Mesh2VTK_PolyData(mesh), pipeline->Renderer,1,1,1,1);
	
	pipeline->Renderer->GetActiveCamera()->SetParallelProjection(1);
	pipeline->Renderer->ResetCamera();
	pipeline->addObserver(vtkCommand::LeftButtonPressEvent, LeftPress);
	pipeline->addObserver(vtkCommand::MouseMoveEvent, MouseMove);
	pipeline->addObserver(vtkCommand::LeftButtonReleaseEvent, LeftRelease);

	pipeline->RenderWindowInteractor->Start();
}