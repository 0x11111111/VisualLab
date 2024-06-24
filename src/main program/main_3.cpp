#include "stdafx.h"
//#include "vtkRenderPipeline.h"
//#include "meshTransform.h"
//#include "simpleRender.h"
//#include "pointcloud_front_recon.h"
//
//#include "IOManip.hpp"
//#include <MRMesh/MRMeshLoad.h>
//#include <MRMesh/MRId.h>
//#include <MRMesh/MRMesh.h>
//#include <MRMesh/MRBitSetParallelFor.h>
//#include <MRMesh/MRMeshTopology.h>
//#include <MRMesh/MRExpected.h>
//#include <MRMesh/MRICP.h>
//#include <MRMesh/MRMeshOrPoints.h>
//#include <MRMesh/MRRingIterator.h>
//
//#include <MRMesh/MREdgePaths.h>
//#include <MRMesh/MRRegionBoundary.h>
//#include <MRMesh/MRMeshPart.h>
//#include <MRMesh/MRMeshComponents.h>
//#include <MRMesh/MRPositionVertsSmoothly.h>
//#include <MRMesh/MRToolPath.h>
//
//
////#include <MRMesh/MRPointCloud.h>
////#include <MRMesh/MRAlphaShape.h>
//
//#include <boost/property_map/property_map.hpp>
//#include <CGAL/boost/graph/Face_filtered_graph.h>
//#include <boost/graph/connected_components.hpp>
//#include <CGAL/boost/graph/helpers.h>
//#include <CGAL/boost/graph/copy_face_graph.h>
//#include <CGAL/Polygon_mesh_processing/measure.h>
//#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
//#include <CGAL/mesh_segmentation.h>
//#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
//#include <CGAL/AABB_tree.h>
//#include <CGAL/AABB_traits.h>
//#include <CGAL/AABB_face_graph_triangle_primitive.h>
//#include <CGAL/Bbox_3.h>
//#include <CGAL/Polyhedron_3.h>
//#include <CGAL/compute_average_spacing.h>
//#include <CGAL/Point_set_3.h>
//#include <CGAL/property_map.h>
//#include <CGAL/IO/PLY.h>
//#include <CGAL/IO/Complex_2_in_triangulation_3_file_writer.h>
//#include <CGAL/poisson_surface_reconstruction.h>
//#include <CGAL/IO/read_points.h>
//#include <CGAL/convex_hull_3.h>
//
//#include <CGAL/Advancing_front_surface_reconstruction.h>
//#include <CGAL/tuple.h>
//#include <boost/lexical_cast.hpp>
//
//#include <vtkIterativeClosestPointTransform.h>
//#include <vtkTransformPolyDataFilter.h>
//#include <vtkMatrix4x4.h>
//#include <vtkPolyDataWriter.h>
//#include <vtkLandmarkTransform.h>
//#include <vtkAppendPolyData.h>
//
//vtkRenderPipeline* pipeline;
//typedef boost::graph_traits<SurfaceMesh>::faces_size_type faces_size_type;
//typedef SurfaceMesh::Property_map<face_descriptor, faces_size_type> FCCmap;
//typedef CGAL::Face_filtered_graph<SurfaceMesh> Filtered_graph;
//typedef CGAL::Point_set_3<Point_3> Point_set;
//typedef CGAL::Polyhedron_3<K> Polyhedron;
//typedef std::pair<Point_3, Vector_3> Pwn;
//typedef std::array<std::size_t, 3> Facet;
//
///**
// * @brief Generate a 4-digit number string with leading zeros.
// *
// * This function takes an integer and converts it into a string representation
// * with a fixed width of 4 characters. If the number has less than 4 digits,
// * leading zeros are added to pad the string to the desired width.
// *
// * @param number The input integer to be converted.
// * @return A string representation of the input number with leading zeros.
// *
// * @note The function uses std::ostringstream, std::setw(), and std::setfill()
// *       to format the output string.
// *
// * @example
// *   int num = 42;
// *   std::string num_str = generate_leading_zero_number_str(num);
// *   // num_str will be "0042"
// */
//std::string generate_leading_zero_number_str(int number)
//{
//	std::ostringstream stream;
//	stream << std::setw(4) << std::setfill('0') << number;
//	return stream.str();
//}
//
//void RightPress(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
//{
//	//std::cout<<"Right Press" << endl;
//}
//void RightRelease(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
//{
//	//std::cout<<"Right Released" << endl;
//}
//
//void LeftPress(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
//{
//	//std::cout<<"Left Press" << endl;
//}
//
//void MouseMove(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
//{
//	
//}
//
//void LeftRelease(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
//{
//}
//
//namespace fs = std::filesystem;
//
//auto GetArchTeeth(fs::path vtp_path)
//{
//	auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
//	reader->SetFileName(vtp_path.string().c_str());
//	reader->Update();
//	auto m_teethPolyData = reader->GetOutput();
//	std::vector<int> m_splitPolyDataFlag;
//	std::vector<vtkSmartPointer<vtkPolyData>> m_labeledPolyData;
//	std::vector<vtkSmartPointer<vtkPoints>> m_labeledPoints;
//	std::vector<vtkSmartPointer<vtkCellArray>> m_labeledTriangles;
//	auto num_points = m_teethPolyData->GetNumberOfPoints();
//	auto num_cells = m_teethPolyData->GetNumberOfCells();
//
//	std::vector<std::vector<int>> m_splitPolydataVisit;
//	vtkSmartPointer<vtkCellData> celldata = m_teethPolyData->GetCellData();
//	vtkSmartPointer<vtkDataArray> labels = celldata->GetArray("Label");
//	vtkNew<vtkTriangle> triangle;
//
//	m_labeledPolyData.resize(20);
//	m_labeledPoints.resize(20);
//	m_labeledTriangles.resize(20);
//	for (int i = 0; i < 20; ++i)
//	{
//		m_labeledPolyData[i] = vtkSmartPointer<vtkPolyData>::New();
//		m_labeledPoints[i] = vtkSmartPointer<vtkPoints>::New();
//		m_labeledTriangles[i] = vtkSmartPointer<vtkCellArray>::New();
//	}
//	m_splitPolyDataFlag.resize(20, 0);
//	m_splitPolydataVisit.resize(20, std::vector<int>(m_teethPolyData->GetPoints()->GetNumberOfPoints(), -1));
//
//	//std::cout << labels->GetSize() << endl;
//
//	// Split the polydata based on the label types
//	for (vtkIdType i = 0; i < labels->GetSize(); i++)
//	{
//		// tmp is a double array that contains an integer value (in this case, only one value)
//		int label = static_cast<int>(labels->GetTuple(i)[0]);
//
//		// polydata->GetCell(i) retrieves the triangle face, and polydata->GetCell(i)->GetPointId(0) retrieves the id of the 0th point
//		for (int j = 0; j < 3; j++)
//		{
//			if (m_splitPolydataVisit[label][m_teethPolyData->GetCell(i)->GetPointId(j)] == -1)
//			{
//				auto face_i = m_teethPolyData->GetCell(i); // Ids of the point_set of the i-th triangle face, which are three int point ids
//				auto point_id_j = face_i->GetPointId(j); // The id of the 0th point of the triangle face (output is the 0th value of the three int point ids)
//				auto new_point_id = m_splitPolyDataFlag[label]; // The initial value of flag is 0, and it outputs the id value in the new container
//				m_labeledPoints[label]->InsertPoint(new_point_id, m_teethPolyData->GetPoint(point_id_j));
//				triangle->GetPointIds()->SetId(j, new_point_id); // Set the id values of the new triangle face
//				m_splitPolydataVisit[label][point_id_j] = new_point_id;
//				m_splitPolyDataFlag[label]++;
//			}
//			else
//			{
//				triangle->GetPointIds()->SetId(j, m_splitPolydataVisit[label][m_teethPolyData->GetCell(i)->GetPointId(j)]);
//			}
//		}
//
//		m_labeledTriangles[label]->InsertNextCell(triangle);
//	}
//
//	// Set the labeled point_set and triangles for each label
//	for (int i = 0; i < 20; i++)
//	{
//		//std::cout << m_splitPolyDataFlag[i] << endl;
//		//std::cout << i << " " << m_labeledPoints[i]->GetNumberOfPoints() << " " << m_labeledTriangles[i]->GetNumberOfCells() << endl;
//		m_labeledPolyData[i]->SetPoints(m_labeledPoints[i]);
//		m_labeledPolyData[i]->SetPolys(m_labeledTriangles[i]);
//	}
//
//	vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
//
//	// Skip the first element in the labeledPolyData vector
//	for (int i = 1; i < 20; i++)
//	{
//		appendFilter->AddInputData(m_labeledPolyData[i]);
//	}
//
//	appendFilter->Update();
//
//	vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();
//	output->ShallowCopy(appendFilter->GetOutput());
//	return output;
//}
//
//int main()
//{
//	using namespace MR;
//	pipeline = new vtkRenderPipeline();
//
//	// 设置文件路径
//	fs::path base_dir = "data/02/";
//	fs::path absolute_path = fs::absolute(base_dir);
//	fs::path cbct_folder_path = absolute_path / "ct.nii";
//	fs::path nii_file_path = cbct_folder_path / "ct.nii";
//	fs::path intraoral_scan_folder_path = absolute_path / "口扫";
//
//	//SurfaceMesh cbct_sm;
//	//Point_set cbct_ps;
//	//CGAL::IO::read_PLY((cbct_folder_path / "extracted_surface_pc.ply").string(), cbct_ps);
//
//	const std::string input_filename = (cbct_folder_path / "extracted_surface_pc.xyz").string();
//	const std::string output_filename = (cbct_folder_path / "reconstructed_surface.off").string();
//
//	Recon(input_filename, output_filename);
//
//	//Mesh cbct_mr = MeshLoad::fromAnySupportedFormat((cbct_folder_path / "jaw_cut.ply").string()).value();
//
//	//PointCloud cbct_pc;
//	//for (auto& v : cbct_mr.topology.getValidVerts())
//	//{
//	//	cbct_pc.addPoint(cbct_mr.point_set[v]);
//	//}
//
//	//auto triangulation = findAlphaShapeAllTriangles(cbct_pc, 0.5);
//
//	//auto alpha_shape_mr = Mesh::fromTriangles(cbct_pc.point_set, triangulation);
//
//	//MeshSave::toAnySupportedFormat(alpha_shape_mr, (cbct_folder_path / "alpha_shape.ply").string());
//
//	//auto alpha_shape_pd = MRMeshToPolyData(alpha_shape_mr);
//
//	//RenderPolydata(alpha_shape_pd, pipeline->Renderer);
//	
//	//// Set up the camera and interactor.
//	//pipeline->Renderer->GetActiveCamera()->SetParallelProjection(1);
//	//pipeline->Renderer->ResetCamera();
//	//// Set up the callback functions for mouse events.
//	//pipeline->addObserver(vtkCommand::LeftButtonPressEvent, LeftPress);
//	//pipeline->addObserver(vtkCommand::MouseMoveEvent, MouseMove);
//	//pipeline->addObserver(vtkCommand::LeftButtonReleaseEvent, LeftRelease);
//	//pipeline->addObserver(vtkCommand::RightButtonPressEvent, RightPress);
//	//pipeline->addObserver(vtkCommand::RightButtonReleaseEvent, RightRelease);
//
//	//pipeline->RenderWindowInteractor->Start();
//
//	// Clean up the pipeline after each run.
//	delete pipeline;
//
//	return 0;
//}