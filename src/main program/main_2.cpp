#include "stdafx.h"
//#include "vtkRenderPipeline.h"
//#include "meshTransform.h"
//#include "simpleRender.h"
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
//				auto face_i = m_teethPolyData->GetCell(i); // Ids of the points of the i-th triangle face, which are three int point ids
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
//	// Set the labeled points and triangles for each label
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
//	// Extract the cbct surface by rayshooting
//	SurfaceMesh cbct_sm;
//
//	// Read the PLY file
//	if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh((cbct_folder_path / "jaw_cut.ply").string(), cbct_sm)) {
//		std::cerr << "Error: cannot read PLY file." << std::endl;
//		return EXIT_FAILURE;
//	}
//	
//	std::vector<Pwn> points;
//	if (!CGAL::IO::read_points((cbct_folder_path / "jaw_cut.ply").string(), std::back_inserter(points),
//		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
//		.normal_map(CGAL::Second_of_pair_property_map<Pwn>())))
//	{
//		std::cerr << "Error: cannot read input file!" << std::endl;
//		return EXIT_FAILURE;
//	}
//	
//	// 估算平均间距
//	const double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 6, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>()));
//	cout << "Average spacing: " << average_spacing << endl;
//	// 表面重建
//	Polyhedron output_mesh;
//	if (CGAL::poisson_surface_reconstruction_delaunay(
//		points.begin(), points.end(),
//		CGAL::First_of_pair_property_map<Pwn>(),
//		CGAL::Second_of_pair_property_map<Pwn>(),
//		output_mesh, average_spacing))
//	{
//		// 输出网格到文件
//		std::ofstream output("data/extracted_surface.off");
//		output << output_mesh;
//	}
//
//
//
//
//
//	// 使用(0, 1, 0) (0, -1, 0)两个方向的射线进行射线投射 获取外表面
//	//// Construct AABB tree
//	//FaceTree tree(faces(cbct_sm).first, faces(cbct_sm).second, cbct_sm);
//	//tree.accelerate_distance_queries();
//
//	//// Get bounding box of the mesh
//	//CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(cbct_sm);
//
//	//// Vectors to store intersected faces
//	//std::unordered_set<face_descriptor> intersected_faces;
//
//	//// Define step size for x and z traversal
//	//double step_size = 0.05; // You can adjust this step size
//
//	//// Function to process ray intersections
//	//auto process_ray = [&](const Ray_3& ray) {
//	//	Ray_intersection intersection = tree.first_intersection(ray);
//	//	if (intersection) {
//	//		// 获取相交的面描述符
//	//		const Primitive_id& pid = intersection->second;
//	//		face_descriptor f = pid;
//	//		intersected_faces.insert(f);
//	//	}
//	//	};
//
//	//// Traverse the x and z range of the bounding box
//	//for (double x = bbox.xmin(); x <= bbox.xmax(); x += step_size) {
//	//	for (double z = bbox.zmin(); z <= bbox.zmax(); z += step_size) {
//	//		Point_3 origin1(x, bbox.ymin() - 1, z); // Just below the bounding box
//	//		Vector_3 direction1(0, 1, 0); // Upward
//	//		Ray ray1(origin1, direction1);
//
//	//		Point_3 origin2(x, bbox.ymax() + 1, z); // Just above the bounding box
//	//		Vector_3 direction2(0, -1, 0); // Downward
//	//		Ray ray2(origin2, direction2);
//
//	//		process_ray(ray1);
//	//		process_ray(ray2);
//	//	}
//	//}
//
//	//// Create a new surface mesh with only the intersected triangles
//	//SurfaceMesh new_mesh;
//	//std::unordered_map<vertex_descriptor, vertex_descriptor> vertex_map;
//
//	//for (face_descriptor f : intersected_faces) {
//	//	std::vector<vertex_descriptor> new_vertices;
//	//	for (auto v : vertices_around_face(cbct_sm.halfedge(f), cbct_sm)) {
//	//		if (vertex_map.find(v) == vertex_map.end()) {
//	//			vertex_map[v] = new_mesh.add_vertex(cbct_sm.point(v));
//	//		}
//	//		new_vertices.push_back(vertex_map[v]);
//	//	}
//	//	new_mesh.add_face(new_vertices);
//	//}
//
//	//// Write the resulting mesh to a file
//	//std::ofstream out(cbct_folder_path / "ray_projection_result.ply");
//	//CGAL::IO::write_PLY(out, new_mesh);
//
//	//std::cout << "Resulting mesh saved to result.ply" << std::endl;
//
//	//vtkSmartPointer<vtkPolyData> upper_teeth_pd = GetArchTeeth(intraoral_scan_folder_path / "UpperJaw_d_predicted_refined_centered.vtp");
//	//vtkSmartPointer<vtkPolyData> lower_teeth_pd = GetArchTeeth(intraoral_scan_folder_path / "LowerJaw_d_predicted_refined_centered.vtp");
//	//
//	//auto upper_teeth_writer = vtkSmartPointer<vtkPLYWriter>::New();
//	//upper_teeth_writer->SetFileName((cbct_folder_path / "upper_teeth.ply").string().c_str());
//	//upper_teeth_writer->SetInputData(upper_teeth_pd);
//	//upper_teeth_writer->Write();
//
//	//auto lower_teeth_writer = vtkSmartPointer<vtkPLYWriter>::New();
//	//lower_teeth_writer->SetFileName((cbct_folder_path / "lower_teeth.ply").string().c_str());
//	//lower_teeth_writer->SetInputData(lower_teeth_pd);
//	//lower_teeth_writer->Write();
//
//	//std::cout << upper_teeth_pd->GetNumberOfPoints() << std::endl;
//	//std::cout << lower_teeth_pd->GetNumberOfPoints() << std::endl;
//
//	//auto upper_teeth_sm = VTK_PolyData2CGAL_Surface_Mesh(upper_teeth_pd);
//	//auto lower_teeth_sm = VTK_PolyData2CGAL_Surface_Mesh(lower_teeth_pd);
//
//	//PMP::isotropic_remeshing(faces(upper_teeth_sm), 0.2, upper_teeth_sm);
//	//upper_teeth_sm.collect_garbage();
//	//PMP::isotropic_remeshing(faces(lower_teeth_sm), 0.2, lower_teeth_sm);
//	//lower_teeth_sm.collect_garbage();
//
//	////upper_teeth_pd = CGAL_Surface_Mesh2VTK_PolyData(upper_teeth_sm);
//	////lower_teeth_pd = CGAL_Surface_Mesh2VTK_PolyData(lower_teeth_sm);
//
//	////std::cout << upper_teeth_pd->GetNumberOfPoints() << std::endl;
//	////std::cout << lower_teeth_pd->GetNumberOfPoints() << std::endl;
//
//	//auto upper_jaw_pd = vtkSmartPointer<vtkPolyData>::New();
//	//auto lower_jaw_pd = vtkSmartPointer<vtkPolyData>::New();
//	//auto upper_jaw_reader = vtkSmartPointer<vtkSTLReader>::New();
//	//auto lower_jaw_reader = vtkSmartPointer<vtkSTLReader>::New();
//	//upper_jaw_reader->SetFileName((intraoral_scan_folder_path / "UpperJaw.stl").string().c_str());
//	//upper_jaw_reader->Update();
//	//upper_jaw_pd = upper_jaw_reader->GetOutput();
//	//lower_jaw_reader->SetFileName((intraoral_scan_folder_path / "LowerJaw.stl").string().c_str());
//	//lower_jaw_reader->Update();
//	//lower_jaw_pd = lower_jaw_reader->GetOutput();
//
//
//
//	////auto extract_sub_mesh_by_fbs = [&](const Mesh& mesh, const FaceBitSet& fbs) {
//	////	std::vector<Triangle3f> pos_triples;
//	////	for (auto& f : mesh.topology.getValidFaces()) {
//	////		if (fbs.test(f)) {
//	////			ThreeVertIds tri_verts;
//	////			mesh.topology.getTriVerts(f, tri_verts);
//
//	////			Triangle3f triangle3f;
//	////			triangle3f[0] = mesh.points[tri_verts[0]];
//	////			triangle3f[1] = mesh.points[tri_verts[1]];
//	////			triangle3f[2] = mesh.points[tri_verts[2]];
//
//	////			pos_triples.push_back(triangle3f);
//	////		}
//	////	}
//	////	return Mesh::fromPointTriples(pos_triples, false);
//	////	};
//
//	////auto cbct_mr = MeshLoad::fromAnySupportedFormat((cbct_folder_path / "filtered_jaw.ply").string()).value();
//	////auto largest_component_fbs = MeshComponents::getLargestComponent(cbct_mr);
//	////auto largest_component_cbct_mr = extract_sub_mesh_by_fbs(cbct_mr, largest_component_fbs);
//
//	////MeshSave::toAnySupportedFormat(largest_component_cbct_mr, (cbct_folder_path / "largest_component_jaw.ply").string());		
//
//	////auto cbct_mr = MeshLoad::fromAnySupportedFormat((cbct_folder_path / "extracted_jaw.ply").string()).value();
//	///*VertBitSet smooth_cbct_vbs(cbct_mr.topology.numValidVerts());
//	//for (auto& v : cbct_mr.topology.getValidVerts())
//	//{
//	//	if (!cbct_mr.topology.isBdVertex(v))
//	//	{
//	//		smooth_cbct_vbs.set(v);
//	//	}
//	//}
//	//positionVertsSmoothly(cbct_mr, smooth_cbct_vbs);*/
//	////SurfaceMesh cbct_upper_sm;
//	////CGAL::IO::read_PLY((cbct_folder_path / "cbct_upper.ply").string(), cbct_upper_sm);
//	////
//	////std::set<vertex_descriptor> upper_constrained_vertices;
//	////for (vertex_descriptor v : vertices(cbct_upper_sm))
//	////{
//	////	if (is_border(v, cbct_upper_sm))
//	////	{
//	////		upper_constrained_vertices.insert(v);
//	////	}
//	////}
//	///*CGAL::Boolean_property_map<std::set<vertex_descriptor> > upper_vcmap(upper_constrained_vertices);
//	//PMP::smooth_shape(cbct_upper_sm, 0.001, CGAL::parameters::number_of_iterations(5)
//	//	.vertex_is_constrained_map(upper_vcmap));
//	//CGAL::IO::write_PLY((cbct_folder_path / "cbct_upper_smoothed.ply").string(), cbct_upper_sm);
//	//*///MeshSave::toAnySupportedFormat(cbct_mr, (cbct_folder_path / "smoothed_jaw.ply").string());
//
//	//// 读取CBCT扫描模型
//	//auto cbct_upper_reader = vtkSmartPointer<vtkPLYReader>::New();
//	//cbct_upper_reader->SetFileName((cbct_folder_path / "cbct_upper.ply").string().c_str());
//	//cbct_upper_reader->Update();
//	//auto cbct_upper_pd = cbct_upper_reader->GetOutput();
//	//RenderPolydata(cbct_upper_pd, pipeline->Renderer, 1, 1, 1, 1);
//
//
//	////SurfaceMesh cbct_lower_sm;
//	////CGAL::IO::read_PLY((cbct_folder_path / "cbct_lower.ply").string(), cbct_lower_sm);
//
//	////std::set<vertex_descriptor> lower_constrained_vertices;
//	////for (vertex_descriptor v : vertices(cbct_lower_sm))
//	////{
//	////	if (is_border(v, cbct_lower_sm))
//	////	{
//	////		lower_constrained_vertices.insert(v);
//	////	}
//	////}
//	////CGAL::Boolean_property_map<std::set<vertex_descriptor> > lower_vcmap(lower_constrained_vertices);
//	////PMP::smooth_shape(cbct_lower_sm, 0.001, CGAL::parameters::number_of_iterations(5)
//	////	.vertex_is_constrained_map(lower_vcmap));
//	////CGAL::IO::write_PLY((cbct_folder_path / "cbct_lower_smoothed.ply").string(), cbct_lower_sm);
//	//////MeshSave::toAnySupportedFormat(cbct_mr, (cbct_folder_path / "smoothed_jaw.ply").string());
//
//	//// 读取CBCT扫描模型
//	//auto cbct_lower_reader = vtkSmartPointer<vtkPLYReader>::New();
//	//cbct_lower_reader->SetFileName((cbct_folder_path / "cbct_lower.ply").string().c_str());
//	//cbct_lower_reader->Update();
//	//auto cbct_lower_pd = cbct_lower_reader->GetOutput();
//	//RenderPolydata(cbct_lower_pd, pipeline->Renderer, 1, 1, 1, 1);
//
//
//	//std::cout << "CBCT model loaded" << std::endl;
//
//
//	//// VTK ICP配准
//	//auto upper_icp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
//	//upper_icp->SetSource(upper_teeth_pd);
//	//upper_icp->SetTarget(cbct_upper_pd);
//	//upper_icp->GetLandmarkTransform()->SetModeToRigidBody();
//	//upper_icp->SetMaximumNumberOfIterations(200);
//	//upper_icp->StartByMatchingCentroidsOn();
//	//upper_icp->Modified();
//	//upper_icp->Update();
//
//	//// 获取变换矩阵
//	//auto upper_matrix = upper_icp->GetMatrix();
//	//std::cout << "Transformation Matrix:" << std::endl;
//	//upper_matrix->Print(std::cout);
//
//	//// 将变换应用到合并后的牙齿模型
//	//auto upper_transform_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//	//upper_transform_filter->SetInputData(upper_jaw_pd);
//	//upper_transform_filter->SetTransform(upper_icp);
//	//upper_transform_filter->Update();
//
//	//// 输出变换后的牙齿模型
//	//auto upper_writer = vtkSmartPointer<vtkPLYWriter>::New();
//	//upper_writer->SetFileName((cbct_folder_path / "transformed_upper.ply").string().c_str());
//	//upper_writer->SetInputData(upper_transform_filter->GetOutput());
//	//upper_writer->Write();
//
//	//RenderPolydata(upper_transform_filter->GetOutput(), pipeline->Renderer, 0, 1, 1, 0.8);
//
//	//upper_transform_filter->SetInputData(upper_teeth_pd);
//	//upper_transform_filter->SetTransform(upper_icp);
//	//upper_transform_filter->Update();
//
//	//// 输出变换后的牙齿模型
//	//upper_writer->SetFileName((cbct_folder_path / "transformed_teeth_upper.ply").string().c_str());
//	//upper_writer->SetInputData(upper_transform_filter->GetOutput());
//	//upper_writer->Write();
//
//
//	//// ICP配准
//	//auto lower_icp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
//	//lower_icp->SetSource(lower_teeth_pd);
//	//lower_icp->SetTarget(cbct_lower_pd);
//	//lower_icp->GetLandmarkTransform()->SetModeToRigidBody();
//	//lower_icp->SetMaximumNumberOfIterations(200);
//	//lower_icp->StartByMatchingCentroidsOn();
//	//lower_icp->Modified();
//	//lower_icp->Update();
//
//	//// 获取变换矩阵
//	//auto lower_matrix = lower_icp->GetMatrix();
//	//std::cout << "Transformation Matrix:" << std::endl;
//	//lower_matrix->Print(std::cout);
//
//	//// 将变换应用到合并后的牙齿模型
//	//auto lower_transform_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//	//lower_transform_filter->SetInputData(lower_jaw_pd);
//	//lower_transform_filter->SetTransform(lower_icp);
//	//lower_transform_filter->Update();
//
//	//// 输出变换后的牙齿模型
//	//auto lower_writer = vtkSmartPointer<vtkPLYWriter>::New();
//	//lower_writer->SetFileName((cbct_folder_path / "transformed_lower.ply").string().c_str());
//	//lower_writer->SetInputData(lower_transform_filter->GetOutput());
//	//lower_writer->Write();
//
//	//RenderPolydata(lower_transform_filter->GetOutput(), pipeline->Renderer, 1, 1, 0, 0.8);
//
//	//lower_transform_filter->SetInputData(lower_teeth_pd);
//	//lower_transform_filter->SetTransform(lower_icp);
//	//lower_transform_filter->Update();
//
//	//// 输出变换后的牙齿模型
//	//lower_writer->SetFileName((cbct_folder_path / "transformed_teeth_lower.ply").string().c_str());
//	//lower_writer->SetInputData(lower_transform_filter->GetOutput());
//	//lower_writer->Write();
//
//
//
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
//}