#include "stdafx.h"
#include "vtkRenderPipeline.h"
#include "meshTransform.h"
#include "simpleRender.h"
#include "IOManip.hpp"
#include <MRMesh/MRMeshLoad.h>
#include <MRMesh/MRId.h>
#include <MRMesh/MRMesh.h>
#include <MRMesh/MRBitSetParallelFor.h>
#include <MRMesh/MRMeshTopology.h>
#include <MRMesh/MRExpected.h>
#include <MRMesh/MRICP.h>
#include <MRMesh/MRMeshOrPoints.h>
#include <MRMesh/MRRingIterator.h>

#include <MRMesh/MREdgePaths.h>
#include <MRMesh/MRRegionBoundary.h>
#include <MRMesh/MRMeshPart.h>
#include <MRMesh/MRMeshComponents.h>
#include <MRMesh/MRBox.h>
#include <MRMesh/MRObjectVoxels.h>
#include <MRMesh/MRVoxelsLoad.h>

#include <MRMesh/MRRegionBoundary.h>
#include <MRMesh/MRMeshFillHole.h>
#include <MRMesh/MRPositionVertsSmoothly.h>
#include <MRMesh/MRMeshDecimate.h>
#include <MRMesh/MRFixSelfIntersections.h>
#include <MRMesh/MRMeshCollide.h>
#include <MRMesh/MRMeshIntersect.h>
#include <MRMesh/MRLine3.h>
#include <MRMesh/MRMeshComponents.h>
#include <MRMesh/MRMeshPart.h>
#include <MRMesh/MRDenseBox.h>

#include <boost/property_map/property_map.hpp>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <boost/graph/connected_components.hpp>
#include <CGAL/boost/graph/helpers.h>
#include <ranges>
#include <cmath>

#include <vtkIterativeClosestPointTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkMatrix4x4.h>
#include <vtkPolyDataWriter.h>
#include <vtkLandmarkTransform.h>
#include <vtkAppendPolyData.h>

vtkRenderPipeline* pipeline;
typedef boost::graph_traits<SurfaceMesh>::faces_size_type faces_size_type;
typedef SurfaceMesh::Property_map<face_descriptor, faces_size_type> FCCmap;
typedef CGAL::Face_filtered_graph<SurfaceMesh> Filtered_graph;
/**
 * @brief Generate a 4-digit number string with leading zeros.
 *
 * This function takes an integer and converts it into a string representation
 * with a fixed width of 4 characters. If the number has less than 4 digits,
 * leading zeros are added to pad the string to the desired width.
 *
 * @param number The input integer to be converted.
 * @return A string representation of the input number with leading zeros.
 *
 * @note The function uses std::ostringstream, std::setw(), and std::setfill()
 *       to format the output string.
 *
 * @example
 *   int num = 42;
 *   std::string num_str = generate_leading_zero_number_str(num);
 *   // num_str will be "0042"
 */
std::string generate_leading_zero_number_str(int number)
{
	std::ostringstream stream;
	stream << std::setw(4) << std::setfill('0') << number;
	return stream.str();
}

void RightPress(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
	//std::cout<<"Right Press" << endl;
}
void RightRelease(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
	//std::cout<<"Right Released" << endl;
}

void LeftPress(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
	//std::cout<<"Left Press" << endl;
}

void MouseMove(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
	
}

void LeftRelease(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
}

namespace fs = std::filesystem;

auto GetArchTeeth(fs::path vtp_path)
{
	auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
	reader->SetFileName(vtp_path.string().c_str());
	reader->Update();
	auto m_teethPolyData = reader->GetOutput();
	std::vector<int> m_splitPolyDataFlag;
	std::vector<vtkSmartPointer<vtkPolyData>> m_labeledPolyData;
	std::vector<vtkSmartPointer<vtkPoints>> m_labeledPoints;
	std::vector<vtkSmartPointer<vtkCellArray>> m_labeledTriangles;
	auto num_points = m_teethPolyData->GetNumberOfPoints();
	auto num_cells = m_teethPolyData->GetNumberOfCells();

	std::vector<std::vector<int>> m_splitPolydataVisit;
	vtkSmartPointer<vtkCellData> celldata = m_teethPolyData->GetCellData();
	vtkSmartPointer<vtkDataArray> labels = celldata->GetArray("Label");
	vtkNew<vtkTriangle> triangle;

	m_labeledPolyData.resize(20);
	m_labeledPoints.resize(20);
	m_labeledTriangles.resize(20);
	for (int i = 0; i < 20; ++i)
	{
		m_labeledPolyData[i] = vtkSmartPointer<vtkPolyData>::New();
		m_labeledPoints[i] = vtkSmartPointer<vtkPoints>::New();
		m_labeledTriangles[i] = vtkSmartPointer<vtkCellArray>::New();
	}
	m_splitPolyDataFlag.resize(20, 0);
	m_splitPolydataVisit.resize(20, std::vector<int>(m_teethPolyData->GetPoints()->GetNumberOfPoints(), -1));

	//std::cout << labels->GetSize() << endl;

	// Split the polydata based on the label types
	for (vtkIdType i = 0; i < labels->GetSize(); i++)
	{
		// tmp is a double array that contains an integer value (in this case, only one value)
		int label = static_cast<int>(labels->GetTuple(i)[0]);

		// polydata->GetCell(i) retrieves the triangle face, and polydata->GetCell(i)->GetPointId(0) retrieves the id of the 0th point
		for (int j = 0; j < 3; j++)
		{
			if (m_splitPolydataVisit[label][m_teethPolyData->GetCell(i)->GetPointId(j)] == -1)
			{
				auto face_i = m_teethPolyData->GetCell(i); // Ids of the points of the i-th triangle face, which are three int point ids
				auto point_id_j = face_i->GetPointId(j); // The id of the 0th point of the triangle face (output is the 0th value of the three int point ids)
				auto new_point_id = m_splitPolyDataFlag[label]; // The initial value of flag is 0, and it outputs the id value in the new container
				m_labeledPoints[label]->InsertPoint(new_point_id, m_teethPolyData->GetPoint(point_id_j));
				triangle->GetPointIds()->SetId(j, new_point_id); // Set the id values of the new triangle face
				m_splitPolydataVisit[label][point_id_j] = new_point_id;
				m_splitPolyDataFlag[label]++;
			}
			else
			{
				triangle->GetPointIds()->SetId(j, m_splitPolydataVisit[label][m_teethPolyData->GetCell(i)->GetPointId(j)]);
			}
		}

		m_labeledTriangles[label]->InsertNextCell(triangle);
	}

	// Set the labeled points and triangles for each label
	for (int i = 0; i < 20; i++)
	{
		//std::cout << m_splitPolyDataFlag[i] << endl;
		//std::cout << i << " " << m_labeledPoints[i]->GetNumberOfPoints() << " " << m_labeledTriangles[i]->GetNumberOfCells() << endl;
		m_labeledPolyData[i]->SetPoints(m_labeledPoints[i]);
		m_labeledPolyData[i]->SetPolys(m_labeledTriangles[i]);
	}

	vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();

	// Skip the first element in the labeledPolyData vector
	for (int i = 4; i < 11; i++)
	{
		appendFilter->AddInputData(m_labeledPolyData[i]);
	}

	appendFilter->Update();

	vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();
	output->ShallowCopy(appendFilter->GetOutput());
	return output;
}

auto extract_sub_mesh_by_fbs = [&](const Mesh& mesh, const FaceBitSet& fbs) {
	std::vector<Triangle3f> pos_triples;
	for (auto& f : mesh.topology.getValidFaces()) {
		if (fbs.test(f)) {
			ThreeVertIds tri_verts;
			mesh.topology.getTriVerts(f, tri_verts);

			Triangle3f triangle3f;
			triangle3f[0] = mesh.points[tri_verts[0]];
			triangle3f[1] = mesh.points[tri_verts[1]];
			triangle3f[2] = mesh.points[tri_verts[2]];

			pos_triples.push_back(triangle3f);
		}
	}
	return Mesh::fromPointTriples(pos_triples, false);
	};

int main()
{
	using namespace MR;
	pipeline = new vtkRenderPipeline();

	// 设置文件路径
	fs::path base_dir = "data/04/";
	fs::path absolute_path = fs::absolute(base_dir);
	fs::path cbct_folder_path = absolute_path / "ct.nii";
	fs::path tiff_folder_path = absolute_path / "tiff";

	/*auto voxel_load = MR::VoxelsLoad::loadTiffDir(
		{
			absolute_path / "tiff",
			{ .3f, .3f, .3f },
			MR::VoxelsLoad::GridType::DenseGrid,
			{}
		}
	);
	auto vdb = voxel_load.value();

	auto dims = vdb.dims;

	auto dim_x = dims.x;
	auto dim_y = dims.y;
	auto dim_z = dims.z;

	auto min_x = 0;
	auto max_x = dim_x;
	auto min_y = 0;
	auto max_y = dim_y / 2;
	auto min_z = 68;
	auto max_z = 150;

	auto active_bounds = Box3i({min_x, min_y, min_z}, { max_x, max_y, max_z });

	MR::ObjectVoxels object_voxels;
	object_voxels.construct(vdb);
	auto& bins = object_voxels.histogram().getBins();

	auto sum = std::accumulate(bins.begin(), bins.end(), 0);
	auto rit = bins.rbegin();
	auto accumulation = 0;

	auto it = bins.begin();
	for (; it != bins.end(); ++it)
	{
		accumulation += *it;
		if (accumulation > sum * 0.99)
		{
			break;
		}
	}

	auto bin_id = std::distance(bins.begin(), it);

	object_voxels.setActiveBounds(active_bounds);
	object_voxels.setIsoValue(bin_id);

	auto m_cbct_mr =*object_voxels.mesh();

	MeshSave::toAnySupportedFormat(m_cbct_mr, "data/03/cbct_mr.ply");

	*/
	auto jaw_mr = MeshLoad::fromAnySupportedFormat(absolute_path / "lower_teeth.ply").value();

	auto jaw_dense_box = DenseBox(jaw_mr);
	auto jaw_dense_corner_1 = jaw_dense_box.corner({ false, true, false });
	auto jaw_dense_corner_2 = jaw_dense_box.corner({ true, true, true });
	auto jaw_dense_center = (jaw_dense_corner_1 + jaw_dense_corner_2) / 2.f;

	RenderPolydata(MRMeshToPolyData(jaw_mr), pipeline->Renderer);

	auto jaw_dense_corner_1_point = double3(jaw_dense_corner_1.x, jaw_dense_corner_1.y, jaw_dense_corner_1.z);
	auto jaw_dense_corner_2_point = double3(jaw_dense_corner_2.x, jaw_dense_corner_2.y, jaw_dense_corner_2.z);
	RenderSphere(jaw_dense_corner_1_point, 1, pipeline->Renderer, 1, 0, 0);
	RenderSphere(jaw_dense_corner_2_point, 1, pipeline->Renderer, 0, 0, 1);

	auto jaw_dense_center_point = double3(jaw_dense_center.x, jaw_dense_center.y, jaw_dense_center.z);
	RenderSphere(jaw_dense_center_point, 1, pipeline->Renderer);



	auto lower_recon_mr = MeshLoad::fromAnySupportedFormat(absolute_path / "lower_recon.ply").value();

	auto lower_recon_dense_box = DenseBox(lower_recon_mr);
	auto lower_recon_dense_corner_1 = lower_recon_dense_box.corner({ false, true, false });
	auto lower_recon_dense_corner_2 = lower_recon_dense_box.corner({ true, true, true });
	auto lower_recon_dense_center = (lower_recon_dense_corner_1 + lower_recon_dense_corner_2) / 2.f;

	auto translation = AffineXf3f::translation(lower_recon_dense_center - jaw_dense_center);

	jaw_mr.transform(translation);

	MeshSave::toAnySupportedFormat(jaw_mr, "data/04/lower_teeth_translated.ply");

	//auto expansion = 1.f;
	//for (auto& v : jaw_mr.topology.getValidVerts())
	//{
	//	Vector3f average_normal(0.f, 0.f, 0.f);
	//	int count = 0;
	//	for (auto& e : orgRing(jaw_mr.topology, v))
	//	{
	//		auto& dst_v = jaw_mr.topology.dest(e);
	//		average_normal += jaw_mr.normal(dst_v);
	//		count++;
	//	}
	//	average_normal /= static_cast<float>(count);
	//	jaw_mr.points[v] += expansion * average_normal;
	//}
	//MeshSave::toAnySupportedFormat(jaw_mr, "data/03/upper_jaw_expanded.ply");

	//auto right_boundary = findRightBoundary(jaw_mr.topology);

	//std::cout << right_boundary.size() << std::endl;

	//FaceBitSet filled_new_faces;

	//for (auto& boundary : right_boundary)
	//{
	//	std::cout << boundary[0] << std::endl;

	//	fillHole(jaw_mr, boundary[0],
	//		{
	//			{},
	//			&filled_new_faces,
	//			FillHoleParams::MultipleEdgesResolveMode::None,
	//			true,
	//			20
	//		}
	//	);
	//}
	//MeshSave::toAnySupportedFormat(jaw_mr, "data/03/upper_jaw_filled.ply");

	//auto recon_mr = MeshLoad::fromAnySupportedFormat(absolute_path / "cbct_mr.ply").value();

	//auto outside_vbs = recon_mr.topology.getValidVerts();

	//for (auto& v : recon_mr.topology.getValidVerts())
	//{
	//	std::vector<MeshIntersectionResult> intersection_results;
	//	rayMeshIntersectAll(
	//		jaw_mr,
	//		Line3f(recon_mr.points[v], recon_mr.normal(v)),
	//		[&intersection_results](const MeshIntersectionResult& result) -> bool
	//		{
	//			intersection_results.push_back(result);
	//			return true;
	//		},
	//		-50.f,
	//		50.f
	//	);
	//	// std::cout << intersection_results.size() << std::endl;

	//	if (intersection_results.size() > 0)
	//	{
	//		auto first_distance = intersection_results[0].distanceAlongLine;
	//		for (auto& intersection_res : intersection_results | std::ranges::views::drop(1))
	//		{
	//			auto distance = intersection_res.distanceAlongLine;
	//			if (std::signbit(first_distance) != std::signbit(distance))
	//			{
	//				outside_vbs.reset(v);
	//				break;
	//			}
	//		}
	//	}
	//}


	//auto outside_fbs = getInnerFaces(recon_mr.topology, outside_vbs);
	//auto outside_largest_components_fbs = MeshComponents::getLargestComponent({ recon_mr, &outside_fbs});

	//auto outside_mr = extract_sub_mesh_by_fbs(recon_mr, outside_largest_components_fbs);

	//MeshSave::toAnySupportedFormat(outside_mr, "data/03/cbct_mr_outside.ply");
	//SelfIntersections::Settings self_intersections_settings;
	//self_intersections_settings.method = SelfIntersections::Settings::Method::CutAndFill;
	//SelfIntersections::fix(jaw_mr, self_intersections_settings);


	/*
	FaceBitSet self_colliding_fbs = findSelfCollidingTrianglesBS(jaw_mr).value();

	dilateRegion(jaw_mr, self_colliding_fbs, .5f);


	FaceBitSet non_colliding_fbs = jaw_mr.topology.getValidFaces() - self_colliding_fbs;

	auto& jaw_non_self_intersection_mr = extract_sub_mesh_by_fbs(jaw_mr, non_colliding_fbs);
	MeshSave::toAnySupportedFormat(jaw_non_self_intersection_mr, "data/03/upper_jaw_non_self_intersection.ply");

	auto right_boundary = findRightBoundary(jaw_non_self_intersection_mr.topology);

	std::cout << right_boundary.size() << std::endl;

	FaceBitSet filled_new_faces;

	for (auto& boundary : right_boundary)
	{
		std::cout << boundary[0] << std::endl;

		fillHole(jaw_non_self_intersection_mr, boundary[0],
			{
				{},
				&filled_new_faces,
				FillHoleParams::MultipleEdgesResolveMode::None,
				true,
				20
			}
		);
	}

	*/

	

	//RemeshSettings remesh_settings;
	//remesh_settings.region = &filled_new_faces;
	//remesh_settings.useCurvature = true;
	//remesh_settings.finalRelaxIters = 10;
	//remesh(m_cbct_mr, remesh_settings);

	//dilateRegion(m_cbct_mr, filled_new_faces, 2.f);

	//auto filled_new_verts = getInnerVerts(m_cbct_mr.topology, &filled_new_faces);

	//positionVertsSmoothly(m_cbct_mr, filled_new_verts);

	//MeshSave::toAnySupportedFormat(m_cbct_mr, "data/03/cbct_mr_filled.ply");

	// Set up the camera and interactor.
	pipeline->Renderer->GetActiveCamera()->SetParallelProjection(1);
	pipeline->Renderer->ResetCamera();
	// Set up the callback functions for mouse events.
	pipeline->addObserver(vtkCommand::LeftButtonPressEvent, LeftPress);
	pipeline->addObserver(vtkCommand::MouseMoveEvent, MouseMove);
	pipeline->addObserver(vtkCommand::LeftButtonReleaseEvent, LeftRelease);
	pipeline->addObserver(vtkCommand::RightButtonPressEvent, RightPress);
	pipeline->addObserver(vtkCommand::RightButtonReleaseEvent, RightRelease);

	pipeline->RenderWindowInteractor->Start();

	// Clean up the pipeline after each run.
	delete pipeline;
}