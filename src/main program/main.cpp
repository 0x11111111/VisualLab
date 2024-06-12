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

#include <MRMesh/MRVoxelsLoad.h>
#include <MRMesh/MRObjectVoxels.h>
#include <MRMesh/MRMeshFwd.h>

#include <filesystem>

vtkRenderPipeline* pipeline;
namespace fs = std::filesystem;

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
	

int main()
{
	using namespace MR;
	pipeline = new vtkRenderPipeline();

	fs::path tiff_path("data/tiff_folder02");
	fs::path mesh_save_path("data/reconstructed02.ply");

	auto voxel_loaded = VoxelsLoad::loadTiffDir(
		{
			tiff_path,
			Vector3f(.3f, .3f, .3f),
			VoxelsLoad::GridType::DenseGrid,
			{}
		});

	auto vdb = voxel_loaded.value();

	auto vdb_dims = vdb.dims;
	std::cout << "VDB dimensions: " << vdb_dims.x << " " << vdb_dims.y << " " << vdb_dims.z << std::endl;
	auto min_x = 0;
	auto min_y = 0;
	auto min_z = vdb.dims.z / 16 * 3;
	auto max_x = vdb.dims.x / 2;
	auto max_y = vdb.dims.y;
	auto max_z = vdb.dims.z / 16 * 5;


	auto active_bounds = Box3i({ min_x, min_y, min_z }, { max_x, max_y, max_z });

	auto obj_voxels = ObjectVoxels::ObjectVoxels();
	obj_voxels.construct(vdb);
	obj_voxels.setActiveBounds(active_bounds);
	//obj_voxels.setDualMarchingCubes(true, false);
	obj_voxels.setIsoValue(100.f);

	auto mesh_recon = *obj_voxels.mesh();

	MeshSave::toAnySupportedFormat(mesh_recon, mesh_save_path);
	
	//// Set up the camera and interactor.
	//pipeline->Renderer->GetActiveCamera()->SetParallelProjection(1);
	//pipeline->Renderer->ResetCamera();
	//// Set up the callback functions for mouse events.
	//pipeline->addObserver(vtkCommand::LeftButtonPressEvent, LeftPress);
	//pipeline->addObserver(vtkCommand::MouseMoveEvent, MouseMove);
	//pipeline->addObserver(vtkCommand::LeftButtonReleaseEvent, LeftRelease);
	//pipeline->addObserver(vtkCommand::RightButtonPressEvent, RightPress);
	//pipeline->addObserver(vtkCommand::RightButtonReleaseEvent, RightRelease);

	//pipeline->RenderWindowInteractor->Start();

	// Clean up the pipeline after each run.
	delete pipeline;
}