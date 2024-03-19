#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <omp.h>
#include <vector>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <map>
#include <vtkOBBTree.h>
#include <vtkPlaneSource.h>
#include <vtkLineSource.h>
#include <vtkTubeFilter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkFeatureEdges.h>
#include <vtkSTLReader.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkSTLWriter.h>
#include <vtkPLYWriter.h>
#include <vtkNamedColors.h>
#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkMath.h>
#include <vtkArrowSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCallbackCommand.h>
#include <vtkCellPicker.h>
#include <vtkPolyDataNormals.h>
#include <vtkSphereSource.h>
#include <vtkPolyLine.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkMath.h>
#include <vtkCellData.h>
#include <vtkImageData.h>
#include <vtkPNGWriter.h>
#include <vtkAutoInit.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/Surface_mesh_deformation.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh_deformation.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Segment_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/extrude.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Polygon_2_algorithms.h>
#include "vectorAlgorithm.h"
#include "Timer.hpp"
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)

typedef CGAL::Simple_cartesian<double>		                    K;
typedef K                                                       Kernel;
//3D
typedef K::Point_3                                              Point_3;
typedef K::Vector_3                                             Vector_3;
typedef K::Segment_3                                            Segment_3;
typedef K::Triangle_3                                           Triangle_3;
typedef K::Plane_3                                              Plane_3;
typedef K::Ray_3                                                Ray_3;
typedef K::Line_3                                               Line_3;
typedef K::Sphere_3                                             Sphere_3;
//2D
typedef K::Vector_2                                             Vector_2;
typedef K::Point_2                                              Point_2;
typedef K::Segment_2                                            Segment_2;
typedef K::Triangle_2                                           Triangle_2;
typedef K::Line_2                                               Line_2;
typedef K::Ray_2                                                Ray_2;
typedef K::Circle_2                                             Circle_2;
//Mesh
typedef CGAL::Surface_mesh<K::Point_3>                          SurfaceMesh;
typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor     vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor   halfedge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor       face_descriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor       edge_descriptor;

typedef CGAL::AABB_face_graph_triangle_primitive<SurfaceMesh> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

namespace PMP = CGAL::Polygon_mesh_processing;