#include <iostream>
#include <fstream>
#include <algorithm>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/tuple.h>
#include <boost/lexical_cast.hpp>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3  Point_3;
typedef std::array<std::size_t, 3> Facet;
namespace std {
	std::ostream&
		operator<<(std::ostream& os, const Facet& f)
	{
		os << "3 " << f[0] << " " << f[1] << " " << f[2];
		return os;
	}
}

void Recon(const std::string input,const std::string output)
{
	std::ifstream in(CGAL::data_file_path(input));
	std::vector<Point_3> points;
	std::vector<Facet> facets, clean_facets;
	std::copy(std::istream_iterator<Point_3>(in), std::istream_iterator<Point_3>(), std::back_inserter(points));
	CGAL::advancing_front_surface_reconstruction(points.begin(), points.end(), std::back_inserter(facets), 1.0, 0.52);

	for (auto& f : facets)
	{
		double edge_length0 = sqrt(squared_distance(points[f[0]], points[f[1]]));
		double edge_length1 = sqrt(squared_distance(points[f[1]], points[f[2]]));
		double edge_length2 = sqrt(squared_distance(points[f[2]], points[f[0]]));
		if (edge_length0 > 2.0 || edge_length1 > 2.0 || edge_length2 > 2.0)
		{

		}
		else
		{
			clean_facets.push_back(f);
		}
	}

	std::ofstream ofs(output);
	if (!ofs) {
		std::cerr << "Error opening file for writing." << std::endl;
		return ;
	}
	ofs << "OFF\n" << points.size() << " " << clean_facets.size() << " 0\n";
	std::copy(points.begin(), points.end(), std::ostream_iterator<Point_3>(ofs, "\n"));
	std::copy(clean_facets.begin(), clean_facets.end(), std::ostream_iterator<Facet>(ofs, "\n"));
	ofs.close();
}

//usage
//std::string input = "data/up6/0456/0456pred.xyz";
//std::string output = "data/up6/0456/output.off";
//Recon(input, output);