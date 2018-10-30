#include <clothodub/clothodub.hpp>

ClothoDub::ClothoDub(double min_turning_radius)
:m_min_turning_radius(min_turning_radius)
{

}

std::vector<double[3]> ClothoDub::calculatePath(
    double q0[3],
    double q1[3],
    int samples)
{ 
    std::vector<double[3]> out_path;

    DubinsPath path;
    dubins_shortest_path(&path, q0, q1, m_min_turning_radius);

    // double path_len = dubins_path_length(&path);
    // std::cout << "Dubins path length: " << path_len << std::endl;

    for(int i=0; i<3; i++)
    {
        // double dubins_segment_length = dubins_segment_length(path, i);
    }

    return out_path;
}
