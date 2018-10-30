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

    double step_size = 5.0;

    dubins_path_sample_many(&path,  0.1, printConfiguration, NULL);

    return out_path;
}