#include <clothodub/clothodub.hpp>
#include <iostream>

int main(int argc, char** argv)
{
    std::cout << "hello" << std::endl;

    ClothoDub clo = ClothoDub(5.0);

    double src_pose[3] = {0.0, 0.0, 0.0};

    double target_pose[3] = {5.0, 1.0, -2.0};

    std::vector<double[3]> path = clo.calculatePath(src_pose, target_pose);
    

    std::cout << "path size: " << path.size() << std::endl;
}