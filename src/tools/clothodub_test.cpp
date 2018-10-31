#include <clothodub/clothodub.hpp>
#include <iostream>

int main(int argc, char** argv)
{

    ClothoDub clo = ClothoDub(5.0);

    std::array<double,3> src_pose = {0.0, 0.0, 0.0};

    std::array<double,3> target_pose = {0.0, 10.0, -2.0};


    clo.setDubinsSampleMultiplicator(1);
    std::vector<std::array<double,3> > path = clo.calculatePath(src_pose, target_pose);
    

    std::cout << "path size: " << path.size() << std::endl;

    // for(int i=0; i<path.size(); i++)
    // {
    //     std::cout << i << ": (" << path[i][0] << "," << path[i][1] << ")" << std::endl;
    // }
}