#pragma once

#include <vector>
#include <dubins.h>
#include <Clothoid.h>


class ClothoDub {
public:
    ClothoDub(double min_turning_radius);

    std::vector<double[3]> calculatePath(
            double q0[3],
            double q1[3],
            int samples = 1000
        );

private:
    double m_min_turning_radius;
};