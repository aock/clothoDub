#pragma once

#include <vector>
#include <array>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <iostream>

extern "C" {
#include <dubins.h>
}
#include <Clothoid.h>



class ClothoDub {
public:
    ClothoDub(double min_turning_radius, bool use_clothoids = true, bool use_dubins = true);

    void setDubinsSampleMultiplicator(int sample_mult);

    std::vector< std::array<double,3> > calculatePath(
            std::array<double,3> q0,
            std::array<double,3> q1,
            int samples = 1000
        );

private:

    void convert_and_append(const std::vector<double>& X, const std::vector<double>& Y,
        std::vector< std::array<double, 3> >& out);

    double m_min_turning_radius;
    int m_dubins_sample_multi;
    bool m_use_clothoids;
    bool m_use_dubins;
};