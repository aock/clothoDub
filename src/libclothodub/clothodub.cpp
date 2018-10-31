#include <clothodub/clothodub.hpp>

ClothoDub::ClothoDub(double min_turning_radius, bool use_clothoids, bool use_dubins)
:m_min_turning_radius(min_turning_radius)
,m_use_clothoids(use_clothoids)
,m_use_dubins(use_dubins)
,m_dubins_sample_multi(1)
{

}

void ClothoDub::setDubinsSampleMultiplicator(int sample_mult)
{
    m_dubins_sample_multi = sample_mult;
}

std::vector< std::array<double,3> > ClothoDub::calculatePath(
    std::array<double,3> q0,
    std::array<double,3> q1,
    double& length,
    int samples)
{ 
    std::vector< std::array<double,3> > out_path;


    if(m_use_clothoids && m_use_dubins)
    {
        // combination of dubins and clothoid


        // 1) DUBINS
        DubinsPath path;

        dubins_shortest_path(&path, q0.data(),  q1.data() , m_min_turning_radius);

        double path_len = dubins_path_length(&path);
        // std::cout << "Dubins path length: " << path_len << std::endl;

        std::vector< std::array<double,3> > dubins_path;

        dubins_path.push_back(q0);
        double current_len = 0.0;

        for(int i=0; i<3; i++)
        {
            const double dubins_segment_len = dubins_segment_length(&path, i);

            // split segment in m_dubins_sample_multi parts
            for(int j=0; j<m_dubins_sample_multi; j++)
            {
                const double part_len = dubins_segment_len / static_cast<double>(m_dubins_sample_multi);
                current_len += dubins_segment_len;
                // std::cout << "Dubins segment " << i+1 << " "
                //           << "Part " << j+1
                //           << " length: " << part_len << std::endl;

                std::array<double,3> curr_q = {0.0};
                dubins_path_sample(&path, current_len, curr_q.data() );
                // std::cout << curr_q[0] << " " << curr_q[1] << " " << curr_q[2] << std::endl;
                dubins_path.push_back(curr_q);
            }

            
        }

        // CLOTHOID SMOOTHING (G1Fitting)

        double clothoid_path_length = 0.0;

        for(int i=1; i < dubins_path.size(); i++)
        {
            const double x0 = dubins_path[i-1][0];
            const double y0 = dubins_path[i-1][1];
            const double theta0 = dubins_path[i-1][2];

            const double x1 = dubins_path[i][0];
            const double y1 = dubins_path[i][1];
            const double theta1 = dubins_path[i][2];

            double k;
            double dk;
            double L;

            int res = Clothoid::buildClothoid(x0, y0, theta0, x1, y1, theta1, k, dk, L);

            clothoid_path_length += L;
        }

        length = clothoid_path_length;

        if(clothoid_path_length > 0.0)
        {
            int prev_set_samples = 0;
            for(int i=1; i < dubins_path.size(); i++)
            {
                const double x0 = dubins_path[i-1][0];
                const double y0 = dubins_path[i-1][1];
                const double theta0 = dubins_path[i-1][2];

                const double x1 = dubins_path[i][0];
                const double y1 = dubins_path[i][1];
                const double theta1 = dubins_path[i][2];

                double k;
                double dk;
                double L;

                int res = Clothoid::buildClothoid(x0, y0, theta0, x1, y1, theta1, k, dk, L);

                double relative_length = L/clothoid_path_length;

                int num_seg_samples = 0;

                if(i >= dubins_path.size() - 1 )
                {
                    num_seg_samples = samples - prev_set_samples;
                } else {
                    num_seg_samples = samples * relative_length;
                }

                // sample points   
                std::vector<double> X, Y;
                res = Clothoid::pointsOnClothoid(x0, y0, theta0, k, dk, L, num_seg_samples, X, Y); 

                convert_and_append(X,Y,out_path);

                prev_set_samples += num_seg_samples;
            }
        }

    } else if(m_use_clothoids && !m_use_dubins) {
        // generate a direkt clothoid to target
        double x0 = q0[0];
        double y0 = q0[1];
        double theta0 = q0[2];

        double x1 = q1[0];
        double y1 = q1[1];
        double theta1 = q1[2];

        double k;
        double dk;
        double L;

        int res = Clothoid::buildClothoid(x0, y0, theta0, x1, y1, theta1, k, dk, L);

        length = L;

        std::vector<double> X, Y;
        res = Clothoid::pointsOnClothoid(x0, y0, theta0, k, dk, L, samples, X, Y);

        convert_and_append(X,Y, out_path);

    } else if(!m_use_clothoids && m_use_dubins)
    {

        DubinsPath path;

        dubins_shortest_path(&path, q0.data(),  q1.data() , m_min_turning_radius);

        double path_len = dubins_path_length(&path);

        length = path_len;

        // remove +1?
        double step = path_len / static_cast<double>(samples-1);

        for(int i=0; i<samples; i++)
        {
            std::array<double, 3> q;
            double curr_len = step * i;
            dubins_path_sample(&path, curr_len, q.data() );
            out_path.push_back(q);
        }

    }

    return out_path;
}

void ClothoDub::convert_and_append(const std::vector<double>& X, const std::vector<double>& Y,
        std::vector< std::array<double, 3> >& out)
{
    for(int i=0; i<X.size(); i++)
    {
        std::array<double, 3> q;
        q[0] = X[i];
        q[1] = Y[i];
        // TODO: fix orientation
        q[2] = 0.0;

        out.push_back(q);
    }
}


